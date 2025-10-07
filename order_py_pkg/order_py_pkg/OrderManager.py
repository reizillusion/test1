import rclpy
from rclpy.node import Node
import threading
import time
from order_interfaces.action import Order
from rclpy.action import ActionServer, GoalResponse
from order_interfaces.srv import GetQueuedOrders
from order_interfaces.srv import UpdateOrderStatus


QUEUED = 0
IN_PROGRESS = 1
COMPLETED = 2
ERROR = 3


class OrderManager(Node):
    def __init__(self):
        super().__init__('order_manager')
        self.lock = threading.Lock()

        
        self.action_server = ActionServer(
            self,
            Order,
            'submit_order',
            execute_callback=self.submit_order_callback,
            goal_callback=self.goal_callback
        )
        
        self.get_queued_srv = self.create_service(GetQueuedOrders, '/get_queued_orders', self.get_queued_callback)
        self.update_status_srv = self.create_service(UpdateOrderStatus, '/update_order_status', self.update_status_callback)

        self.order_number = 0
        self.order_dic = {}       
        self.timer = self.create_timer(3.0, self.order_moniter)

        self.get_logger().info("OrderManager启动")
    
    def goal_callback(self, goal_request):
        """处理新目标的回调函数"""
        self.get_logger().info(f"收到新订单请求: {goal_request.order_items}")
        return GoalResponse.ACCEPT
    
    def submit_order_callback(self, goal_handle):
        """处理已接受目标的执行"""
        with self.lock:
            order_items = goal_handle.request.order_items
            order_id = self.order_number

            self.order_dic[order_id] = {
                'order_items': order_items,
                'status': QUEUED,
                'goal_handle': goal_handle,
                'last_update_time': time.time()
            }

            self.order_number += 1
        
        self.get_logger().info(f"订单号{order_id}已提交订单：{order_items}")

        feedback_msg = Order.Feedback()
        feedback_msg.feedback = f'订单{order_id}已接收，共 {len(order_items)} 个商品'
        goal_handle.publish_feedback(feedback_msg)

        
        while rclpy.ok():
            with self.lock:
                status = self.order_dic[order_id]['status']
                if status == COMPLETED or status == ERROR:
                    break
            
            time.sleep(0.1) 
        
     
        result = Order.Result()
        if status == COMPLETED:
            result.order_result = f"订单{order_id}已完成"
        else:
            result.order_result = f"订单{order_id}失败"
        
        return result

    def get_queued_callback(self, request, response):
        with self.lock:
            response.order_ids = []
            response.items_list = []
            for order_id, order_info in self.order_dic.items():
                if order_info['status'] == QUEUED:
                    response.order_ids.append(order_id)
                    items_str = ','.join(order_info['order_items'])                    
                    response.items_list.append(items_str)
        return response

    def update_order_status(self, order_id, status):
        with self.lock:
            if order_id in self.order_dic:
                self.order_dic[order_id]['status'] = status
                self.order_dic[order_id]['last_update_time'] = time.time()
                return True
            return False

    def update_status_callback(self, request, response):
        order_id = request.order_id
        new_status = request.new_status
        response.success = self.update_order_status(order_id, new_status)
        return response
    
    def reset_order(self, order_id):
        with self.lock:
            self.order_dic[order_id]['status'] = QUEUED
            timenow = time.time()
            self.order_dic[order_id]['last_update_time'] = timenow
            self.get_logger().warning(f'订单{order_id}重置')

    def order_moniter(self):
        timeout = 15
        timenow = time.time()
        with self.lock:
            for id, order in list(self.order_dic.items()):
                if order['status'] == IN_PROGRESS and timenow - order['last_update_time'] > timeout:
                    self.reset_order(id)
                    self.get_logger().warning(f'订单{id}超时重置')
                    order['last_update_time'] = timenow


def main():
    rclpy.init()
    node = OrderManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()    
        rclpy.shutdown() 
        
if __name__ == '__main__':
    main()