import rclpy
from rclpy.node import Node
from order_interfaces.srv import GetQueuedOrders
from order_interfaces.srv import UpdateOrderStatus
import time
import threading

QUEUED = 0
IN_PROGRESS = 1
COMPLETED = 2
ERROR = 3

class BaristaBot(Node):
    def __init__(self):
        super().__init__('barista_bot')
        self.get_queued_client = self.create_client(GetQueuedOrders, '/get_queued_orders')

        self.update_status_client = self.create_client(UpdateOrderStatus, '/update_order_status')

        self.get_logger().info("BaristaBot启动")

        self.worker_thread = threading.Thread(target=self.process_orders)
        self.worker_thread.daemon = True
        self.worker_thread.start()

        self.timer = self.create_timer(3.0, self.get_orders)
    
    def get_orders(self):
        if not self.worker_thread.is_alive():

            self.worker_thread = threading.Thread(target=self.process_orders)
            self.worker_thread.daemon = True
            self.worker_thread.start()



    def process_orders(self):
        while rclpy.ok():
            try:
                response = self.get_queued_orders()
                if response is None:
                    time.sleep(1.0)
                    continue
                
                order_ids = getattr(response, 'order_ids', None)
                items_list = getattr(response, 'items_list', None)

                if not order_ids or len(order_ids) == 0:
                    self.get_logger().info("当前没有排队订单，稍后重试...")
                    time.sleep(1.0)
                    continue

                order_id = order_ids[0]
                order_items = items_list[0] if items_list else ""
                if not order_items:
                    self.get_logger().info(f"订单{order_id}没有物品，跳过")
                    time.sleep(0.5)
                    continue

                order_items = order_items.split(',')

                self.get_logger().info(f"开始处理订单: {order_id},{order_items}")
            
                try:
                    self.call_update_order_status(order_id, IN_PROGRESS)
                except Exception:
                    self.get_logger().warning("更新订单状态失败（IN_PROGRESS）")
                           
        
                for coffee in order_items:
                    self.get_logger().info(f"制作中: {coffee.strip()}")
                    time.sleep(5)

                self.get_logger().info(f"订单{order_id}完成")
                try:
                    self.call_update_order_status(order_id, COMPLETED)
                except Exception:
                    self.get_logger().warning("更新订单状态失败（COMPLETED）")
            except Exception as information:
                self.get_logger().warning(f'订单处理失败{str(information)}')
                time.sleep(0.5)

    def get_queued_orders(self):
        try:
            if not self.get_queued_client.wait_for_service(1.0):
                self.get_logger().warning('等待获取订单......')
                time.sleep(5)
                return None
            request = GetQueuedOrders.Request()
            future = self.get_queued_client.call_async(request)
            while rclpy.ok() and not future.done():
                time.sleep(0.1)
            if not future.done():
                return None
            return future.result()
        except Exception as information:
            self.get_logger().warning(f'获取订单失败{str(information)}')

    def call_update_order_status(self, order_id: int, new_status: int) -> bool:
        """Call the UpdateOrderStatus service and return the success boolean.

        Blocks briefly waiting for service and response (suitable for worker thread).
        """
        try:
            if not self.update_status_client.wait_for_service(1.0):
                self.get_logger().warning('等待更新订单状态...')
                time.sleep(1.0)
                return False
            req = UpdateOrderStatus.Request()
            req.order_id = int(order_id)
            req.new_status = int(new_status)
            future = self.update_status_client.call_async(req)
            while rclpy.ok() and not future.done():
                time.sleep(0.05)
            if not future.done():
                return False
            resp = future.result()
            return getattr(resp, 'success', False)
        except Exception as e:
            self.get_logger().warning(f'更新订单状态失败: {e}')
            return False


def main():
    rclpy.init()
    node = BaristaBot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()