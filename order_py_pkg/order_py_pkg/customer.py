import rclpy
from rclpy.node import Node
import random
import time
from order_interfaces.action import Order
from rclpy.action import ActionClient

COFFEE_TYPES = [
    "latte", "cappuccino", "espresso", "americano", 
    "macchiato", "mocha", "flat white", "cold brew",
    "frappuccino", "affogato", "ristretto", "doppio",
    "max coffee","beiyou latte"
]

class Customer(Node):
    def __init__(self):
        super().__init__('custom')
        self.action_client = ActionClient(self,Order,'submit_order')
        self.timer = self.create_timer(
            random.uniform(5.0, 15.0), 
            self.submit_random_order
        )
        self.get_logger().info("custom启动,准备提交订单...")


    def submit_random_order(self):
            order_items = random.sample(
                COFFEE_TYPES, 
                random.randint(1, 4)
            )

            goal_msg = Order.Goal()
           
            goal_msg.order_items = order_items
    
           
            try:
                future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
            except TypeError:
                future = self.action_client.send_goal_async(goal_msg)
           
            future.add_done_callback(self.goal_response_callback)
            

            self.timer.cancel()
            self.timer = self.create_timer(
                random.uniform(5.0, 15.0), 
                self.submit_random_order
            )

    def goal_response_callback(self, future):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().info('Goal rejected')
                return
            self.get_logger().info('Goal accepted')
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.get_result_callback)
        except Exception as e:
            self.get_logger().warning(f'goal_response_callback error: {e}')

    def feedback_callback(self, feedback_msg):
        try:
            feedback = feedback_msg.feedback
            status = getattr(feedback, 'status', '')
            completed = getattr(feedback, 'completed_items', None)
            total = getattr(feedback, 'total_items', None)
            self.get_logger().info(f'Feedback: {status}; {completed}/{total}')
        except Exception as e:
            self.get_logger().warning(f'feedback_callback error: {e}')

    def get_result_callback(self, future):
        try:
            res = future.result()
            result = getattr(res, 'result', None)
            if result is None:
                self.get_logger().warning('Result future returned no result')
                return
            success = getattr(result, 'success', False)
            message = getattr(result, 'message', '')
            self.get_logger().info(f'Result: success={success}, message={message}')
        except Exception as e:
            self.get_logger().warning(f'get_result_callback error: {e}')


def main():
    rclpy.init()
    customer_node = Customer()
    
    try:
        rclpy.spin(customer_node)
    except KeyboardInterrupt:
        pass
    finally:
        customer_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    