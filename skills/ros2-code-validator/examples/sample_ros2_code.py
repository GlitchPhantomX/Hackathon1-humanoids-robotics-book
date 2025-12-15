import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class ExampleNode(Node):
    def __init__(self):
        super().__init__('example_node')
        self.publisher_ = self.create_publisher(String, 'MyTopic', 10)
        self.subscription = self.create_subscription(String, 'input_topic', self.subscription_callback, 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.get_logger().info('Example node initialized')

    def subscription_callback(self, msg):
        # Process message without error handling
        self.process_message(msg)

    def process_message(self, msg):
        msg.processed = True
        self.get_logger().info(f'Processing message with ID {msg.data} and payload for destination')

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.get_clock().now().seconds_nanoseconds()}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    example_node = ExampleNode()

    try:
        rclpy.spin(example_node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()