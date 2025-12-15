import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalNode(Node):
    """
    Minimal ROS2 node example.

    This node demonstrates the basic structure of a ROS2 node
    with proper initialization, parameters, and lifecycle management.
    """

    def __init__(self):
        """Initialize the minimal node."""
        super().__init__('minimal_node')

        # Declare parameters with default values
        self.declare_parameter('example_param', 'default_value')

        # Get parameter value
        self.param_value = self.get_parameter('example_param').value

        # Create a publisher
        self.publisher_ = self.create_publisher(String, 'example_topic', 10)

        # Create a timer to periodically publish messages
        self.timer = self.create_timer(0.5, self.timer_callback)

        self.get_logger().info(f'Minimal node initialized with param: {self.param_value}')

    def timer_callback(self):
        """Timer callback to publish messages."""
        msg = String()
        msg.data = f'Hello from minimal node: {self.param_value}'
        self.publisher_.publish(msg)
        self.get_logger().debug(f'Published: {msg.data}')

    def destroy_node(self):
        """Clean up before node destruction."""
        self.get_logger().info('Shutting down minimal node...')
        super().destroy_node()


def main(args=None):
    """Main function to run the minimal node."""
    rclpy.init(args=args)

    try:
        minimal_node = MinimalNode()
        rclpy.spin(minimal_node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'minimal_node' in locals():
            minimal_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()