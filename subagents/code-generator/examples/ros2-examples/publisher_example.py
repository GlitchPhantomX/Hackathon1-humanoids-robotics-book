import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import math


class LaserScanPublisher(Node):
    """
    Laser scan publisher example.

    This node publishes simulated laser scan data to demonstrate
    sensor data publishing in ROS2.
    """

    def __init__(self):
        """Initialize the laser scan publisher node."""
        super().__init__('laser_scan_publisher')

        # Create publisher for laser scan messages
        self.publisher_ = self.create_publisher(LaserScan, 'scan', 10)

        # Create timer to publish messages at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_scan)

        # Laser scan parameters
        self.angle_min = -math.pi / 2  # -90 degrees
        self.angle_max = math.pi / 2   # 90 degrees
        self.angle_increment = math.pi / 180  # 1 degree
        self.scan_time = 1.0 / 10.0   # 10 Hz
        self.range_min = 0.1
        self.range_max = 10.0

        # Calculate number of ranges
        self.num_ranges = int((self.angle_max - self.angle_min) / self.angle_increment) + 1

        self.get_logger().info('Laser scan publisher initialized')

    def publish_scan(self):
        """Publish a simulated laser scan message."""
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser_frame'

        # Set laser scan parameters
        msg.angle_min = self.angle_min
        msg.angle_max = self.angle_max
        msg.angle_increment = self.angle_increment
        msg.scan_time = self.scan_time
        msg.time_increment = 0.0
        msg.range_min = self.range_min
        msg.range_max = self.range_max

        # Generate simulated range data
        ranges = []
        for i in range(self.num_ranges):
            # Simulate a wall at 2 meters distance at 0 degrees
            angle = self.angle_min + i * self.angle_increment
            if abs(angle) < math.pi / 8:  # Within 22.5 degrees of center
                ranges.append(2.0)  # Wall at 2 meters
            else:
                ranges.append(self.range_max)  # No obstacle

        msg.ranges = ranges
        msg.intensities = [100.0] * len(ranges)  # Constant intensity

        self.publisher_.publish(msg)
        self.get_logger().debug(f'Published laser scan with {len(ranges)} ranges')


def main(args=None):
    """Main function to run the laser scan publisher."""
    rclpy.init(args=args)

    try:
        laser_publisher = LaserScanPublisher()
        rclpy.spin(laser_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        if 'laser_publisher' in locals():
            laser_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()