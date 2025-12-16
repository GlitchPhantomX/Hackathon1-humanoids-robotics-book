import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import statistics


class LaserScanSubscriber(Node):
    """
    Laser scan subscriber example.

    This node subscribes to laser scan data and processes it to
    detect obstacles and publish relevant information.
    """

    def __init__(self):
        """Initialize the laser scan subscriber node."""
        super().__init__('laser_scan_subscriber')

        # Create subscriber for laser scan messages
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )
        self.subscription  # Prevent unused variable warning

        # Create publisher for obstacle information
        self.obstacle_publisher = self.create_publisher(String, 'obstacle_info', 10)

        # Store recent scan data for analysis
        self.scan_history = []

        self.get_logger().info('Laser scan subscriber initialized')

    def scan_callback(self, msg):
        """Process incoming laser scan message."""
        self.get_logger().debug(f'Received scan with {len(msg.ranges)} ranges')

        # Filter out invalid range values
        valid_ranges = [r for r in msg.ranges if msg.range_min <= r <= msg.range_max]

        if valid_ranges:
            # Calculate statistics
            min_distance = min(valid_ranges)
            avg_distance = statistics.mean(valid_ranges) if valid_ranges else float('inf')

            # Check for obstacles in front (within 3 meters in the center 30 degrees)
            center_idx = len(msg.ranges) // 2
            front_ranges = msg.ranges[max(0, center_idx - 8):min(len(msg.ranges), center_idx + 8)]
            front_valid = [r for r in front_ranges if msg.range_min <= r <= 3.0]

            # Create obstacle information message
            obstacle_msg = String()
            if front_valid:
                obstacle_msg.data = f'OBSTACLE_DETECTED: min_dist={min_distance:.2f}m, avg_dist={avg_distance:.2f}m'
                self.get_logger().warn(f'Obstacle detected: {min_distance:.2f}m')
            else:
                obstacle_msg.data = f'NO_OBSTACLE: min_dist={min_distance:.2f}m, avg_dist={avg_distance:.2f}m'
                self.get_logger().info(f'Clear path: {min_distance:.2f}m')

            # Publish obstacle information
            self.obstacle_publisher.publish(obstacle_msg)

            # Store in history (keep last 10 scans)
            self.scan_history.append({
                'timestamp': msg.header.stamp,
                'min_distance': min_distance,
                'avg_distance': avg_distance,
                'obstacles_detected': bool(front_valid)
            })
            if len(self.scan_history) > 10:
                self.scan_history.pop(0)
        else:
            self.get_logger().warn('No valid range data in scan')

    def get_scan_statistics(self):
        """Get statistics about recent scans."""
        if not self.scan_history:
            return "No scan data available"

        min_distances = [s['min_distance'] for s in self.scan_history]
        avg_distances = [s['avg_distance'] for s in self.scan_history]
        obstacle_count = sum(1 for s in self.scan_history if s['obstacles_detected'])

        stats = f"Scan Statistics (last {len(self.scan_history)} scans):\n"
        stats += f"  Min distance - Min: {min(min_distances):.2f}, Avg: {statistics.mean(min_distances):.2f}, Max: {max(min_distances):.2f}\n"
        stats += f"  Avg distance - Min: {min(avg_distances):.2f}, Avg: {statistics.mean(avg_distances):.2f}, Max: {max(avg_distances):.2f}\n"
        stats += f"  Obstacles detected: {obstacle_count}/{len(self.scan_history)} scans"

        return stats


def main(args=None):
    """Main function to run the laser scan subscriber."""
    rclpy.init(args=args)

    try:
        laser_subscriber = LaserScanSubscriber()
        rclpy.spin(laser_subscriber)
    except KeyboardInterrupt:
        # Print statistics when interrupted
        if 'laser_subscriber' in locals():
            print("\n" + laser_subscriber.get_scan_statistics())
    finally:
        if 'laser_subscriber' in locals():
            laser_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()