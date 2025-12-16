#!/usr/bin/env python3
"""
Invalid ROS2 Node Example - Demonstrates common ROS2 code issues and violations.

This example shows a poorly-structured ROS2 node with multiple issues:
- Missing proper imports and structure
- Poor parameter handling
- Incorrect QoS settings
- Missing error handling
- Bad resource management
- Violations of ROS2 conventions
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from example_interfaces.srv import AddTwoInts
import time
# Importing everything from rclpy (not recommended)
from rclpy import *


class BadRos2Node(Node):
    """
    A poorly-structured ROS2 node demonstrating multiple violations of best practices.

    This node has numerous issues including:
    - Poor parameter handling
    - Missing QoS configurations
    - No error handling
    - Bad resource management
    - Violations of ROS2 naming conventions
    """

    def __init__(self):
        """Initialize the node with improper parameter handling."""
        super().__init__('bad_ros2_node')

        # Missing parameter declaration - should declare all parameters
        # self.declare_parameter('publish_frequency', 1.0)  # This line is commented out, showing the issue

        # Getting parameter without checking if it exists
        self.publish_frequency = self.get_parameter('publish_frequency').value  # This will fail
        self.laser_threshold = 1.0  # Not using parameters properly

        # No QoS profile specified - using default which may not be appropriate
        self.publisher = self.create_publisher(String, 'bad_topic_name', 10)

        # No QoS profile for subscriber either
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            10  # No QoS profile specified
        )

        # Service without proper callback group
        self.service = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback
        )

        # Timer with hardcoded value instead of using parameter
        self.timer = self.create_timer(1.0, self.timer_callback)

        # Global variables (bad practice)
        global bad_global_counter
        bad_global_counter = 0

        # No logging initialization or proper messaging
        print("Node initialized")  # Using print instead of proper logging

    def laser_callback(self, msg):
        """
        Handle incoming laser scan messages without error handling.

        This function has multiple issues:
        - No type hints
        - No error handling
        - Direct modification of class state without safety checks
        """
        # No validation of message content
        self.last_laser_ranges = msg.ranges  # Direct assignment without validation

        # No error handling - if ranges is empty or invalid, this will crash
        min_range = min(self.last_laser_ranges)  # Could crash with empty list

        # Direct comparison without checking for infinity or invalid values
        if min_range < self.laser_threshold:
            print(f'Obstacle at {min_range}m')  # Using print instead of logging

    def timer_callback(self):
        """
        Timer callback with multiple issues.

        Issues include:
        - No error handling
        - Global variable usage
        - No validation of publisher state
        """
        global bad_global_counter

        # No check if node is enabled or should continue
        # Directly modifying global state
        bad_global_counter += 1

        # Creating message without error handling
        msg = String()
        msg.data = f'Bad node message #{bad_global_counter}'

        # Publishing without checking if publisher is valid
        self.publisher.publish(msg)

        # Using print instead of proper logging
        if bad_global_counter % 10 == 0:
            print(f'Published {bad_global_counter} messages')

    def add_two_ints_callback(self, request, response):
        """
        Service callback with multiple issues.

        Issues include:
        - No type hints
        - No error handling
        - No input validation
        - No proper logging
        """
        # No input validation - could cause errors
        # No error handling if request.a or request.b don't exist
        result = request.a + request.b  # Could fail if a or b are not numbers

        response.sum = result

        # Using print instead of proper logging
        print(f'Calculated {request.a} + {request.b} = {result}')

        return response  # Missing return in some code paths

    def dangerous_function(self):
        """
        Function demonstrating dangerous practices.

        Issues include:
        - No error handling
        - Direct system calls
        - Poor resource management
        """
        import os
        # Direct system calls without validation
        os.system("rm -rf /tmp")  # Extremely dangerous - never do this

        # No cleanup of resources
        file_handle = open("/tmp/bad_file.txt", "w")  # No context manager
        file_handle.write("Dangerous operation")  # File never closed properly
        # file_handle.close()  # This line is commented out, showing the issue


def main(args=None):
    """
    Main function with issues.

    Issues include:
    - No proper exception handling
    - No resource cleanup
    - No graceful shutdown
    """
    # No try-catch block
    rclpy.init(args=args)

    bad_ros2_node = BadRos2Node()

    # No executor specified - using default single-threaded
    rclpy.spin(bad_ros2_node)  # This blocks indefinitely without proper shutdown handling

    # No cleanup code - these lines are commented out showing the issue
    # bad_ros2_node.destroy_node()
    # rclpy.shutdown()


# Global code execution outside of main - bad practice
if __name__ == '__main__':
    main()  # This executes immediately when imported