---
sidebar_position: 5
title: 'ROS 2 Python Packages: Building and Using Python Nodes'
description: 'Understanding how to create, build, and use Python packages in ROS 2'
---

# ROS 2 Python Packages: Building and Using Python Nodes

Python is one of the most popular languages for robotics development due to its simplicity and rich ecosystem. This chapter covers how to create, structure, and use Python packages in ROS 2.

## Learning Objectives

By the end of this chapter, you will be able to:
- Create ROS 2 Python packages with proper structure
- Implement nodes using the rclpy client library
- Build and install Python packages in ROS 2
- Use ROS 2 tools and utilities in Python
- Debug and test Python-based ROS 2 nodes

## Package Structure and Organization

A ROS 2 Python package follows a specific structure that enables proper building, installation, and execution.

### Basic Package Structure

```
my_robot_package/
├── CMakeLists.txt          # For mixed C++/Python packages
├── package.xml             # Package metadata
├── setup.cfg               # Python installation configuration
├── setup.py                # Python package setup
├── resource/my_robot_package  # Executable marker (optional)
├── my_robot_package/       # Python module
│   ├── __init__.py
│   ├── my_node.py
│   └── my_module.py
└── test/                   # Test files
    └── test_my_node.py
```

### Package Metadata (package.xml)

The `package.xml` file contains metadata about the package:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_robot_package</name>
  <version>0.0.0</version>
  <description>My robot package for ROS 2</description>
  <maintainer email="user@example.com">User Name</maintainer>
  <license>Apache License 2.0</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### Python Package Setup (setup.py)

The `setup.py` file defines how the Python package is installed:

```python
from setuptools import setup
from glob import glob
import os

package_name = 'my_robot_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='My robot package for ROS 2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = my_robot_package.my_node:main',
            'another_node = my_robot_package.another_node:main',
        ],
    },
)
```

## Creating Python Nodes

### Basic Node Structure

```python
#!/usr/bin/env python3
"""
A basic ROS 2 Python node example
"""
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node_name')
        self.get_logger().info('MyNode has been initialized')

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Node with Publishers and Subscribers

```python
#!/usr/bin/env python3
"""
A ROS 2 Python node with publishers and subscribers
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        # Create publisher
        self.publisher = self.create_publisher(String, 'robot_status', 10)

        # Create subscriber
        self.subscription = self.create_subscription(
            LaserScan,
            'laser_scan',
            self.laser_callback,
            10
        )

        # Create timer
        self.timer = self.create_timer(1.0, self.publish_status)

        self.status_msg = String()
        self.status_msg.data = 'Robot operational'

        self.get_logger().info('Robot controller initialized')

    def laser_callback(self, msg):
        # Process laser scan data
        if msg.ranges:
            min_range = min(msg.ranges)
            if min_range < 1.0:
                self.status_msg.data = f'Obstacle detected: {min_range:.2f}m'
            else:
                self.status_msg.data = f'Path clear: {min_range:.2f}m'
        else:
            self.status_msg.data = 'No laser data'

    def publish_status(self):
        self.publisher.publish(self.status_msg)
        self.get_logger().info(f'Status: {self.status_msg.data}')

def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotController()

    try:
        rclpy.spin(robot_controller)
    except KeyboardInterrupt:
        pass
    finally:
        robot_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Common ROS 2 Python Patterns

### Using Parameters

```python
#!/usr/bin/env python3
"""
Node demonstrating parameter usage
"""
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')

        # Declare parameters with default values
        self.declare_parameter('robot_name', 'my_robot')
        self.declare_parameter('max_velocity', 1.0)
        self.declare_parameter('safety_distance', 0.5)

        # Get parameter values
        self.robot_name = self.get_parameter('robot_name').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.safety_distance = self.get_parameter('safety_distance').value

        # Create callback for parameter changes
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.get_logger().info(
            f'Robot: {self.robot_name}, '
            f'Max velocity: {self.max_velocity}, '
            f'Safety distance: {self.safety_distance}'
        )

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'max_velocity' and param.value > 5.0:
                return SetParametersResult(successful=False, reason='Max velocity too high')
        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    node = ParameterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Using Services

```python
#!/usr/bin/env python3
"""
Node demonstrating service usage
"""
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from example_interfaces.srv import Trigger

class ServiceNode(Node):
    def __init__(self):
        super().__init__('service_node')

        # Create service servers
        self.enable_service = self.create_service(
            SetBool,
            'enable_robot',
            self.enable_callback
        )

        self.reset_service = self.create_service(
            Trigger,
            'reset_robot',
            self.reset_callback
        )

        self.robot_enabled = False
        self.get_logger().info('Service node initialized')

    def enable_callback(self, request, response):
        self.robot_enabled = request.data
        response.success = True
        response.message = f'Robot {"enabled" if self.robot_enabled else "disabled"}'
        self.get_logger().info(response.message)
        return response

    def reset_callback(self, request, response):
        # Perform reset operations
        self.get_logger().info('Resetting robot...')
        response.success = True
        response.message = 'Robot reset completed'
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ServiceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Building and Installing Python Packages

### Building the Package

To build a Python package in ROS 2:

```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash  # or your ROS 2 distribution

# Navigate to your workspace
cd ~/ros2_ws

# Build only this package
colcon build --packages-select my_robot_package

# Or build all packages
colcon build
```

### Sourcing the Package

After building, source your workspace:

```bash
source ~/ros2_ws/install/setup.bash
```

### Running Python Nodes

Once built and sourced, you can run your nodes:

```bash
# Run the node directly
ros2 run my_robot_package my_node

# Run with parameters
ros2 run my_robot_package my_node --ros-args -p robot_name:=turtlebot -p max_velocity:=2.0

# Use ros2 launch (if you have launch files)
ros2 launch my_robot_package my_launch_file.launch.py
```

## Advanced Python Topics

### Using Launch Files

Create a launch file to run multiple nodes together:

```python
# launch/my_launch_file.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_package',
            executable='my_node',
            name='my_node',
            parameters=[
                {'robot_name': 'turtlebot'},
                {'max_velocity': 1.0}
            ],
            output='screen'
        ),
        Node(
            package='my_robot_package',
            executable='another_node',
            name='another_node',
            output='screen'
        )
    ])
```

### Testing Python Nodes

Create unit tests for your nodes:

```python
# test/test_my_node.py
import unittest
import rclpy
from my_robot_package.my_node import MyNode

class TestMyNode(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = MyNode()

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def test_node_initialization(self):
        self.assertIsNotNone(self.node)
        # Add more specific tests here

if __name__ == '__main__':
    unittest.main()
```

### Using Python Tools and Libraries

ROS 2 Python nodes can leverage the rich Python ecosystem:

```python
#!/usr/bin/env python3
"""
Node using external Python libraries
"""
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32MultiArray
import cv2  # OpenCV
import yaml  # YAML parsing

class AdvancedNode(Node):
    def __init__(self):
        super().__init__('advanced_node')

        # Example: Using numpy for mathematical operations
        self.publisher = self.create_publisher(Float32MultiArray, 'math_result', 10)
        self.timer = self.create_timer(1.0, self.compute_and_publish)

        # Example: Loading configuration from YAML
        config_path = self.get_package_share_directory('my_robot_package') + '/config/config.yaml'
        with open(config_path, 'r') as file:
            self.config = yaml.safe_load(file)

        self.get_logger().info('Advanced node initialized')

    def compute_and_publish(self):
        # Example computation using numpy
        data = np.random.random(10).astype(np.float32)
        msg = Float32MultiArray()
        msg.data = data.tolist()
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = AdvancedNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Best Practices

### Code Organization
- Keep nodes focused on single responsibilities
- Use modules to organize related functionality
- Follow Python naming conventions (PEP 8)
- Document your code with docstrings

### Error Handling
- Use try-except blocks for robust error handling
- Implement proper cleanup in finally blocks
- Log errors appropriately for debugging

### Performance Considerations
- Avoid blocking operations in callbacks
- Use appropriate callback groups for concurrency
- Monitor memory usage in long-running nodes

### Debugging Tips
- Use `self.get_logger().info()` for debugging output
- Check ROS 2 command line tools: `ros2 node list`, `ros2 topic list`
- Use `rqt_console` for viewing logs from multiple nodes

## Troubleshooting Common Issues

### Import Errors
**Problem**: Module not found errors
**Solutions**:
- Verify package is properly built and sourced
- Check that `__init__.py` files exist
- Ensure PYTHONPATH includes your package

### Permission Issues
**Problem**: Permission denied when running nodes
**Solutions**:
- Don't run ROS 2 as root
- Check file permissions on your workspace
- Verify ROS 2 installation permissions

### Node Communication Issues
**Problem**: Nodes cannot communicate
**Solutions**:
- Check that nodes are in the same domain
- Verify topic/service names match
- Ensure proper QoS settings

import ReadingTime from '@site/src/components/ReadingTime';
import ViewToggle from '@site/src/components/ViewToggle';

<ReadingTime minutes={16} />
<ViewToggle />