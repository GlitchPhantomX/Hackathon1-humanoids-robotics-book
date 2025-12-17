---
sidebar_position: 5
title: 'ROS 2 Python Packages: Building and Using Python Nodes'
description: 'Understanding how to create, build, and use Python packages in ROS 2'
---

import ReadingTime from '@site/src/components/ReadingTime';

<ReadingTime minutes={5} />

<h1 className="main-heading">ROS 2 Python Packages: Building and Using Python Nodes</h1>
<div className="underline-class"></div>

Python is popular for robotics due to its simplicity and rich ecosystem. This chapter covers creating and using Python packages in ROS 2.

<div className="border-line"></div>

<h2 className="second-heading">Learning Objectives</h2>
<div className="underline-class"></div>

- • Create Python packages with proper structure
- • Implement nodes using rclpy
- • Build and install packages
- • Use ROS 2 tools in Python
- • Debug and test nodes

<div className="border-line"></div>

<h2 className="second-heading">Package Structure</h2>
<div className="underline-class"></div>
```
my_robot_package/
├── package.xml
├── setup.py
├── setup.cfg
├── my_robot_package/
│   ├── __init__.py
│   └── my_node.py
└── test/
```

<h3 className="third-heading">package.xml</h3>
<div className="underline-class"></div>
```xml
<package format="3">
  <name>my_robot_package</name>
  <version>0.0.0</version>
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <export><build_type>ament_python</build_type></export>
</package>
```

<h3 className="third-heading">setup.py</h3>
<div className="underline-class"></div>
```python
from setuptools import setup

setup(
    name='my_robot_package',
    packages=['my_robot_package'],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'my_node = my_robot_package.my_node:main',
        ],
    },
)
```

<div className="border-line"></div>

<h2 className="second-heading">Creating Nodes</h2>
<div className="underline-class"></div>

<h3 className="third-heading">Basic Node</h3>
<div className="underline-class"></div>
```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.get_logger().info('Node initialized')

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

<h3 className="third-heading">Publisher/Subscriber</h3>
<div className="underline-class"></div>
```python
from std_msgs.msg import String

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.publisher = self.create_publisher(String, 'status', 10)
        self.subscription = self.create_subscription(String, 'cmd', self.callback, 10)
        self.timer = self.create_timer(1.0, self.publish_status)
    
    def callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')
    
    def publish_status(self):
        msg = String()
        msg.data = 'OK'
        self.publisher.publish(msg)
```

<div className="border-line"></div>

<h2 className="second-heading">Common Patterns</h2>
<div className="underline-class"></div>

<h3 className="third-heading">Parameters</h3>
<div className="underline-class"></div>
```python
class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')
        self.declare_parameter('robot_name', 'my_robot')
        self.declare_parameter('max_velocity', 1.0)
        
        self.robot_name = self.get_parameter('robot_name').value
        self.max_velocity = self.get_parameter('max_velocity').value
```

<h3 className="third-heading">Services</h3>
<div className="underline-class"></div>
```python
from std_srvs.srv import SetBool

class ServiceNode(Node):
    def __init__(self):
        super().__init__('service_node')
        self.srv = self.create_service(SetBool, 'enable', self.callback)
    
    def callback(self, request, response):
        response.success = True
        response.message = f'Enabled: {request.data}'
        return response
```

<div className="border-line"></div>

<h2 className="second-heading">Building & Running</h2>
<div className="underline-class"></div>
```bash
# Build
cd ~/ros2_ws
colcon build --packages-select my_robot_package
source install/setup.bash

# Run
ros2 run my_robot_package my_node
ros2 run my_robot_package my_node --ros-args -p robot_name:=bot
```

<div className="border-line"></div>

<h2 className="second-heading">Launch Files</h2>
<div className="underline-class"></div>
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_package',
            executable='my_node',
            parameters=[{'robot_name': 'bot'}],
            output='screen'
        )
    ])
```

<div className="border-line"></div>

<h2 className="second-heading">Testing</h2>
<div className="underline-class"></div>
```python
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
    
    def test_initialization(self):
        self.assertIsNotNone(self.node)
```

<div className="border-line"></div>

<h2 className="second-heading">Using Python Libraries</h2>
<div className="underline-class"></div>
```python
import numpy as np
import cv2
import yaml

class AdvancedNode(Node):
    def __init__(self):
        super().__init__('advanced_node')
        self.data = np.random.random(10)
        
        with open('config.yaml', 'r') as f:
            self.config = yaml.safe_load(f)
```

<div className="border-line"></div>

<h2 className="second-heading">Best Practices</h2>
<div className="underline-class"></div>

<h3 className="third-heading">Code Organization</h3>
<div className="underline-class"></div>

- • Single responsibility per node
- • Use modules for organization
- • Follow PEP 8
- • Document with docstrings

<h3 className="third-heading">Error Handling</h3>
<div className="underline-class"></div>

- • Use try-except blocks
- • Cleanup in finally
- • Log errors properly

<h3 className="third-heading">Performance</h3>
<div className="underline-class"></div>

- • Avoid blocking in callbacks
- • Use callback groups
- • Monitor memory

<h3 className="third-heading">Debugging</h3>
<div className="underline-class"></div>

- • Use `self.get_logger().info()`
- • Check with `ros2 node list`, `ros2 topic list`
- • Use `rqt_console`

<div className="border-line"></div>

<h2 className="second-heading">Troubleshooting</h2>
<div className="underline-class"></div>

<h3 className="third-heading">Import Errors</h3>
<div className="underline-class"></div>

- • Build and source properly
- • Check `__init__.py` files
- • Verify PYTHONPATH

<h3 className="third-heading">Permission Issues</h3>
<div className="underline-class"></div>

- • Don't run as root
- • Check file permissions
- • Verify installation

<h3 className="third-heading">Communication Issues</h3>
<div className="underline-class"></div>

- • Check domain
- • Verify topic/service names
- • Ensure proper QoS

<h2 className="second-heading">Summary</h2>
<div className="underline-class"></div>

Python packages in ROS 2 use rclpy for node creation with publishers, subscribers, services, and parameters. Use proper structure, testing, and best practices.