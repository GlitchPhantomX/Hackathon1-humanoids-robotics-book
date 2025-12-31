---
sidebar_position: 5
title: 'ROS 2 Python پیکیجز: Python نوڈس بنانا اور استعمال کرنا'
description: 'ROS 2 میں Python پیکیجز بنانے، بنانے اور استعمال کرنے کی سمجھ'
---

import ReadingTime from '@site/src/components/ReadingTime';


<ReadingTime minutes={5} />

<h1 className="main-heading">ROS 2 Python پیکیجز: Python نوڈس بنانا اور استعمال کرنا</h1>
<div className="underline-class"></div>

Python روبوٹکس کے لیے مقبول ہے اس کی سادگی اور غنی ماحول کی وجہ سے۔ یہ باب ROS 2 میں Python پیکیجز بنانے اور استعمال کرنے کو کور کرتا ہے۔

<div className="border-line"></div>

<h2 className="second-heading">سیکھنے کے اہداف</h2>
<div className="underline-class"></div>

- • مناسب سٹرکچر کے ساتھ Python پیکیجز بنانا
- • rclpy کا استعمال کرتے ہوئے نوڈس نافذ کرنا
- • پیکیجز بنانا اور انسٹال کرنا
- • Python میں ROS 2 ٹولز استعمال کرنا
- • نوڈس کی ڈیبگنگ اور ٹیسٹنگ کرنا

<div className="border-line"></div>

<h2 className="second-heading">پیکیج سٹرکچر</h2>
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

<h2 className="second-heading">نوڈس بنانا</h2>
<div className="underline-class"></div>

<h3 className="third-heading">بنیادی نوڈ</h3>
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

<h3 className="third-heading">پبلیشر/سبسکرائیبر</h3>
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

<h2 className="second-heading">عام پیٹرنز</h2>
<div className="underline-class"></div>

<h3 className="third-heading">پیرامیٹرز</h3>
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

<h3 className="third-heading">سروسز</h3>
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

<h2 className="second-heading">تعمیر اور چلانا</h2>
<div className="underline-class"></div>
```bash
# تعمیر
cd ~/ros2_ws
colcon build --packages-select my_robot_package
source install/setup.bash

# چلانا
ros2 run my_robot_package my_node
ros2 run my_robot_package my_node --ros-args -p robot_name:=bot
```

<div className="border-line"></div>

<h2 className="second-heading">لاؤنچ فائلز</h2>
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

<h2 className="second-heading">ٹیسٹنگ</h2>
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

<h2 className="second-heading">Python لائبریریز کا استعمال</h2>
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

<h2 className="second-heading">بہترین مشقیں</h2>
<div className="underline-class"></div>

<h3 className="third-heading">کوڈ کی تنظیم</h3>
<div className="underline-class"></div>

- • ہر نوڈ کے لیے واحد ذمہ داری
- • تنظیم کے لیے ماڈیولز استعمال کریں
- • PEP 8 کو فالو کریں
- • ڈاک سٹرنگز کے ساتھ دستاویز کریں

<h3 className="third-heading">غلطی کا انتظام</h3>
<div className="underline-class"></div>

- • try-except بلاکس استعمال کریں
- • finally میں کلین اپ کریں
- • غلطیوں کو مناسب طریقے سے لاگ کریں

<h3 className="third-heading">کارکردگی</h3>
<div className="underline-class"></div>

- • کال بیکس میں بلاکنگ سے بچیں
- • کال بیک گروپس استعمال کریں
- • میموری کو مانیٹر کریں

<h3 className="third-heading">ڈیبگنگ</h3>
<div className="underline-class"></div>

- • `self.get_logger().info()` استعمال کریں
- • `ros2 node list`, `ros2 topic list` کے ساتھ چیک کریں
- • `rqt_console` استعمال کریں

<div className="border-line"></div>

<h2 className="second-heading">ٹربل شوٹنگ</h2>
<div className="underline-class"></div>

<h3 className="third-heading">امپورٹ کی غلطیاں</h3>
<div className="underline-class"></div>

- • مناسب طریقے سے تعمیر کریں اور سورس کریں
- • `__init__.py` فائلز چیک کریں
- • PYTHONPATH کی تصدیق کریں

<h3 className="third-heading">اجازت کے مسائل</h3>
<div className="underline-class"></div>

- • روٹ کے طور پر نہ چلائیں
- • فائل کی اجازت چیک کریں
- • انسٹالیشن کی تصدیق کریں

<h3 className="third-heading">مواصلاتی مسائل</h3>
<div className="underline-class"></div>

- • ڈومین چیک کریں
- • ٹاپک/سروس کے نام کی تصدیق کریں
- • مناسب QoS کو یقینی بنائیں

<h2 className="second-heading">خلاصہ</h2>
<div className="underline-class"></div>

ROS 2 میں Python پیکیجز نوڈس کی تخلیق کے لیے rclpy استعمال کرتے ہیں جس میں پبلیشرز، سبسکرائیبرز، سروسز، اور پیرامیٹرز شامل ہیں۔ مناسب سٹرکچر، ٹیسٹنگ، اور بہترین مشقیں استعمال کریں۔