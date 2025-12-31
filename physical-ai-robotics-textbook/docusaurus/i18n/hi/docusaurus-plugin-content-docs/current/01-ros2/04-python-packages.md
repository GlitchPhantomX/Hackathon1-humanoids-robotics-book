---
sidebar_position: 5
title: 'आरओएस 2 पायथन पैकेज: पायथन नोड्स बनाना और उपयोग करना'
description: 'आरओएस 2 में पायथन पैकेज बनाने, बनाने और उपयोग करने की समझ'
---

import ReadingTime from '@site/src/components/ReadingTime';

<ReadingTime minutes={5} />

<h1 className="main-heading">आरओएस 2 पायथन पैकेज: पायथन नोड्स बनाना और उपयोग करना</h1>
<div className="underline-class"></div>

पायथन रोबोटिक्स के लिए लोकप्रिय है क्योंकि इसकी सरलता और समृद्ध पारिस्थितिकी। यह अध्याय आरओएस 2 में पायथन पैकेज बनाने और उपयोग करना को कवर करता है।

<div className="border-line"></div>

<h2 className="second-heading">सीखने के उद्देश्य</h2>
<div className="underline-class"></div>

- • उचित संरचना के साथ पायथन पैकेज बनाएं
- • आरसीएलपीवाई का उपयोग करके नोड्स लागू करें
- • पैकेज बनाएं और स्थापित करें
- • पायथन में आरओएस 2 उपकरणों का उपयोग करें
- • नोड्स का डिबग और परीक्षण करें

<div className="border-line"></div>

<h2 className="second-heading">पैकेज संरचना</h2>
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

<h2 className="second-heading">नोड्स बनाना</h2>
<div className="underline-class"></div>

<h3 className="third-heading">मूल नोड</h3>
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

<h3 className="third-heading">प्रकाशक/सदस्य</h3>
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

<h2 className="second-heading">सामान्य पैटर्न</h2>
<div className="underline-class"></div>

<h3 className="third-heading">पैरामीटर</h3>
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

<h3 className="third-heading">सेवाएं</h3>
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

<h2 className="second-heading">बिल्डिंग और चलाना</h2>
<div className="underline-class"></div>
```bash
# बिल्ड
cd ~/ros2_ws
colcon build --packages-select my_robot_package
source install/setup.bash

# चलाएं
ros2 run my_robot_package my_node
ros2 run my_robot_package my_node --ros-args -p robot_name:=bot
```

<div className="border-line"></div>

<h2 className="second-heading">लॉन्च फाइलें</h2>
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

<h2 className="second-heading">परीक्षण</h2>
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

<h2 className="second-heading">पायथन लाइब्रेरीज़ का उपयोग करना</h2>
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

<h2 className="second-heading">सर्वोत्तम प्रथाएं</h2>
<div className="underline-class"></div>

<h3 className="third-heading">कोड संगठन</h3>
<div className="underline-class"></div>

- • प्रति नोड एकल जिम्मेदारी
- • संगठन के लिए मॉडल का उपयोग करें
- • पीईपी 8 का पालन करें
- • डॉकस्ट्रिंग के साथ दस्तावेज़ करें

<h3 className="third-heading">त्रुटि संभाल</h3>
<div className="underline-class"></div>

- • ट्राई-एक्सेप्ट ब्लॉक का उपयोग करें
- • अंत में साफ-सफाई में
- • त्रुटियों को उचित रूप से लॉग करें

<h3 className="third-heading">प्रदर्शन</h3>
<div className="underline-class"></div>

- • कॉलबैक में ब्लॉकिंग से बचें
- • कॉलबैक समूह का उपयोग करें
- • मेमोरी की निगरानी करें

<h3 className="third-heading">डिबगिंग</h3>
<div className="underline-class"></div>

- • `self.get_logger().info()` का उपयोग करें
- • `ros2 node list`, `ros2 topic list` के साथ जांच करें
- • `rqt_console` का उपयोग करें

<div className="border-line"></div>

<h2 className="second-heading">समस्या निवारण</h2>
<div className="underline-class"></div>

<h3 className="third-heading">इम्पोर्ट त्रुटियां</h3>
<div className="underline-class"></div>

- • उचित रूप से बिल्ड और सोर्स करें
- • `__init__.py` फाइल की जांच करें
- • पायथनपाथ की पुष्टि करें

<h3 className="third-heading">अनुमति समस्याएं</h3>
<div className="underline-class"></div>

- • रूट के रूप में न चलाएं
- • फाइल अनुमतियों की जांच करें
- • स्थापना की पुष्टि करें

<h3 className="third-heading">संचार समस्याएं</h3>
<div className="underline-class"></div>

- • डोमेन की जांच करें
- • टॉपिक/सेवा नामों की पुष्टि करें
- • उचित क्यूओएस सुनिश्चित करें

<h2 className="second-heading">सारांश</h2>
<div className="underline-class"></div>

आरओएस 2 में पायथन पैकेज प्रकाशक, सदस्य, सेवाएं और पैरामीटर के साथ नोड्स बनाने के लिए आरसीएलपीवाई का उपयोग करते हैं। उचित संरचना, परीक्षण और सर्वोत्तम प्रथाओं का उपयोग करें।