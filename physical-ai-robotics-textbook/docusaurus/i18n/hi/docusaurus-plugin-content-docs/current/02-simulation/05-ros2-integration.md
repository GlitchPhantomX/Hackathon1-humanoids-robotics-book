---
sidebar_position: 6
title: 'आरओएस 2 एकीकरण: सिमुलेशन से वास्तविक रोबोटिक्स'
description: 'व्यापक रोबोटिक्स विकास के लिए सिमुलेशन वातावरण को आरओएस 2 के साथ एकीकृत करना'
---

import ReadingTime from '@site/src/components/ReadingTime';
import ViewToggle from '@site/src/components/ViewToggle';

<ReadingTime minutes={18} />
<!-- <ViewToggle /> -->

<h1 className="main-heading">आरओएस 2 एकीकरण: सिमुलेशन को वास्तविक रोबोटिक्स से जोड़ना</h1>

<div className="underline-class"></div>

<h2 className="second-heading">सीखने के उद्देश्य</h2>

<div className="border-line"></div>

इस अध्याय के अंत तक, आप यह करने में सक्षम होंगे:
- • द्विदिशात्मक संचार के लिए सिमुलेशन वातावरण को आरओएस 2 के साथ एकीकृत करना
- • सिमुलेशन और आरओएस 2 के बीच सेंसर और एक्चुएटर ब्रिज कार्यान्वित करना
- • सिमुलेशन नियंत्रण और निगरानी के लिए आरओएस 2 उपकरणों का उपयोग करना
- • वास्तविक-दुनिया के संचालन को दर्शाते हुए सिमुलेशन कार्यप्रवाह डिज़ाइन करना
- • मानवरूपी रोबोट के लिए सिमुलेशन-से-वास्तविकता स्थानांतरण को मान्य करना

<div className="border-line"></div>

<h2 className="second-heading">अभ्यास</h2>

<div className="border-line"></div>

<details>
<summary>अभ्यास 2.5.1: मूल आरओएस 2-सिमुलेशन ब्रिज (⭐, ~25 मिनट)</summary>

<h3 className="third-heading">अभ्यास 2.5.1: मूल आरओएस 2-सिमुलेशन ब्रिज सेटअप</h3>

<div className="border-line"></div>

**कठिनाई**: ⭐ | **समय**: 25 मिनट

<h4 className="fourth-heading">कार्य</h4>

<div className="border-line"></div>

आरओएस 2 एकीकरण प्लगइन्स के साथ रोबोट मॉडल बनाएं

<h4 className="fourth-heading">परीक्षण कमांड</h4>

<div className="border-line"></div>

```bash
# बिल्ड और लॉन्च
gz sdf -k your_robot.model.sdf
gz sim -r your_world.sdf

# विषयों की जांच करें
ros2 topic list | grep my_robot
ros2 topic echo /my_robot/camera/image_raw

# कमांड भेजें
ros2 topic pub /my_robot/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}}'
```

<h4 className="fourth-heading">सफलता मानदंड</h4>

<div className="border-line"></div>

- [ ] आरओएस 2 प्लगइन्स के साथ रोबोट लोड होता है
- [ ] कैमरा डेटा प्रकाशित होता है
- [ ] आईएमयू डेटा प्रकाशित होता है
- [ ] जॉइंट स्टेट्स प्रकाशित होते हैं
- [ ] रोबोट कमांड्स का उत्तर देता है

</details>

<details>
<summary>अभ्यास 2.5.2: उन्नत सेंसर प्रसंस्करण (⭐⭐, ~45 मिनट)</summary>

<h3 className="third-heading">अभ्यास 2.5.2: उन्नत सेंसर एकीकरण</h3>

<div className="border-line"></div>

**कठिनाई**: ⭐⭐ | **समय**: 45 मिनट

<h4 className="fourth-heading">कार्य</h4>

<div className="border-line"></div>

पूर्ण सेंसर प्रसंस्करण पाइपलाइन बनाएं

<h4 className="fourth-heading">परीक्षण कमांड</h4>

<div className="border-line"></div>

```bash
# पाइपलाइन लॉन्च करें
ros2 launch my_robot_perception sensor_processing.launch.py

# प्रसंस्करण का परीक्षण करें
ros2 topic echo /my_robot/camera/processed_image
ros2 topic echo /my_robot/obstacles

# दृश्यकरण
ros2 run rviz2 rviz2 -d config/sensor_fusion.rviz
```

<h4 className="fourth-heading">सफलता मानदंड</h4>

<div className="border-line"></div>

- [ ] ऑब्जेक्ट डिटेक्शन काम करता है
- [ ] बाधा डिटेक्शन ऑब्जेक्ट्स की पहचान करता है
- [ ] आईएमयू डेटा एकीकृत है
- [ ] सेंसर फ्यूजन सुसंगत है
- [ ] आरवीआईजेड2 में डेटा दृश्यकृत है

</details>

<details>
<summary>अभ्यास 2.5.3: मल्टी-रोबोट समन्वयन (⭐⭐⭐, ~60 मिनट)</summary>

<h3 className="third-heading">अभ्यास 2.5.3: मल्टी-रोबोट समन्वयन</h3>

<div className="border-line"></div>

**कठिनाई**: ⭐⭐⭐ | **समय**: 60 मिनट

<h4 className="fourth-heading">कार्य</h4>

<div className="border-line"></div>

समन्वयन के साथ मल्टी-रोबोट सिमुलेशन कार्यान्वित करें

<h4 className="fourth-heading">परीक्षण कमांड</h4>

<div className="border-line"></div>

```bash
# मल्टी-रोबोट लॉन्च करें
ros2 launch my_robot_multi simulation.launch.py

# रोबोट्स की जांच करें
ros2 topic list | grep robot_
ros2 topic echo /multi_robot/coordinator/status

# कमांड भेजें
ros2 topic pub /robot_0/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}}'
```

<h4 className="fourth-heading">सफलता मानदंड</h4>

<div className="border-line"></div>

- [ ] कई रोबोट संचालित होते हैं
- [ ] समन्वयन टकरावों को रोकता है
- [ ] कार्यों का उचित रूप से वितरण
- [ ] संचार कार्यात्मक है
- [ ] प्रदर्शन स्थिर है

</details>

<div className="border-line"></div>

<h2 className="second-heading">समस्या निवारण</h2>

<div className="border-line"></div>

<details>
<summary>समस्या निवारण: आरओएस 2 एकीकरण समस्याएँ</summary>

<h3 className="third-heading">समस्या निवारण: आरओएस 2 एकीकरण समस्याएँ</h3>

<div className="border-line"></div>

<h4 className="fourth-heading">समस्या: नोड्स सिमुलेशन से कनेक्ट नहीं हो सकते</h4>

<div className="border-line"></div>

**लक्षण**:
- • कोई सेंसर डेटा प्रकाशित नहीं होता
- • रोबोट प्रतिक्रिया नहीं देता
- • विषय दिखाई नहीं देते

<div className="border-line"></div>

**समाधान**:
```xml
<!-- सही प्लगइन कॉन्फ़िग -->
<plugin name="camera_controller" filename="libgazebo_ros_camera.so">
  <ros>
    <namespace>/my_robot</namespace>
    <remapping>~/image_raw:=/camera/image_raw</remapping>
  </ros>
</plugin>
```

```bash
# पुष्टि करें
ros2 topic list
sudo apt install ros-humble-gazebo-ros-pkgs
```

<div className="border-line"></div>

<h4 className="fourth-heading">समस्या: सेंसर डेटा गलत है</h4>

<div className="border-line"></div>

**लक्षण**:
- • विषयों में कोई डेटा नहीं
- • मान परास से बाहर
- • उच्च विलंबता

<div className="border-line"></div>

**समाधान**:
```xml
<!-- उचित सेंसर कॉन्फ़िग -->
<sensor name="camera" type="camera">
  <update_rate>30</update_rate>
  <always_on>true</always_on>
</sensor>
```

```bash
# फ्रेम्स की जांच करें
ros2 run tf2_tools view_frames
ros2 topic echo /my_robot/camera/image_raw
```

<div className="border-line"></div>

<h4 className="fourth-heading">समस्या: प्रदर्शन समस्याएँ</h4>

<div className="border-line"></div>

**लक्षण**:
- • कम रीयल-टाइम फैक्टर
- • उच्च सीपीयू उपयोग
- • कतार अतिप्रवाह चेतावनियाँ

<div className="border-line"></div>

**समाधान**:
```python
# कुशल क्यूओएस
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

sensor_qos = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.BEST_EFFORT
)
```

```xml
<!-- अपडेट दर कम करें -->
<sensor name="camera" type="camera">
  <update_rate>15</update_rate>
</sensor>
```

<div className="border-line"></div>

<h4 className="fourth-heading">समस्या: समय सिंक्रनाइज़ेशन समस्याएँ</h4>

<div className="border-line"></div>

**लक्षण**:
- • असुसंगत टाइमस्टैम्प
- • विलंबित टीएफ परिवर्तन

<div className="border-line"></div>

**समाधान**:
```python
# सभी नोड्स में
self.declare_parameter('use_sim_time', True)

# लॉन्च फ़ाइल्स में
Node(
    parameters=[{'use_sim_time': True}]
)
```

</details>

<div className="border-line"></div>

<h2 className="second-heading">आरओएस 2-सिमुलेशन एकीकरण</h2>

<div className="border-line"></div>

<h3 className="third-heading">एकीकरण पाइपलाइन</h3>

<div className="border-line"></div>

आरओएस 2 सिमुलेशन एकीकरण एक बंद लूप बनाता है:
- • सिमुलेशन वास्तविक सेंसर डेटा प्रदान करता है
- • आरओएस 2 नोड्स वास्तविक एल्गोरिदम के साथ प्रसंस्करण करते हैं
- • नियंत्रण कमांड सिमुलेटेड रोबोट को भेजे जाते हैं
- • सिमुलेशन कमांड्स का उत्तर देता है

लाभ:
- • हार्डवेयर के बिना एल्गोरिदम विकास
- • सुरक्षित वातावरण परीक्षण
- • नियंत्रण प्रणाली मान्यता
- • समानांतर हार्डवेयर/सॉफ्टवेयर विकास

<div className="border-line"></div>

<h3 className="third-heading">गेज़बो-आरओएस 2 ब्रिज</h3>

<div className="border-line"></div>

```xml
<!-- आरओएस 2 प्लगइन्स के साथ रोबोट -->
<model name="ros2_robot">
  <sensor name="camera" type="camera">
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/my_robot</namespace>
        <remapping>~/image_raw:=/camera/image_raw</remapping>
      </ros>
    </plugin>
  </sensor>

  <sensor name="imu" type="imu">
    <plugin name="imu_controller" filename="libgazebo_ros_imu.so">
      <ros>
        <namespace>/my_robot</namespace>
      </ros>
    </plugin>
  </sensor>

  <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
    <ros>
      <namespace>/my_robot</namespace>
    </ros>
    <command_topic>cmd_vel</command_topic>
    <odometry_topic>odom</odometry_topic>
  </plugin>
</model>
```

<div className="border-line"></div>

<h2 className="second-heading">सेंसर एकीकरण</h2>

<div className="border-line"></div>

<h3 className="third-heading">कैमरा एकीकरण</h3>

<div className="border-line"></div>

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraProcessor(Node):
    def __init__(self):
        super().__init__('camera_processor')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image, '/my_robot/camera/image_raw', self.callback, 10)
        self.pub = self.create_publisher(
            Image, '/my_robot/camera/processed', 10)

    def callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        edges = cv2.Canny(cv_image, 50, 150)
        self.pub.publish(self.bridge.cv2_to_imgmsg(edges, 'mono8'))
```

<div className="border-line"></div>

<h3 className="third-heading">लाइडार एकीकरण</h3>

<div className="border-line"></div>

```python
from sensor_msgs.msg import LaserScan
import numpy as np

class LIDARProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor')
        self.sub = self.create_subscription(
            LaserScan, '/my_robot/scan', self.callback, 10)

    def callback(self, msg):
        ranges = np.array(msg.ranges)
        valid = ranges[(ranges >= msg.range_min) &
                      (ranges <= msg.range_max)]
        min_dist = np.min(valid) if len(valid) > 0 else float('inf')
        self.get_logger().info(f'न्यूनतम दूरी: {min_dist:.2f}मी')
```

<div className="border-line"></div>

<h3 className="third-heading">आईएमयू एकीकरण</h3>

<div className="border-line"></div>

```python
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import math

class IMUProcessor(Node):
    def __init__(self):
        super().__init__('imu_processor')
        self.sub = self.create_subscription(
            Imu, '/my_robot/imu/data', self.callback, 10)
        self.pub = self.create_publisher(Vector3, '/orientation', 10)

    def callback(self, msg):
        q = msg.orientation
        roll, pitch, yaw = self.quat_to_euler(q.x, q.y, q.z, q.w)

        orientation = Vector3()
        orientation.x = roll
        orientation.y = pitch
        orientation.z = yaw
        self.pub.publish(orientation)
```

<div className="border-line"></div>

<h2 className="second-heading">एक्चुएटर एकीकरण</h2>

<div className="border-line"></div>

<h3 className="third-heading">जॉइंट नियंत्रण</h3>

<div className="border-line"></div>

```python
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math

class JointController(Node):
    def __init__(self):
        super().__init__('joint_controller')
        self.pub = self.create_publisher(
            JointTrajectory, '/joint_trajectory', 10)
        self.timer = self.create_timer(0.1, self.control_loop)
        self.time = 0.0

    def control_loop(self):
        traj = JointTrajectory()
        traj.joint_names = ['hip', 'knee']

        point = JointTrajectoryPoint()
        point.positions = [0.1 * math.sin(self.time),
                          0.2 * math.sin(self.time * 2)]
        traj.points = [point]

        self.pub.publish(traj)
        self.time += 0.1
```

<div className="border-line"></div>

<h3 className="third-heading">नेविगेशन एकीकरण</h3>

<div className="border-line"></div>

```python
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class NavigationController(Node):
    def __init__(self):
        super().__init__('nav_controller')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.target = (5.0, 5.0)

    def odom_callback(self, msg):
        dx = self.target[0] - msg.pose.pose.position.x
        dy = self.target[1] - msg.pose.pose.position.y
        distance = math.sqrt(dx*dx + dy*dy)

        cmd = Twist()
        cmd.linear.x = min(1.0, distance * 0.5)
        self.cmd_pub.publish(cmd)
```

<div className="border-line"></div>

<h2 className="second-heading">लॉन्च फ़ाइल्स</h2>

<div className="border-line"></div>

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='my_robot_perception',
            executable='camera_processor',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='my_robot_control',
            executable='navigation_controller',
            parameters=[{'use_sim_time': True}]
        )
    ])
```

<div className="border-line"></div>

<h2 className="second-heading">सर्वोत्तम प्रथाएँ</h2>

<div className="border-line"></div>

<h3 className="third-heading">प्रदर्शन अनुकूलन</h3>

<div className="border-line"></div>

- • **विषय दर प्रबंधन**: अपडेट दर नियंत्रित करें
- • **डेटा फ़िल्टरिंग**: केवल आवश्यक डेटा प्रकाशित करें
- • **क्यूओएस कॉन्फ़िगरेशन**: उपयुक्त क्यूओएस सेटिंग्स का उपयोग करें
- • **संसाधन आवंटन**: सीपीयू/मेमोरी उपयोग की निगरानी करें

<div className="border-line"></div>

<h3 className="third-heading">डेटा सुसंगतता</h3>

<div className="border-line"></div>

- • **समय सिंक्रनाइज़ेशन**: सिमुलेशन समय का उपयोग करें
- • **फ्रेम सुसंगतता**: टीएफ ट्री बनाए रखें
- • **डेटा मान्यता**: सेंसर डेटा की पुष्टि करें
- • **त्रुटि नियंत्रण**: मजबूत त्रुटि नियंत्रण

<div className="border-line"></div>

<h2 className="second-heading">सामान्य समस्याएँ</h2>

<div className="border-line"></div>

**कनेक्शन समस्याएँ**
- • गेज़बो-आरओएस प्लगइन्स लोड होने की पुष्टि करें
- • नामस्थान कॉन्फ़िगरेशन की जाँच करें
- • डीडीएस डोमेन सेटिंग्स की पुष्टि करें

**सेंसर डेटा प्रकाशित नहीं हो रहा**
- • प्लगइन कॉन्फ़िगरेशन की जाँच करें
- • विषय नामों की पुष्टि करें
- • सेंसर मॉडल परिभाषित होने की पुष्टि करें

**प्रदर्शन समस्याएँ**
- • सेंसर अपडेट दर कम करें
- • सक्रिय सेंसर सीमित रखें
- • क्यूओएस सेटिंग्स अनुकूलित करें

**उच्च सीपीयू उपयोग**
- • नोड निष्पादन की निगरानी करें
- • संदेश प्रकाशन कम करें
- • मल्टी-थ्रेडेड एक्सीक्यूटर्स का उपयोग करें

<div className="border-line"></div>

<h2 className="second-heading">सारांश</h2>

<div className="border-line"></div>

सिमुलेशन के साथ आरओएस 2 एकीकरण मानवरूपी रोबोटिक्स के लिए शक्तिशाली विकास मंच बनाता है। सेंसर, एक्चुएटर और नियंत्रण प्रणालियों को ठीक से जोड़ना सिमुलेशन और वास्तविकता के बीच व्यापक परीक्षण वातावरण स्थापित करता है। सफलता के लिए मजबूत संचार पैटर्न, प्रदर्शन अनुकूलन और आभासी और वास्तविक दुनिया के व्यवहार के बीच सुसंगतता बनाए रखने की आवश्यकता होती है।