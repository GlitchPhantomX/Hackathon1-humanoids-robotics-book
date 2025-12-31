---
sidebar_position: 6
title: 'ROS 2 انضمام: سیمولیشن سے حقیقی روبوٹکس'
description: 'جامع روبوٹکس ڈویلپمنٹ کے لیے سیمولیشن ماحول کو ROS 2 کے ساتھ انضمام کرنا'
---

import ReadingTime from '@site/src/components/ReadingTime';
import ViewToggle from '@site/src/components/ViewToggle';


<ReadingTime minutes={18} />
<!-- <ViewToggle /> -->

<h1 className="main-heading">ROS 2 انضمام: سیمولیشن کو حقیقی روبوٹکس سے جوڑنا</h1>

<div className="underline-class"></div>

<h2 className="second-heading">سیکھنے کے اہداف</h2>

<div className="border-line"></div>

اس باب کے اختتام تک، آپ کے اہل ہوں گے:
- • دو طرفہ مواصلات کے لیے سیمولیشن ماحول کو ROS 2 کے ساتھ انضمام کرنا
- • سیمولیشن اور ROS 2 کے درمیان سینسر اور ایکچوایٹر برج نافذ کرنا
- • سیمولیشن کنٹرول اور مانیٹرنگ کے لیے ROS 2 ٹولز استعمال کرنا
- • حقیقی دنیا کے کاموں کو نقل کرنے والے سیمولیشن ورک فلوز ڈیزائن کرنا
- • ہیومنوائڈ روبوٹس کے لیے سیمولیشن-سے-حقیقت منتقلی کی توثیق کرنا

<div className="border-line"></div>

<h2 className="second-heading">مشقیں</h2>

<div className="border-line"></div>

<details>
<summary>مشق 2.5.1: بنیادی ROS 2-سیمولیشن برج (⭐, ~25 منٹ)</summary>

<h3 className="third-heading">مشق 2.5.1: بنیادی ROS 2-سیمولیشن برج سیٹ اپ</h3>

<div className="border-line"></div>

**پیچیدگی**: ⭐ | **وقت**: 25 منٹ

<h4 className="fourth-heading">کام</h4>

<div className="border-line"></div>

ROS 2 انضمام پلگ انز کے ساتھ روبوٹ ماڈل بنائیں

<h4 className="fourth-heading">ٹیسٹ کمانڈز</h4>

<div className="border-line"></div>

```bash
# تعمیر اور لانچ
gz sdf -k your_robot.model.sdf
gz sim -r your_world.sdf

# ٹوپکس چیک کریں
ros2 topic list | grep my_robot
ros2 topic echo /my_robot/camera/image_raw

# کمانڈز بھیجیں
ros2 topic pub /my_robot/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}}'
```

<h4 className="fourth-heading">کامیابی کے معیار</h4>

<div className="border-line"></div>

- [ ] ROS 2 پلگ انز کے ساتھ روبوٹ لوڈ ہوتا ہے
- [ ] کیمرہ ڈیٹا پبلش ہوتا ہے
- [ ] IMU ڈیٹا پبلش ہوتا ہے
- [ ] جوائنٹ اسٹیٹس پبلش ہوتے ہیں
- [ ] روبوٹ کمانڈز کا جواب دیتا ہے

</details>

<details>
<summary>مشق 2.5.2: اعلیٰ سینسر پروسیسنگ (⭐⭐, ~45 منٹ)</summary>

<h3 className="third-heading">مشق 2.5.2: اعلیٰ سینسر انضمام</h3>

<div className="border-line"></div>

**پیچیدگی**: ⭐⭐ | **وقت**: 45 منٹ

<h4 className="fourth-heading">کام</h4>

<div className="border-line"></div>

مکمل سینسر پروسیسنگ پائپ لائن بنائیں

<h4 className="fourth-heading">ٹیسٹ کمانڈز</h4>

<div className="border-line"></div>

```bash
# پائپ لائن لانچ کریں
ros2 launch my_robot_perception sensor_processing.launch.py

# پروسیسنگ ٹیسٹ کریں
ros2 topic echo /my_robot/camera/processed_image
ros2 topic echo /my_robot/obstacles

# ویژولائز کریں
ros2 run rviz2 rviz2 -d config/sensor_fusion.rviz
```

<h4 className="fourth-heading">کامیابی کے معیار</h4>

<div className="border-line"></div>

- [ ] آبجیکٹ ڈیٹیکشن کام کرتا ہے
- [ ] رکاوٹ ڈیٹیکشن اشیاء کی شناخت کرتا ہے
- [ ] IMU ڈیٹا انضمام ہوتا ہے
- [ ] سینسر فیوژن مطابقت رکھتی ہے
- [ ] ڈیٹا RViz2 میں ویژولائزڈ ہوتا ہے

</details>

<details>
<summary>مشق 2.5.3: متعدد روبوٹ کوآرڈینیشن (⭐⭐⭐, ~60 منٹ)</summary>

<h3 className="third-heading">مشق 2.5.3: متعدد روبوٹ کوآرڈینیشن</h3>

<div className="border-line"></div>

**پیچیدگی**: ⭐⭐⭐ | **وقت**: 60 منٹ

<h4 className="fourth-heading">کام</h4>

<div className="border-line"></div>

کوآرڈینیشن کے ساتھ متعدد روبوٹ سیمولیشن نافذ کریں

<h4 className="fourth-heading">ٹیسٹ کمانڈز</h4>

<div className="border-line"></div>

```bash
# متعدد روبوٹ لانچ کریں
ros2 launch my_robot_multi simulation.launch.py

# روبوٹس چیک کریں
ros2 topic list | grep robot_
ros2 topic echo /multi_robot/coordinator/status

# کمانڈز بھیجیں
ros2 topic pub /robot_0/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}}'
```

<h4 className="fourth-heading">کامیابی کے معیار</h4>

<div className="border-line"></div>

- [ ] متعدد روبوٹس کام کرتے ہیں
- [ ] کوآرڈینیشن تصادم سے بچاتی ہے
- [ ] کام مناسب طریقے سے تقسیم ہوتے ہیں
- [ ] مواصلات کار کام کرتی ہے
- [ ] کارکردگی مستحکم ہے

</details>

<div className="border-line"></div>

<h2 className="second-heading">ٹربل شوٹنگ</h2>

<div className="border-line"></div>

<details>
<summary>ٹربل شوٹنگ: ROS 2 انضمام کے مسائل</summary>

<h3 className="third-heading">ٹربل شوٹنگ: ROS 2 انضمام کے مسائل</h3>

<div className="border-line"></div>

<h4 className="fourth-heading">مسئلہ: نوڈس سیمولیشن سے مربوط نہیں ہو سکتے</h4>

<div className="border-line"></div>

**علامات**:
- • کوئی سینسر ڈیٹا پبلش نہیں ہوتا
- • روبوٹ جواب نہیں دیتا
- • ٹوپکس ظاہر نہیں ہوتے

<div className="border-line"></div>

**حل**:
```xml
<!-- درست پلگ ان کنفیگ -->
<plugin name="camera_controller" filename="libgazebo_ros_camera.so">
  <ros>
    <namespace>/my_robot</namespace>
    <remapping>~/image_raw:=/camera/image_raw</remapping>
  </ros>
</plugin>
```

```bash
# تصدیق کریں
ros2 topic list
sudo apt install ros-humble-gazebo-ros-pkgs
```

<div className="border-line"></div>

<h4 className="fourth-heading">مسئلہ: سینسر ڈیٹا غلط ہے</h4>

<div className="border-line"></div>

**علامات**:
- • ٹوپکس میں کوئی ڈیٹا نہیں
- • ویلیوز حد سے باہر ہیں
- • زیادہ تاخیر

<div className="border-line"></div>

**حل**:
```xml
<!-- مناسب سینسر کنفیگ -->
<sensor name="camera" type="camera">
  <update_rate>30</update_rate>
  <always_on>true</always_on>
</sensor>
```

```bash
# فریم چیک کریں
ros2 run tf2_tools view_frames
ros2 topic echo /my_robot/camera/image_raw
```

<div className="border-line"></div>

<h4 className="fourth-heading">مسئلہ: کارکردگی کے مسائل</h4>

<div className="border-line"></div>

**علامات**:
- • کم ریل ٹائم فیکٹر
- • زیادہ CPU استعمال
- • قطار میں اوور فلو وارننگز

<div className="border-line"></div>

**حل**:
```python
# کارآمد QoS
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

sensor_qos = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.BEST_EFFORT
)
```

```xml
<!-- اپ ڈیٹ شرح کم کریں -->
<sensor name="camera" type="camera">
  <update_rate>15</update_rate>
</sensor>
```

<div className="border-line"></div>

<h4 className="fourth-heading">مسئلہ: ٹائم سنکرونائزیشن کے مسائل</h4>

<div className="border-line"></div>

**علامات**:
- • غیر مطابق ٹائم اسٹیمپس
- • تاخیر شدہ TF ٹرانسفارمیشنز

<div className="border-line"></div>

**حل**:
```python
# تمام نوڈس میں
self.declare_parameter('use_sim_time', True)

# لانچ فائلز میں
Node(
    parameters=[{'use_sim_time': True}]
)
```

</details>

<div className="border-line"></div>

<h2 className="second-heading">ROS 2-سیمولیشن انضمام</h2>

<div className="border-line"></div>

<h3 className="third-heading">انضمام پائپ لائن</h3>

<div className="border-line"></div>

ROS 2 سیمولیشن انضمام ایک بند حلقہ بنتا ہے:
- • سیمولیشن حقیقی طرز کا سینسر ڈیٹا فراہم کرتا ہے
- • ROS 2 نوڈس حقیقی الگورتھم کے ساتھ پروسیس کرتے ہیں
- • کنٹرول کمانڈز سیمولیٹڈ روبوٹس کو بھیجی جاتی ہیں
- • سیمولیشن کمانڈز کا جواب دیتا ہے

فوائد:
- • ہارڈ ویئر کے بغیر الگورتھم ڈویلپمنٹ
- • محفوظ ماحول ٹیسٹنگ
- • کنٹرول سسٹم کی توثیق
- • متوازی ہارڈ ویئر/سافٹ ویئر ڈویلپمنٹ

<div className="border-line"></div>

<h3 className="third-heading">Gazebo-ROS 2 برج</h3>

<div className="border-line"></div>

```xml
<!-- ROS 2 پلگ انز کے ساتھ روبوٹ -->
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

<h2 className="second-heading">سینسر انضمام</h2>

<div className="border-line"></div>

<h3 className="third-heading">کیمرہ انضمام</h3>

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

<h3 className="third-heading">LIDAR انضمام</h3>

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
        self.get_logger().info(f'Min distance: {min_dist:.2f}m')
```

<div className="border-line"></div>

<h3 className="third-heading">IMU انضمام</h3>

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

<h2 className="second-heading">ایکچوایٹر انضمام</h2>

<div className="border-line"></div>

<h3 className="third-heading">جوائنٹ کنٹرول</h3>

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

<h3 className="third-heading">نیویگیشن انضمام</h3>

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

<h2 className="second-heading">لانچ فائلز</h2>

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

<h2 className="second-heading">بہترین مشقیں</h2>

<div className="border-line"></div>

<h3 className="third-heading">کارکردگی کی بہتری</h3>

<div className="border-line"></div>

- • **ٹوپک شرح کا انتظام**: اپ ڈیٹ شرح کنٹرول کریں
- • **ڈیٹا فلٹرنگ**: صرف ضروری ڈیٹا پبلش کریں
- • **QoS کنفیگریشن**: مناسب QoS ترتیبات استعمال کریں
- • **ریسورس الاؤکیشن**: CPU/میموری استعمال کو مانیٹر کریں

<div className="border-line"></div>

<h3 className="third-heading">ڈیٹا مطابقت</h3>

<div className="border-line"></div>

- • **ٹائم سنکرونائزیشن**: سیمولیشن ٹائم استعمال کریں
- • **فریم مطابقت**: TF ٹری برقرار رکھیں
- • **ڈیٹا کی توثیق**: سینسر ڈیٹا کی تصدیق کریں
- • **غلطی کا انتظام**: مضبوط غلطی کا انتظام

<div className="border-line"></div>

<h2 className="second-heading">عام مسائل</h2>

<div className="border-line"></div>

**کنکشن کے مسائل**
- • Gazebo-ROS پلگ انز لوڈ ہونے کی تصدیق کریں
- • نیم اسپیس کنفیگریشن چیک کریں
- • DDS ڈومین ترتیبات کی تصدیق کریں

**سینسر ڈیٹا پبلش نہیں ہو رہا**
- • پلگ ان کنفیگریشن چیک کریں
- • ٹوپک ناموں کی تصدیق کریں
- • سینسر ماڈلز کی وضاحت کی تصدیق کریں

**کارکردگی کے مسائل**
- • سینسر اپ ڈیٹ شرح کم کریں
- • ایکٹو سینسرز کو محدود کریں
- • QoS ترتیبات کو بہتر بنائیں

**زیادہ CPU استعمال**
- • نوڈ ایگزیکیوشن مانیٹر کریں
- • میسج پبلیکیشن کم کریں
- • ملٹی-تھریڈڈ ایگزیکیوٹرز استعمال کریں

<div className="border-line"></div>

<h2 className="second-heading">خلاصہ</h2>

<div className="border-line"></div>

سیمولیشن کے ساتھ ROS 2 کا انضمام ہیومنوائڈ روبوٹکس کے لیے طاقتور ڈویلپمنٹ پلیٹ فارم بنتا ہے۔ سینسرز، ایکچوایٹرز، اور کنٹرول سسٹم کو صحیح طریقے سے جوڑنا جامع ٹیسٹنگ ماحول قائم کرتا ہے جو سیمولیشن اور حقیقت کے درمیان پل بناتا ہے۔ کامیابی کے لیے مضبوط مواصلاتی پیٹرنز، کارکردگی کی بہتری، اور ورچوئل اور حقیقی دنیا کے رویوں کے درمیان مطابقت برقرار رکھنے کی ضرورت ہوتی ہے۔