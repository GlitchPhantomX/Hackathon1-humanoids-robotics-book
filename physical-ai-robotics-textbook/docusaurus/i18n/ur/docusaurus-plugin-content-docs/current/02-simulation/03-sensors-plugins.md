---
sidebar_position: 4
title: 'سینسرز اور پلگ انز: حقیقی طرز کے ادراک کے ساتھ سیمولیشن کو بہتر بنانا'
description: 'روبوٹکس سیمولیشن میں حقیقی طرز کے ادراک کے لیے سینسرز اور پلگ انز کو نافذ کرنا'
---

import ReadingTime from '@site/src/components/ReadingTime';


<ReadingTime minutes={5} />

<h1 className="main-heading">سینسرز اور پلگ انز: حقیقی طرز کے ادراک کے ساتھ سیمولیشن کو بہتر بنانا</h1>
<div className="underline-class"></div>

سیمولیشن میں ہیومنوائڈ روبوٹس کے لیے معنی خیز ادراک کا ڈیٹا بنانے کے لیے حقیقی طرز کے سینسرز اور پلگ انز نافذ کریں۔

<div className="border-line"></div>

<h2 className="second-heading">سیکھنے کے اہداف</h2>
<div className="underline-class"></div>

- • مختلف سینسر اقسام نافذ کرنا
- • سینسر پیرامیٹرز تشکیل دینا
- • اپنی پلگ انز بنانا
- • ROS 2 کے ساتھ سینسرز کو انضمام کرنا
- • سینسر ڈیٹا کی توثیق کرنا

<div className="border-line"></div>

<h2 className="second-heading">مشقیں</h2>
<div className="underline-class"></div>

<details>
<summary>مشق 2.3.1: بنیادی سینسر انضمام (⭐, ~30 منٹ)</summary>

<h3 className="third-heading">مشق 2.3.1: بنیادی سینسر انضمام</h3>
<div className="underline-class"></div>

**پیچیدگی**: ⭐ | **وقت**: 30 منٹ | **ضروریات**: Gazebo، ROS 2، XML

<h4 className="fourth-heading">شروع کرنے والا کوڈ</h4>
<div className="underline-class"></div>

- • کیمرہ کنفیگریشن (ریزولوشن، FOV)
- • ROS 2 پلگ ان
- • نوائز ماڈلنگ

<h4 className="fourth-heading">کامیابی کے معیار</h4>
<div className="underline-class"></div>

- [ ] کیمرہ ROS 2 کو پبلش کرتا ہے
- [ ] ٹوپکس کے ذریعے ڈیٹا قابل رسائی ہے
- [ ] نوائز تشکیل دیا گیا ہے

<h4 className="fourth-heading">ٹیسٹ کمانڈز</h4>
<div className="underline-class"></div>
```bash
ros2 launch your_robot_gazebo your_robot.launch.py
ros2 topic list | grep camera
ros2 run image_view image_view _image:=/your_robot/camera/image_raw
```

<h4 className="fourth-heading">اشارے</h4>
<div className="underline-class"></div>

- • TF کے لیے مناسب فریم نام استعمال کریں
- • پلگ ان نیم اسپیس کی تصدیق کریں

</details>

<details>
<summary>مشق 2.3.2: اعلیٰ سینسر کنفیگریشن (⭐⭐, ~45 منٹ)</summary>

<h3 className="third-heading">مشق 2.3.2: اعلیٰ سینسر کنفیگریشن</h3>
<div className="underline-class"></div>

**پیچیدگی**: ⭐⭐ | **وقت**: 45 منٹ

<h4 className="fourth-heading">شروع کرنے والا کوڈ</h4>
<div className="underline-class"></div>

- • کیمرہ، LIDAR، IMU سینسرز
- • نوائز ماڈلنگ
- • سینسر فیوژن

<h4 className="fourth-heading">ٹیسٹ کمانڈز</h4>
<div className="underline-class"></div>
```bash
ros2 topic list | grep -E "(camera|scan|imu)"
ros2 topic hz /your_robot/camera/image_raw
ros2 run rviz2 rviz2
```

<h4 className="fourth-heading">اشارے</h4>
<div className="underline-class"></div>

- • معیار کو کارکردگی کے ساتھ توازن دیں
- • مناسب اپ ڈیٹ شرح استعمال کریں

</details>

<details>
<summary>مشق 2.3.3: اپنی سینسر پلگ ان (⭐⭐⭐, ~60 منٹ)</summary>

<h3 className="third-heading">مشق 2.3.3: اپنی سینسر پلگ ان</h3>
<div className="underline-class"></div>

**پیچیدگی**: ⭐⭐⭐ | **وقت**: 60 منٹ | **ضروریات**: C++، Gazebo API

<h4 className="fourth-heading">شروع کرنے والا کوڈ</h4>
<div className="underline-class"></div>

- • Gazebo سینسرز کو بڑھانا
- • ROS 2 کے ساتھ انضمام
- • اپنی ڈیٹا پروسیسنگ

<h4 className="fourth-heading">ٹیسٹ کمانڈز</h4>
<div className="underline-class"></div>
```bash
cd ~/ros2_ws
colcon build --packages-select your_robot_gazebo
ros2 launch your_robot_gazebo custom_sensor_demo.launch.py
```

<h4 className="fourth-heading">اشارے</h4>
<div className="underline-class"></div>

- • Gazebo پلگ ان ہدایات کو فالو کریں
- • پہلے آئزولیشن میں پلگ ان ٹیسٹ کریں

</details>

<div className="border-line"></div>

<h2 className="second-heading">ٹربل شوٹنگ</h2>
<div className="underline-class"></div>

<details>
<summary>عام مسائل</summary>

<h3 className="third-heading">ٹربل شوٹنگ</h3>
<div className="underline-class"></div>

<h4 className="fourth-heading">نوائز/غیر حقیقی ڈیٹا</h4>
<div className="underline-class"></div>

**حل**:
```xml
<noise><type>gaussian</type><mean>0.0</mean><stddev>0.007</stddev></noise>
<physics><max_step_size>0.001</max_step_size></physics>
```

<h4 className="fourth-heading">سینسر ٹوپکس غائب ہیں</h4>
<div className="underline-class"></div>

**حل**:
```xml
<plugin name="camera_controller" filename="libgazebo_ros_camera.so">
  <ros><namespace>/robot_name</namespace></ros>
</plugin>
```
```bash
sudo apt install ros-humble-gazebo-ros-pkgs
```

<h4 className="fourth-heading">پلگ ان لوڈ ہونے میں ناکام ہو جاتا ہے</h4>
<div className="underline-class"></div>

**حل**:
```bash
find ~/ros2_ws/install -name "*plugin*.so"
colcon build --packages-select your_robot_gazebo
```
```cpp
GZ_REGISTER_SENSOR_PLUGIN(YourPluginClassName)
```

<h4 className="fourth-heading">کم کارکردگی</h4>
<div className="underline-class"></div>

**حل**:
```xml
<sensor><update_rate>15</update_rate></sensor>
<ray><scan><horizontal><samples>360</samples></horizontal></scan></ray>
```

<h4 className="fourth-heading">ڈیٹا تاخیر سے/غیر ہم وقت</h4>
<div className="underline-class"></div>

**حل**:
```python
qos_profile = rclpy.qos.QoSProfile(
    depth=1, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT
)
```

</details>

<div className="border-line"></div>

<h2 className="second-heading">سینسر اقسام</h2>
<div className="underline-class"></div>

<h3 className="third-heading">عام سینسرز</h3>
<div className="underline-class"></div>

<h4 className="fourth-heading">کیمرہ</h4>
<div className="underline-class"></div>
```xml
<sensor name="camera" type="camera">
  <camera>
    <horizontal_fov>1.047</horizontal_fov>
    <image><width>640</width><height>480</height></image>
    <noise><type>gaussian</type><stddev>0.007</stddev></noise>
  </camera>
  <plugin filename="libgazebo_ros_camera.so">
    <ros><namespace>/robot</namespace></ros>
  </plugin>
</sensor>
```

<h4 className="fourth-heading">LIDAR</h4>
<div className="underline-class"></div>
```xml
<sensor name="lidar" type="ray">
  <ray>
    <scan><horizontal>
      <samples>720</samples>
      <min_angle>-3.14</min_angle>
    </horizontal></scan>
    <range><min>0.1</min><max>10.0</max></range>
  </ray>
  <plugin filename="libgazebo_ros_ray_sensor.so"/>
</sensor>
```

<h4 className="fourth-heading">IMU</h4>
<div className="underline-class"></div>
```xml
<sensor name="imu" type="imu">
  <update_rate>100</update_rate>
  <imu>
    <angular_velocity><x><noise type="gaussian"><stddev>2e-4</stddev></noise></x></angular_velocity>
  </imu>
  <plugin filename="libgazebo_ros_imu.so"/>
</sensor>
```

<h3 className="third-heading">ROS 2 انضمام</h3>
<div className="underline-class"></div>
```python
import rclpy
from sensor_msgs.msg import Image, LaserScan, Imu

class SensorProcessor(Node):
    def __init__(self):
        super().__init__('sensor_processor')
        self.camera_sub = self.create_subscription(Image, '/camera/image_raw', self.camera_cb, 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_cb, 10)

    def camera_cb(self, msg):
        # تصویر کو پروسیس کریں
        pass
```

<div className="border-line"></div>

<h2 className="second-heading">اپنی پلگ انز</h2>
<div className="underline-class"></div>

<h3 className="third-heading">پلگ ان ترقی</h3>
<div className="underline-class"></div>
```cpp
#include <gazebo/sensors/sensors.hh>
#include <rclcpp/rclcpp.hpp>

class CustomSensorPlugin : public SensorPlugin {
  public: void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) {
    this->parent_sensor_ = std::dynamic_pointer_cast<sensors::RaySensor>(_sensor);
    this->node_ = std::make_shared<rclcpp::Node>("custom_sensor");
    this->publisher_ = this->node_->create_publisher<sensor_msgs::msg::PointCloud2>("/custom", 10);
  }
};
GZ_REGISTER_SENSOR_PLUGIN(CustomSensorPlugin)
```

<h3 className="third-heading">پلگ ان کنفیگریشن</h3>
<div className="underline-class"></div>
```xml
<sensor name="custom_lidar" type="ray">
  <plugin filename="libCustomSensorPlugin.so">
    <ros><namespace>/robot</namespace></ros>
  </plugin>
</sensor>
```

<div className="border-line"></div>

<h2 className="second-heading">اعلیٰ ماڈلنگ</h2>
<div className="underline-class"></div>

<h3 className="third-heading">نوائز ماڈلنگ</h3>
<div className="underline-class"></div>
```xml
<camera>
  <lens><type>stereographic</type></lens>
  <noise><type>gaussian</type><mean>0.0</mean><stddev>0.007</stddev></noise>
</camera>
```

<h3 className="third-heading">سینسر فیوژن</h3>
<div className="underline-class"></div>
```xml
<!-- ہیومنوائڈ پر متعدد سینسرز -->
<gazebo reference="head_link">
  <sensor name="head_camera" type="camera"/>
  <sensor name="head_imu" type="imu"/>
</gazebo>
<gazebo reference="torso_link">
  <sensor name="torso_lidar" type="ray"/>
</gazebo>
```

<div className="border-line"></div>

<h2 className="second-heading">بہترین مشقیں</h2>
<div className="underline-class"></div>

<h3 className="third-heading">کارکردگی کی بہتری</h3>
<div className="underline-class"></div>

- • اپ ڈیٹ شرح کم کریں
- • سینسر ریزولوشن کو محدود کریں
- • کارآمد نوائز ماڈلز استعمال کریں
- • سینسرز کو منتخب طور پر چالو کریں

<h3 className="third-heading">ریلزم بمقابلہ کارکردگی</h3>
<div className="underline-class"></div>
```python
class SensorPerformanceTester(Node):
    def __init__(self):
        self.sensor_stats = {'camera': {'count': 0, 'total_time': 0}}
        self.timer = self.create_timer(5.0, self.log_stats)

    def log_stats(self):
        freq = self.sensor_stats['camera']['count'] / 5.0
        self.get_logger().info(f'Camera freq: {freq:.2f}Hz')
```

<h3 className="third-heading">توثیق</h3>
<div className="underline-class"></div>

- • حقیقی ہارڈ ویئر کے ساتھ موازنہ کریں
- • گراؤنڈ ٹروتھ سینسرز استعمال کریں
- • احصائی توثیق
- • کراس توثیق

<h2 className="second-heading">خلاصہ</h2>
<div className="underline-class"></div>

معیاری نوائز اور اپنی پلگ انز کے ساتھ مناسب سینسر ماڈلز نافذ کریں تاکہ خصوصی افعال کے لیے۔ کارکردگی کے ساتھ سینسر کے ریلزم کو توازن دیں جبکہ درست ادراک کا ڈیٹا برقرار رکھیں۔