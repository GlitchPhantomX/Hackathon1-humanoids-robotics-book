---
sidebar_position: 4
title: 'सेंसर और प्लगइन्स: वास्तविक धारणा के साथ सिमुलेशन को बढ़ाना'
description: 'रोबोटिक्स सिमुलेशन में वास्तविक धारणा के लिए सेंसर और प्लगइन्स का कार्यान्वयन'
---

import ReadingTime from '@site/src/components/ReadingTime';

<ReadingTime minutes={5} />

<h1 className="main-heading">सेंसर और प्लगइन्स: वास्तविक धारणा के साथ सिमुलेशन को बढ़ाना</h1>
<div className="underline-class"></div>

सिमुलेशन में मानवरूपी रोबोट के लिए अर्थपूर्ण धारणा डेटा बनाने के लिए वास्तविक सेंसर और प्लगइन्स का कार्यान्वयन करें।

<div className="border-line"></div>

<h2 className="second-heading">सीखने के उद्देश्य</h2>
<div className="underline-class"></div>

- • विभिन्न सेंसर प्रकार का कार्यान्वयन करें
- • सेंसर पैरामीटर कॉन्फ़िगर करें
- • कस्टम प्लगइन्स बनाएं
- • आरओएस 2 के साथ सेंसर एकीकृत करें
- • सेंसर डेटा की वैधता सुनिश्चित करें

<div className="border-line"></div>

<h2 className="second-heading">अभ्यास</h2>
<div className="underline-class"></div>

<details>
<summary>अभ्यास 2.3.1: मूल सेंसर एकीकरण (⭐, ~30 मिनट)</summary>

<h3 className="third-heading">अभ्यास 2.3.1: मूल सेंसर एकीकरण</h3>
<div className="underline-class"></div>

**कठिनाई**: ⭐ | **समय**: 30 मिनट | **आवश्यकताएँ**: गेज़बो, आरओएस 2, एक्सएमएल

<h4 className="fourth-heading">स्टार्टर कोड</h4>
<div className="underline-class"></div>

- • कैमरा कॉन्फ़िगरेशन (रिज़ॉल्यूशन, एफओवी)
- • आरओएस 2 प्लगइन
- • शोर मॉडलिंग

<h4 className="fourth-heading">सफलता मानदंड</h4>
<div className="underline-class"></div>

- [ ] कैमरा आरओएस 2 में प्रकाशित करता है
- [ ] डेटा विषयों के माध्यम से सुलभ है
- [ ] शोर कॉन्फ़िगर किया गया है

<h4 className="fourth-heading">परीक्षण कमांड</h4>
<div className="underline-class"></div>
```bash
ros2 launch your_robot_gazebo your_robot.launch.py
ros2 topic list | grep camera
ros2 run image_view image_view _image:=/your_robot/camera/image_raw
```

<h4 className="fourth-heading">संकेत</h4>
<div className="underline-class"></div>

- • टीएफ के लिए उचित फ्रेम नामों का उपयोग करें
- • प्लगइन नामस्थान की पुष्टि करें

</details>

<details>
<summary>अभ्यास 2.3.2: उन्नत सेंसर कॉन्फ़िगरेशन (⭐⭐, ~45 मिनट)</summary>

<h3 className="third-heading">अभ्यास 2.3.2: उन्नत सेंसर कॉन्फ़िगरेशन</h3>
<div className="underline-class"></div>

**कठिनाई**: ⭐⭐ | **समय**: 45 मिनट

<h4 className="fourth-heading">स्टार्टर कोड</h4>
<div className="underline-class"></div>

- • कैमरा, लाइडार, आईएमयू सेंसर
- • शोर मॉडलिंग
- • सेंसर फ्यूजन

<h4 className="fourth-heading">परीक्षण कमांड</h4>
<div className="underline-class"></div>
```bash
ros2 topic list | grep -E "(camera|scan|imu)"
ros2 topic hz /your_robot/camera/image_raw
ros2 run rviz2 rviz2
```

<h4 className="fourth-heading">संकेत</h4>
<div className="underline-class"></div>

- • गुणवत्ता को प्रदर्शन के साथ संतुलित करें
- • उचित अपडेट दर का उपयोग करें

</details>

<details>
<summary>अभ्यास 2.3.3: कस्टम सेंसर प्लगइन (⭐⭐⭐, ~60 मिनट)</summary>

<h3 className="third-heading">अभ्यास 2.3.3: कस्टम सेंसर प्लगइन</h3>
<div className="underline-class"></div>

**कठिनाई**: ⭐⭐⭐ | **समय**: 60 मिनट | **आवश्यकताएँ**: सी++, गेज़बो एपीआई

<h4 className="fourth-heading">स्टार्टर कोड</h4>
<div className="underline-class"></div>

- • गेज़बो सेंसर विस्तारित करें
- • आरओएस 2 के साथ एकीकृत करें
- • कस्टम डेटा प्रसंस्करण

<h4 className="fourth-heading">परीक्षण कमांड</h4>
<div className="underline-class"></div>
```bash
cd ~/ros2_ws
colcon build --packages-select your_robot_gazebo
ros2 launch your_robot_gazebo custom_sensor_demo.launch.py
```

<h4 className="fourth-heading">संकेत</h4>
<div className="underline-class"></div>

- • गेज़बो प्लगइन दिशानिर्देशों का पालन करें
- • पहले अलगाव में प्लगइन का परीक्षण करें

</details>

<div className="border-line"></div>

<h2 className="second-heading">समस्या निवारण</h2>
<div className="underline-class"></div>

<details>
<summary>सामान्य समस्याएँ</summary>

<h3 className="third-heading">समस्या निवारण</h3>
<div className="underline-class"></div>

<h4 className="fourth-heading">शोर/अवास्तविक डेटा</h4>
<div className="underline-class"></div>

**समाधान**:
```xml
<noise><type>gaussian</type><mean>0.0</mean><stddev>0.007</stddev></noise>
<physics><max_step_size>0.001</max_step_size></physics>
```

<h4 className="fourth-heading">गुम सेंसर विषय</h4>
<div className="underline-class"></div>

**समाधान**:
```xml
<plugin name="camera_controller" filename="libgazebo_ros_camera.so">
  <ros><namespace>/robot_name</namespace></ros>
</plugin>
```
```bash
sudo apt install ros-humble-gazebo-ros-pkgs
```

<h4 className="fourth-heading">प्लगइन लोड करने में विफल</h4>
<div className="underline-class"></div>

**समाधान**:
```bash
find ~/ros2_ws/install -name "*plugin*.so"
colcon build --packages-select your_robot_gazebo
```
```cpp
GZ_REGISTER_SENSOR_PLUGIN(YourPluginClassName)
```

<h4 className="fourth-heading">खराब प्रदर्शन</h4>
<div className="underline-class"></div>

**समाधान**:
```xml
<sensor><update_rate>15</update_rate></sensor>
<ray><scan><horizontal><samples>360</samples></horizontal></scan></ray>
```

<h4 className="fourth-heading">डेटा देरी/समकालिक नहीं</h4>
<div className="underline-class"></div>

**समाधान**:
```python
qos_profile = rclpy.qos.QoSProfile(
    depth=1, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT
)
```

</details>

<div className="border-line"></div>

<h2 className="second-heading">सेंसर प्रकार</h2>
<div className="underline-class"></div>

<h3 className="third-heading">सामान्य सेंसर</h3>
<div className="underline-class"></div>

<h4 className="fourth-heading">कैमरा</h4>
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

<h4 className="fourth-heading">लाइडार</h4>
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

<h4 className="fourth-heading">आईएमयू</h4>
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

<h3 className="third-heading">आरओएस 2 एकीकरण</h3>
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
        # प्रक्रिया छवि
        pass
```

<div className="border-line"></div>

<h2 className="second-heading">कस्टम प्लगइन्स</h2>
<div className="underline-class"></div>

<h3 className="third-heading">प्लगइन विकास</h3>
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

<h3 className="third-heading">प्लगइन कॉन्फ़िगरेशन</h3>
<div className="underline-class"></div>
```xml
<sensor name="custom_lidar" type="ray">
  <plugin filename="libCustomSensorPlugin.so">
    <ros><namespace>/robot</namespace></ros>
  </plugin>
</sensor>
```

<div className="border-line"></div>

<h2 className="second-heading">उन्नत मॉडलिंग</h2>
<div className="underline-class"></div>

<h3 className="third-heading">शोर मॉडलिंग</h3>
<div className="underline-class"></div>
```xml
<camera>
  <lens><type>stereographic</type></lens>
  <noise><type>gaussian</type><mean>0.0</mean><stddev>0.007</stddev></noise>
</camera>
```

<h3 className="third-heading">सेंसर फ्यूजन</h3>
<div className="underline-class"></div>
```xml
<!-- मानवरूपी पर कई सेंसर -->
<gazebo reference="head_link">
  <sensor name="head_camera" type="camera"/>
  <sensor name="head_imu" type="imu"/>
</gazebo>
<gazebo reference="torso_link">
  <sensor name="torso_lidar" type="ray"/>
</gazebo>
```

<div className="border-line"></div>

<h2 className="second-heading">सर्वोत्तम प्रथाएँ</h2>
<div className="underline-class"></div>

<h3 className="third-heading">प्रदर्शन अनुकूलन</h3>
<div className="underline-class"></div>

- • अपडेट दर कम करें
- • सेंसर रिज़ॉल्यूशन सीमित करें
- • कुशल शोर मॉडल का उपयोग करें
- • सेंसर को चयनात्मक रूप से सक्रिय करें

<h3 className="third-heading">वास्तविकता बनाम प्रदर्शन</h3>
<div className="underline-class"></div>
```python
class SensorPerformanceTester(Node):
    def __init__(self):
        self.sensor_stats = {'camera': {'count': 0, 'total_time': 0}}
        self.timer = self.create_timer(5.0, self.log_stats)

    def log_stats(self):
        freq = self.sensor_stats['camera']['count'] / 5.0
        self.get_logger().info(f'कैमरा आवृत्ति: {freq:.2f}Hz')
```

<h3 className="third-heading">वैधता</h3>
<div className="underline-class"></div>

- • वास्तविक हार्डवेयर के साथ तुलना करें
- • ग्राउंड ट्रुथ सेंसर का उपयोग करें
- • सांख्यिकीय वैधता
- • पार-वैधता

<h2 className="second-heading">सारांश</h2>
<div className="underline-class"></div>

विशिष्ट कार्यक्षमता के लिए वास्तविक शोर और कस्टम प्लगइन्स के साथ उचित सेंसर मॉडल का कार्यान्वयन करें। सटीक धारणा डेटा बनाए रखते हुए सेंसर वास्तविकता को प्रदर्शन के साथ संतुलित करें।