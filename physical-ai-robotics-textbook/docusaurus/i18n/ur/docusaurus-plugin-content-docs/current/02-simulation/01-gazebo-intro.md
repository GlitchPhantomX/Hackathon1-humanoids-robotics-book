---
sidebar_position: 2
title: 'گیزبو تعارف: روبوٹکس کے لیے سیمولیشن'
description: 'روبوٹکس ڈویلپمنٹ کے لیے گیزبو سیمولیشن ماحول کے ساتھ شروعات'
---

import ReadingTime from '@site/src/components/ReadingTime';


<ReadingTime minutes={4} />

<h1 className="main-heading">گیزبو تعارف: روبوٹکس کے لیے سیمولیشن</h1>
<div className="underline-class"></div>

گیزبو روبوٹکس کے لیے ایک طاقتور 3D سیمولیشن ماحول ہے جس میں حقیقی طرز کی فزکس، گریفکس، اور ROS 2 انضمام ہے۔

<div className="border-line"></div>

<h2 className="second-heading">سیکھنے کے اہداف</h2>
<div className="underline-class"></div>

- • گیزبو آرکیٹیکچر کو سمجھنا
- • گیزبو انسٹال کرنا اور ترتیب دینا
- • سیمولیشن ورلڈز بنانا
- • ROS 2 کے ساتھ انضمام
- • سیمولیٹڈ روبوٹس کو کنٹرول کرنا

<div className="border-line"></div>

<h2 className="second-heading">مشقیں</h2>
<div className="underline-class"></div>

<details>
<summary>مشق 2.1.1: انسٹالیشن اور بنیادی سیمولیشن (⭐, ~30 منٹ)</summary>

<h3 className="third-heading">مشق 2.1.1: انسٹالیشن اور بنیادی سیمولیشن</h3>
<div className="underline-class"></div>

**پیچیدگی**: ⭐ | **وقت**: 30 منٹ | **ضروریات**: Ubuntu، ROS 2

<h4 className="fourth-heading">شروع کرنے والا کوڈ</h4>
<div className="underline-class"></div>

- • گیزبو گارڈن انسٹال کریں
- • ڈیفالٹ ورلڈ لانچ کریں
- • انٹرفیس کو ایکسپلور کریں

<h4 className="fourth-heading">کامیابی کے معیار</h4>
<div className="underline-class"></div>

- [ ] گیزبو کامیابی سے لانچ ہوتا ہے
- [ ] کیمرہ کنٹرولز کام کرتے ہیں
- [ ] سیمولیشن ہموار چلتا ہے

<h4 className="fourth-heading">ٹیسٹ کمانڈز</h4>
<div className="underline-class"></div>
```bash
sudo apt update
sudo apt install gazebo-garden ros-humble-gazebo-ros-pkgs
gazebo
```

<h4 className="fourth-heading">اشارے</h4>
<div className="underline-class"></div>

- • گریفکس ڈرائیورز چیک کریں
- • سسٹم کی ضروریات کی تصدیق کریں

</details>

<details>
<summary>مشق 2.1.2: حسب ضرورت ورلڈ تخلیق (⭐⭐, ~45 منٹ)</summary>

<h3 className="third-heading">مشق 2.1.2: حسب ضرورت ورلڈ تخلیق</h3>
<div className="underline-class"></div>

**پیچیدگی**: ⭐⭐ | **وقت**: 45 منٹ

<h4 className="fourth-heading">شروع کرنے والا کوڈ</h4>
<div className="underline-class"></div>

- • زمینی سطح اور لائٹنگ
- • متعدد اشیاء
- • روبوٹ ماڈل
- • فزکس پیرامیٹرز

<h4 className="fourth-heading">ٹیسٹ کمانڈز</h4>
<div className="underline-class"></div>
```bash
gz sdf -k your_world.sdf
gazebo your_world.sdf
```

<h4 className="fourth-heading">اشارے</h4>
<div className="underline-class"></div>

- • ڈیفالٹ ٹیمپلیٹ کے ساتھ شروع کریں
- • ٹیسٹ کرنے سے پہلے SDF کی توثیق کریں

</details>

<details>
<summary>مشق 2.1.3: ROS 2 انضمام (⭐⭐⭐, ~60 منٹ)</summary>

<h3 className="third-heading">مشق 2.1.3: ROS 2 انضمام</h3>
<div className="underline-class"></div>

**پیچیدگی**: ⭐⭐⭐ | **وقت**: 60 منٹ

<h4 className="fourth-heading">شروع کرنے والا کوڈ</h4>
<div className="underline-class"></div>

- • URDF روبوٹ ماڈل
- • لانچ فائل
- • روبوٹ/جوائنٹ اسٹیٹ پبلشرز
- • کنٹرولر نوڈ

<h4 className="fourth-heading">ٹیسٹ کمانڈز</h4>
<div className="underline-class"></div>
```bash
ros2 launch your_robot_gazebo your_robot.launch.py
ros2 topic list
```

<h4 className="fourth-heading">اشارے</h4>
<div className="underline-class"></div>

- • robot_state_publisher استعمال کریں
- • اجزاء کو الگ الگ ٹیسٹ کریں

</details>

<div className="border-line"></div>

<h2 className="second-heading">ٹربل شوٹنگ</h2>
<div className="underline-class"></div>

<details>
<summary>عام مسائل</summary>

<h3 className="third-heading">ٹربل شوٹنگ</h3>
<div className="underline-class"></div>

<h4 className="fourth-heading">گیزبو شروع نہیں ہوتا</h4>
<div className="underline-class"></div>

**حل**:
```bash
glxinfo | grep "OpenGL version"
sudo apt install mesa-utils
export LIBGL_ALWAYS_SOFTWARE=1
```

<h4 className="fourth-heading">روبوٹ زمین کے ذریعے گرتا ہے</h4>
<div className="underline-class"></div>

**حل**:
```xml
<inertial><mass value="1.0"/>
<inertia ixx="0.01" iyy="0.01" izz="0.01"/></inertial>
```

<h4 className="fourth-heading">سست سیمولیشن</h4>
<div className="underline-class"></div>

**حل**:
```xml
<sensor><update_rate>30</update_rate></sensor>
<collision><geometry><box><size>0.5 0.5 0.5</size></box></geometry></collision>
<physics><max_step_size>0.01</max_step_size></physics>
```

<h4 className="fourth-heading">ROS 2 انضمام ناکام ہو جاتا ہے</h4>
<div className="underline-class"></div>

**حل**:
```xml
<plugin name="joint_state_publisher" filename="libgazebo_ros_joint_state_publisher.so"/>
```
```bash
sudo apt install ros-humble-gazebo-ros-pkgs
```

<h4 className="fourth-heading">میڈلز/ٹیکسچرز غائب ہیں</h4>
<div className="underline-class"></div>

**حل**:
```bash
echo $GAZEBO_MODEL_PATH
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/path/to/models
gz sdf -k model.sdf
```

</details>

<div className="border-line"></div>

<h2 className="second-heading">گیزبو کیا ہے؟</h2>
<div className="underline-class"></div>

روبوٹس کے لیے 3D ڈائینامک سیمولیٹر جس میں ہے:
- • **حقیقی طرز کی فزکس**: ODE، بُلیٹ، DART
- • **گریفکس**: OGRE رینڈرنگ
- • **سینسرز**: کیمرہ، LIDAR، IMU، GPS
- • **پلگ انز**: توسیع پذیر آرکیٹیکچر
- • **ROS انضمام**: بے داغ ROS 2 سپورٹ

<h3 className="third-heading">اہم خصوصیات</h3>
<div className="underline-class"></div>

- • پیچیدہ کنیمیٹکس کی حمایت
- • حقیقی طرز کی فزکس اور توازن
- • سینسر سیمولیشن
- • اعلی درجے کا کولیژن ڈیٹیکشن
- • پیچیدہ ماحول

<div className="border-line"></div>

<h2 className="second-heading">انسٹالیشن</h2>
<div className="underline-class"></div>
```bash
sudo apt update
sudo apt install gazebo-garden ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control
```

<div className="border-line"></div>

<h2 className="second-heading">بنیادی تصورات</h2>
<div className="underline-class"></div>

<h3 className="third-heading">ورلڈ فائلز</h3>
<div className="underline-class"></div>
```xml
<world name="default">
  <include><uri>model://ground_plane</uri></include>
  <include><uri>model://sun</uri></include>
  <model name="box">
    <pose>0 0 0.5 0 0 0</pose>
    <link name="link">
      <collision><geometry><box><size>1 1 1</size></box></geometry></collision>
      <visual><geometry><box><size>1 1 1</size></box></geometry></visual>
    </link>
  </model>
</world>
```

<h3 className="third-heading">ماڈل فائلز</h3>
<div className="underline-class"></div>
```xml
<model name="my_robot">
  <link name="chassis">
    <collision><geometry><box><size>1 0.5 0.2</size></box></geometry></collision>
    <visual><geometry><box><size>1 0.5 0.2</size></box></geometry></visual>
    <inertial><mass>1.0</mass></inertial>
  </link>
</model>
```

<div className="border-line"></div>

<h2 className="second-heading">گیزبو چلانا</h2>
<div className="underline-class"></div>
```bash
gazebo
gazebo /path/to/world.world
ros2 launch gazebo_ros gazebo.launch.py
ros2 launch gazebo_ros gazebo.launch.py world:=/path/to/world.sdf
```

<h3 className="third-heading">GUI کنٹرولز</h3>
<div className="underline-class"></div>

- • **اوروٹ**: دائیں کلک + ڈریگ
- • **پین**: شفٹ + دائیں کلک + ڈریگ
- • **زوم**: سکرول وہیل
- • **فوكس**: ڈبل کلک آبجیکٹ

<div className="border-line"></div>

<h2 className="second-heading">ROS 2 انضمام</h2>
<div className="underline-class"></div>

<h3 className="third-heading">روبوٹس اسپون کرنا</h3>
<div className="underline-class"></div>
```python
from gazebo_msgs.srv import SpawnEntity

class RobotSpawner(Node):
    def spawn_robot(self, name, xml, pose):
        req = SpawnEntity.Request()
        req.name = name
        req.xml = xml
        req.initial_pose = pose
        return self.cli.call_async(req)
```

<h3 className="third-heading">گیزبو پلگ انز</h3>
<div className="underline-class"></div>
```xml
<!-- Joint control -->
<plugin name="joint_state_publisher" filename="libgazebo_ros_joint_state_publisher.so">
  <update_rate>30</update_rate>
</plugin>

<!-- Camera -->
<plugin name="camera_controller" filename="libgazebo_ros_camera.so">
  <ros><namespace>/robot</namespace></ros>
</plugin>

<!-- IMU -->
<plugin name="imu_plugin" filename="libgazebo_ros_imu.so">
  <ros><remapping>~/out:=/imu/data</remapping></ros>
</plugin>
```

<div className="border-line"></div>

<h2 className="second-heading">بہترین مشقیں</h2>
<div className="underline-class"></div>

<h3 className="third-heading">کارکردگی</h3>
<div className="underline-class"></div>

- • • سینسر اپ ڈیٹ کی شرح کم کریں
- • • کولیژن جیومیٹری کو سادہ بنائیں
- • • فزکس اسٹیپس کو محدود کریں
- • • مناسب ورلڈ سائز

<h3 className="third-heading">ریلزم</h3>
<div className="underline-class"></div>

- • • درست انرٹیل خصوصیات
- • • مناسب جوائنٹ حدود
- • • حقیقی طرز کا سینسر نوائز
- • • درست فرکشن ویلیوز

<h3 className="third-heading">ڈیبگنگ</h3>
<div className="underline-class"></div>

- • • وائر فریم موڈ (پریس 'W')
- • • کنٹیکٹ وژولائزیشن فعال کریں
- • • ریل ٹائم فیکٹر کو مانیٹر کریں
- • • بلٹ ان ٹولز استعمال کریں

<div className="border-line"></div>

<h2 className="second-heading">خلاصہ</h2>
<div className="underline-class"></div>

گیزبو حقیقی طرز کی فزکس، حقیقی طرز کے سینسرز، اور ROS 2 انضمام کے ساتھ طاقتور سیمولیشن فراہم کرتا ہے جو روبوٹکس ڈویلپمنٹ کے لیے ہارڈ ویئر ڈپلائمنٹ سے پہلے ہے۔

<h2 className="second-heading">وسائل</h2>
<div className="underline-class"></div>

- • [گیزبو دستاویزات](http://gazebosim.org/)
- • [ROS 2 گیزبو](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Gazebo.html)
- • [گیزبو ٹیوٹوریلز](http://gazebosim.org/tutorials)