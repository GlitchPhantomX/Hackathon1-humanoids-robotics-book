---
sidebar_position: 3
title: 'URDF اور SDF: روبوٹ اور ماحول کی ماڈلنگ'
description: 'سیمولیشن ماحول کی ماڈلنگ کے لیے URDF اور SDF کی سمجھ'
---



<h1 className="main-heading">URDF اور SDF: روبوٹ اور ماحول کی ماڈلنگ</h1>
<div className="underline-class"></div>

Gazebo سیمولیشن میں روبوٹ ماڈلنگ کے لیے URDF اور ماحول کی ماڈلنگ کے لیے SDF کو سمجھیں۔

<div className="border-line"></div>

<h2 className="second-heading">سیکھنے کے اہداف</h2>
<div className="underline-class"></div>

- • URDF اور SDF کے فرق کو سمجھنا
- • SDF ماحول ماڈلز بنانا
- • URDF روبوٹس کو SDF کے ساتھ انضمام کرنا
- • Gazebo ایکسٹینشنز استعمال کرنا
- • پیچیدہ سیمولیشن منظرنامے بنانا

<div className="border-line"></div>

<h2 className="second-heading">مشقیں</h2>
<div className="underline-class"></div>

<details>
<summary>مشق 2.2.1: بنیادی SDF ماحول (⭐, ~30 منٹ)</summary>

<h3 className="third-heading">مشق 2.2.1: بنیادی SDF ماحول</h3>
<div className="underline-class"></div>

**پیچیدگی**: ⭐ | **وقت**: 30 منٹ | **ضروریات**: Gazebo، XML

<h4 className="fourth-heading">شروع کرنے والا کوڈ</h4>
<div className="underline-class"></div>

- • فزکس کنفیگریشن
- • زمینی سطح اور لائٹنگ
- • سادہ باکس ماڈل

<h4 className="fourth-heading">کامیابی کے معیار</h4>
<div className="underline-class"></div>

- [ ] SDF فائل درست ہے
- [ ] Gazebo میں ورلڈ لوڈ ہوتا ہے
- [ ] فزکس مناسب طریقے سے کام کرتا ہے

<h4 className="fourth-heading">ٹیسٹ کمانڈز</h4>
<div className="underline-class"></div>
```bash
gz sdf -k your_world.sdf
gazebo your_world.sdf
```

<h4 className="fourth-heading">اشارے</h4>
<div className="underline-class"></div>

- • ٹیمپلیٹ کے طور پر ڈیفالٹ ورلڈ استعمال کریں
- • پہلے سینٹیکس کی توثیق کریں

</details>

<details>
<summary>مشق 2.2.2: URDF-SDF انضمام (⭐⭐, ~45 منٹ)</summary>

<h3 className="third-heading">مشق 2.2.2: URDF-SDF انضمام</h3>
<div className="underline-class"></div>

**پیچیدگی**: ⭐⭐ | **وقت**: 45 منٹ

<h4 className="fourth-heading">شروع کرنے والا کوڈ</h4>
<div className="underline-class"></div>

- • URDF روبوٹ ماڈل
- • SDF ورلڈ فائل
- • Gazebo ROS 2 پلگ انز
- • جوائنٹ ٹرانسمیشنز

<h4 className="fourth-heading">ٹیسٹ کمانڈز</h4>
<div className="underline-class"></div>
```bash
gz sdf -p robot.urdf > robot.sdf
ros2 launch your_robot_gazebo your_simulation.launch.py
ros2 topic list | grep joint
```

<h4 className="fourth-heading">اشارے</h4>
<div className="underline-class"></div>

- • مناسب Gazebo پلگ انز استعمال کریں
- • کنٹرولرز کے ساتھ جوائنٹ ناموں کو میچ کریں

</details>

<details>
<summary>مشق 2.2.3: اعلیٰ ماحول (⭐⭐⭐, ~60 منٹ)</summary>

<h3 className="third-heading">مشق 2.2.3: اعلیٰ ماحول</h3>
<div className="underline-class"></div>

**پیچیدگی**: ⭐⭐⭐ | **وقت**: 60 منٹ

<h4 className="fourth-heading">شروع کرنے والا کوڈ</h4>
<div className="underline-class"></div>

- • متعدد کمرے/ منزلیں
- • فرنیچر اور رکاوٹیں
- • بہتر فزکس پیرامیٹرز
- • Gazebo سینسر ایکسٹینشنز

<h4 className="fourth-heading">ٹیسٹ کمانڈز</h4>
<div className="underline-class"></div>
```bash
gz sdf -k complex_environment.sdf
gazebo --verbose complex_environment.sdf
gz topic -e /stats
```

<h4 className="fourth-heading">اشارے</h4>
<div className="underline-class"></div>

- • ماحول کے لیے اسٹیٹک ماڈلز استعمال کریں
- • کولیژن جیومیٹری کو بہتر بنائیں

</details>

<div className="border-line"></div>

<h2 className="second-heading">ٹربل شوٹنگ</h2>
<div className="underline-class"></div>

<details>
<summary>عام مسائل</summary>

<h3 className="third-heading">ٹربل شوٹنگ</h3>
<div className="underline-class"></div>

<h4 className="fourth-heading">URDF کنورژن ناکام ہو جاتا ہے</h4>
<div className="underline-class"></div>

**حل**:
```bash
find /path -name "*.dae" -o -name "*.stl"
check_urdf robot.urdf
gz sdf -p robot.urdf > robot.sdf
```

<h4 className="fourth-heading">فزکس اسٹیبلٹی</h4>
<div className="underline-class"></div>

**حل**:
```xml
<inertial><mass value="1.0"/><inertia ixx="0.01" iyy="0.01" izz="0.01"/></inertial>
<physics><max_step_size>0.001</max_step_size></physics>
<joint><axis><dynamics><damping>1.0</damping></dynamics></axis></joint>
```

<h4 className="fourth-heading">ROS 2 انضمام ناکام ہو جاتا ہے</h4>
<div className="underline-class"></div>

**حل**:
```xml
<plugin name="joint_state_publisher" filename="libgazebo_ros_joint_state_publisher.so">
  <ros><namespace>/robot_name</namespace></ros>
</plugin>
```
```bash
sudo apt install ros-humble-gazebo-ros-pkgs
```

<h4 className="fourth-heading">سست کارکردگی</h4>
<div className="underline-class"></div>

**حل**:
```xml
<collision><geometry><box><size>0.5 0.5 0.5</size></box></geometry></collision>
<sensor><update_rate>30</update_rate></sensor>
<physics><max_step_size>0.01</max_step_size></physics>
```

<h4 className="fourth-heading">جوائنٹ حدود کے مسائل</h4>
<div className="underline-class"></div>

**حل**:
```xml
<joint type="revolute">
  <limit lower="-1.57" upper="1.57" effort="100" velocity="3.0"/>
</joint>
<transmission name="joint_trans">
  <joint name="joint_name">
    <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
  </joint>
</transmission>
```

</details>

<div className="border-line"></div>

<h2 className="second-heading">URDF بمقابلہ SDF</h2>
<div className="underline-class"></div>

<h3 className="third-heading">URDF</h3>
<div className="underline-class"></div>

** مضبوطیاں **:
- • روبوٹ کنیمیٹک چینز
- • جوائنٹ حدود اور اقسام
- • ROS انضمام
- • Xacro کے ساتھ پیرامیٹرائزیشن

<h3 className="third-heading">SDF</h3>
<div className="underline-class"></div>

** مضبوطیاں **:
- • مکمل سیمولیشن ماحول
- • فزکس خصوصیات
- • Gazebo پلگ انز اور سینسرز
- • متعدد ماڈل انکلیوژن

<div className="border-line"></div>

<h2 className="second-heading">URDF کو SDF میں تبدیل کرنا</h2>
<div className="underline-class"></div>

<h3 className="third-heading">براہ راست URDF میں Gazebo</h3>
<div className="underline-class"></div>
```xml
<world name="default">
  <include><uri>model://ground_plane</uri></include>
  <include><uri>file://$(find robot)/urdf/humanoid.urdf</uri></include>
</world>
```

<h3 className="third-heading">URDF کو لپیٹنا</h3>
<div className="underline-class"></div>
```xml
<model name="humanoid_robot">
  <include><uri>model://humanoid_robot_model</uri></include>
  <plugin filename="libgazebo_ros_control.so"/>
</model>
```

<div className="border-line"></div>

<h2 className="second-heading">SDF ماحول</h2>
<div className="underline-class"></div>

<h3 className="third-heading">ورلڈ سٹرکچر</h3>
<div className="underline-class"></div>
```xml
<world name="my_world">
  <physics type="ode">
    <max_step_size>0.001</max_step_size>
    <gravity>0 0 -9.8</gravity>
  </physics>
  <light type="directional"><direction>-0.4 0.2 -0.9</direction></light>
  <include><uri>model://ground_plane</uri></include>
</world>
```

<h3 className="third-heading">اپنی ماڈلز</h3>
<div className="underline-class"></div>
```xml
<model name="room">
  <link name="floor">
    <collision><geometry><box><size>10 10 0.1</size></box></geometry></collision>
    <visual><geometry><box><size>10 10 0.1</size></box></geometry></visual>
  </link>
  <link name="wall">
    <pose>0 5 1.5 0 0 0</pose>
    <collision><geometry><box><size>10 0.1 3</size></box></geometry></collision>
  </link>
</model>
```

<div className="border-line"></div>

<h2 className="second-heading">اعلیٰ خصوصیات</h2>
<div className="underline-class"></div>

<h3 className="third-heading">جوائنٹ ٹرانسمیشنز</h3>
<div className="underline-class"></div>
```xml
<joint name="elbow_joint" type="revolute">
  <axis><xyz>1 0 0</xyz>
    <limit><lower>-2.0</lower><upper>1.0</upper></limit>
  </axis>
</joint>
<transmission name="elbow_trans">
  <joint name="elbow_joint">
    <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
  </joint>
</transmission>
```

<h3 className="third-heading">Gazebo ایکسٹینشنز</h3>
<div className="underline-class"></div>
```xml
<gazebo>
  <plugin name="joint_state_publisher" filename="libgazebo_ros_joint_state_publisher.so">
    <update_rate>30</update_rate>
  </plugin>
</gazebo>
<gazebo reference="foot">
  <mu1>0.8</mu1><mu2>0.8</mu2>
</gazebo>
```

<div className="border-line"></div>

<h2 className="second-heading">ہیومنوائڈ کے لیے فزکس</h2>
<div className="underline-class"></div>

<h3 className="third-heading">فزکس پیرامیٹرز</h3>
<div className="underline-class"></div>
```xml
<physics type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_update_rate>1000</real_time_update_rate>
  <ode>
    <solver><type>quick</type><iters>20</iters></solver>
    <constraints><cfm>1e-5</cfm><erp>0.2</erp></constraints>
  </ode>
</physics>
```

<h3 className="third-heading">جوائنٹ ڈیمپنگ</h3>
<div className="underline-class"></div>
```xml
<joint name="knee_joint" type="revolute">
  <axis>
    <limit><lower>0</lower><upper>2.5</upper></limit>
    <dynamics><damping>1.0</damping><friction>0.1</friction></dynamics>
  </axis>
</joint>
```

<div className="border-line"></div>

<h2 className="second-heading">بہترین مشقیں</h2>
<div className="underline-class"></div>

- • تفصیلی نام استعمال کریں
- • کولیژن جیومیٹری کو سادہ بنائیں
- • ماحول کے لیے اسٹیٹک ماڈلز استعمال کریں
- • URDF-SDF کنورژن ٹیسٹ کریں
- • پیچیدہ ماڈلز کو دستاویز کریں

<h2 className="second-heading">ROS 2 لانچ انضمام</h2>
<div className="underline-class"></div>
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('gazebo_ros'),
            '/launch/gazebo.launch.py'
        ]),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    )
    return LaunchDescription([gazebo_launch])
```

<h2 className="second-heading">خلاصہ</h2>
<div className="underline-class"></div>

URDF روبوٹس کی وضاحت کرتا ہے، SDF مکمل سیمولیشن ماحول کی وضاحت کرتا ہے۔ حقیقی طرز کے ہیومنوائڈ روبوٹ سیمولیشن کے لیے دونوں فارمیٹس کو جوڑنے کے لیے Gazebo ایکسٹینشنز استعمال کریں۔