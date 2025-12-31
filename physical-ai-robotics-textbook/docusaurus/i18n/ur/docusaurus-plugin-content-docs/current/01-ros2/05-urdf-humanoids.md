---
sidebar_position: 6
title: 'ہیومنوائڈ روبوٹس کے لیے URDF: روبوٹ کی تفصیل اور ماڈلنگ'
description: 'ROS 2 میں ہیومنوائڈ روبوٹس کے ماڈل کے لیے URDF (یونیفائیڈ روبوٹ ڈسکرپشن فارمیٹ) کی سمجھ'
---

import ReadingTime from '@site/src/components/ReadingTime';


<ReadingTime minutes={6} />

<h1 className="main-heading">ہیومنوائڈ روبوٹس کے لیے URDF: روبوٹ کی تفصیل اور ماڈلنگ</h1>
<div className="underline-class"></div>

URDF ROS میں روبوٹ ماڈلز کی تفصیل کے لیے معیاری فارمیٹ ہے، جو کنیمیٹک سٹرکچر، جوئنٹس، اور جسمانی خصوصیات کی وضاحت کرتا ہے۔

<div className="border-line"></div>

<h2 className="second-heading">سیکھنے کے اہداف</h2>
<div className="underline-class"></div>

- • ہیومنوائڈ روبوٹس کے لیے URDF ماڈلز بنانا
- • لنکس، جوئنٹس، اور مواد کی وضاحت کرنا
- • کنیمیٹک چینز نافذ کرنا
- • ماڈلز کو آسان بنانے کے لیے Xacro استعمال کرنا
- • RViz میں تصدیق اور تصور کرنا

<div className="border-line"></div>

<h2 className="second-heading">مشقیں</h2>
<div className="underline-class"></div>

<details>
<summary>مشق 1.5.1: بنیادی ہیومنوائڈ URDF (⭐, ~30 منٹ)</summary>

<h3 className="third-heading">مشق 1.5.1: بنیادی ہیومنوائڈ URDF</h3>
<div className="underline-class"></div>

**پیچیدگی**: ⭐ | **وقت**: 30 منٹ

<h4 className="fourth-heading">شروع کرنے والا کوڈ</h4>
<div className="underline-class"></div>

- • بیس کے طور پر ٹورسو
- • سر، باہیں، ٹانگیں
- • مناسب جوئنٹس

<h4 className="fourth-heading">کامیابی کے معیار</h4>
<div className="underline-class"></div>

- [ ] درست XML
- [ ] ویژوئل/کولیژن/انرٹیل خصوصیات
- [ ] RViz میں ڈسپلے

<h4 className="fourth-heading">ٹیسٹ کمانڈز</h4>
<div className="underline-class"></div>
```bash
check_urdf basic_humanoid.urdf
ros2 launch rviz2 rviz2
```

<h4 className="fourth-heading">اشارے</h4>
<div className="underline-class"></div>

- • سادہ شروع کریں، ترقی کے ساتھ شامل کریں
- • مناسب جوئنٹ اقسام استعمال کریں

</details>

<details>
<summary>مشق 1.5.2: Xacro-بیسڈ ماڈل (⭐⭐, ~45 منٹ)</summary>

<h3 className="third-heading">مشق 1.5.2: Xacro-بیسڈ ماڈل</h3>
<div className="underline-class"></div>

**پیچیدگی**: ⭐⭐ | **وقت**: 45 منٹ

<h4 className="fourth-heading">شروع کرنے والا کوڈ</h4>
<div className="underline-class"></div>

- • دوبارہ استعمال کے قابل اعضا میکروز
- • ٹورسو میکرو
- • پیرامیٹرائزڈ جوئنٹس

<h4 className="fourth-heading">ٹیسٹ کمانڈز</h4>
<div className="underline-class"></div>
```bash
xacro humanoid_robot.xacro > humanoid_robot.urdf
check_urdf humanoid_robot.urdf
```

<h4 className="fourth-heading">اشارے</h4>
<div className="underline-class"></div>

- • ابعاد کے لیے خصوصیات استعمال کریں
- • میکروز کو انفرادی طور پر ٹیسٹ کریں

</details>

<details>
<summary>مشق 1.5.3: اعلیٰ خصوصیات (⭐⭐⭐, ~60 منٹ)</summary>

<h3 className="third-heading">مشق 1.5.3: اعلیٰ خصوصیات</h3>
<div className="underline-class"></div>

**پیچیدگی**: ⭐⭐⭐ | **وقت**: 60 منٹ

<h4 className="fourth-heading">شروع کرنے والا کوڈ</h4>
<div className="underline-class"></div>

- • ٹرانسمیشن اجزاء
- • گیزبو پلگ انز
- • حقیقی انرٹیل خصوصیات

<h4 className="fourth-heading">ٹیسٹ کمانڈز</h4>
<div className="underline-class"></div>
```bash
ros2 launch gazebo_ros gazebo
ros2 run tf2_tools view_frames
```

<h4 className="fourth-heading">اشارے</h4>
<div className="underline-class"></div>

- • حقیقی انرٹیل ویلیوز استعمال کریں
- • ہر خصوصیت کی تصدیق کریں

</details>

<div className="border-line"></div>

<h2 className="second-heading">ٹربل شوٹنگ</h2>
<div className="underline-class"></div>

<details>
<summary>عام مسائل</summary>

<h3 className="third-heading">ٹربل شوٹنگ</h3>
<div className="underline-class"></div>

<h4 className="fourth-heading">ماڈل/غلط جوئنٹس میں تبدیلی</h4>
<div className="underline-class"></div>

**حل**:
```xml
<joint name="joint" type="revolute">
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
</joint>
```
```bash
check_urdf robot.urdf
urdf_to_graphiz robot.urdf
```

<h4 className="fourth-heading">RViz میں روبوٹ نہیں ہے</h4>
<div className="underline-class"></div>

**حل**:
```bash
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:=$(cat robot.urdf)
ros2 topic echo /joint_states
ros2 run tf2_tools view_frames
```

<h4 className="fourth-heading">سیمولیشن میں روبوٹ گرتا ہے</h4>
<div className="underline-class"></div>

**حل**:
```xml
<inertial>
  <mass value="1.0"/>
  <origin xyz="0 0 0"/>
  <inertia ixx="0.01" iyy="0.01" izz="0.01"/>
</inertial>
```

<h4 className="fourth-heading">Xacro کمپائلیشن کی غلطیاں</h4>
<div className="underline-class"></div>

**حل**:
```xml
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="robot">
<xacro:macro name="macro" params="param1 param2:=default">...</xacro:macro>
<xacro:property name="value" value="1.0"/>
```

</details>

<div className="border-line"></div>

<h2 className="second-heading">URDF بنیادیں</h2>
<div className="underline-class"></div>

<h3 className="third-heading">اہم اجزاء</h3>
<div className="underline-class"></div>

- • **لنکس**: سخت جسم
- • **جوئنٹس**: ڈیگریز آف فریڈم کے ساتھ کنکشنز
- • **مواد**: ویژوئل خصوصیات
- • **انرٹیل**: ماس، COM، انرٹیا
- • **کولیژن**: سادہ جیومیٹری
- • **ویژوئل**: تفصیلی جیومیٹری

<h3 className="third-heading">بنیادی سٹرکچر</h3>
<div className="underline-class"></div>
```xml
<robot name="humanoid">
  <link name="base_link">
    <visual><geometry><box size="0.5 0.2 0.2"/></geometry></visual>
    <collision><geometry><box size="0.5 0.2 0.2"/></geometry></collision>
    <inertial><mass value="1.0"/></inertial>
  </link>

  <joint name="base_to_torso" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.3"/>
  </joint>
</robot>
```

<div className="border-line"></div>

<h2 className="second-heading">ہیومنوائڈ سٹرکچر</h2>
<div className="underline-class"></div>
```
base_link
└── torso
    ├── head
    ├── left_arm → left_forearm → left_hand
    ├── right_arm → right_forearm → right_hand
    ├── left_leg → left_lower_leg → left_foot
    └── right_leg → right_lower_leg → right_foot
```

<h3 className="third-heading">جوئنٹ اقسام</h3>
<div className="underline-class"></div>

- • **ریوولوٹ**: سنگل ایکس ریوٹیشن
- • **کنٹینیوس**: لامحدود ریوٹیشن
- • **پریزمیٹک**: لکیری موشن
- • **فکسڈ**: سخت کنکشنز

<div className="border-line"></div>

<h2 className="second-heading">مکمل ہیومنوائڈ مثال</h2>
<div className="underline-class"></div>
```xml
<robot name="humanoid">
  <link name="torso">
    <inertial><mass value="5.0"/><inertia ixx="0.5" iyy="0.5" izz="0.5"/></inertial>
    <visual><geometry><box size="0.3 0.3 0.6"/></geometry></visual>
  </link>

  <joint name="torso_to_head" type="revolute">
    <parent link="torso"/><child link="head"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.0" upper="1.0" effort="10.0" velocity="1.0"/>
  </joint>

  <link name="head">
    <visual><geometry><sphere radius="0.15"/></geometry></visual>
  </link>

  <joint name="torso_to_left_arm" type="revolute">
    <parent link="torso"/><child link="left_upper_arm"/>
    <origin xyz="0.25 0 0.3"/>
    <axis xyz="0 1 0"/>
  </joint>
</robot>
```

<div className="border-line"></div>

<h2 className="second-heading">Xacro میکروز</h2>
<div className="underline-class"></div>
```xml
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid">
  <xacro:property name="torso_height" value="0.6"/>

  <xacro:macro name="limb" params="name parent">
    <joint name="${parent}_to_${name}" type="revolute">
      <parent link="${parent}"/><child link="${name}"/>
      <axis xyz="0 1 0"/>
    </joint>
    <link name="${name}">
      <visual><geometry><cylinder radius="0.05" length="0.3"/></geometry></visual>
    </link>
  </xacro:macro>

  <xacro:limb name="left_arm" parent="torso"/>
  <xacro:limb name="right_arm" parent="torso"/>
</robot>
```

<div className="border-line"></div>

<h2 className="second-heading">اعلیٰ خصوصیات</h2>
<div className="underline-class"></div>

<h3 className="third-heading">ٹرانسمیشنز</h3>
<div className="underline-class"></div>
```xml
<transmission name="tran1">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="torso_to_head">
    <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
  </joint>
  <actuator name="motor1"><mechanicalReduction>1</mechanicalReduction></actuator>
</transmission>
```

<h3 className="third-heading">گیزبو اجزاء</h3>
<div className="underline-class"></div>
```xml
<gazebo reference="torso">
  <material>Gazebo/Gray</material>
</gazebo>

<gazebo>
  <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
    <robotNamespace>/humanoid</robotNamespace>
  </plugin>
</gazebo>
```

<div className="border-line"></div>

<h2 className="second-heading">تصدیق اور تصور</h2>
<div className="underline-class"></div>
```bash
check_urdf robot.urdf
urdf_to_graphiz robot.urdf
ros2 run rviz2 rviz2
```

<h3 className="third-heading">روبوٹ اسٹیٹ پبلشر</h3>
<div className="underline-class"></div>
```python
from sensor_msgs.msg import JointState

class RobotStatePublisher(Node):
    def __init__(self):
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

    def publish_joint_states(self):
        msg = JointState()
        msg.name = ['joint1', 'joint2']
        msg.position = [0.0, 1.57]
        self.joint_pub.publish(msg)
```

<div className="border-line"></div>

<h2 className="second-heading">بہترین مشقیں</h2>
<div className="underline-class"></div>

- • انسان کی طرح جوئنٹ حدود کو فالو کریں
- • مناسب ماس تقسیم
- • سادہ کولیژن جیومیٹری
- • پیچیدہ ماڈلز کے لیے Xacro استعمال کریں
- • جوئنٹ مقاصد کو دستاویز کریں
- • منطقی طور پر منظم کریں

<h2 className="second-heading">خلاصہ</h2>
<div className="underline-class"></div>

URDF لنکس، جوئنٹس، اور خصوصیات کے ساتھ روبوٹ سٹرکچر کی وضاحت کرتا ہے۔ برقرار رکھنے کے قابل کے لیے Xacro استعمال کریں اور استعمال سے پہلے ماڈلز کی تصدیق کریں۔