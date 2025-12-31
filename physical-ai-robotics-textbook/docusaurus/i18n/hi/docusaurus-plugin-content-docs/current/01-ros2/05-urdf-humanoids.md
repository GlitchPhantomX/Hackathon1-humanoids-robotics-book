---
sidebar_position: 6
title: 'मानवरूपी रोबोट के लिए यूआरडीएफ: रोबोट विवरण और मॉडलिंग'
description: 'आरओएस 2 में मानवरूपी रोबोट के मॉडलिंग के लिए यूआरडीएफ (यूनिफाइड रोबोट डिस्क्रिप्शन फॉर्मेट) की समझ'
---

import ReadingTime from '@site/src/components/ReadingTime';

<ReadingTime minutes={6} />

<h1 className="main-heading">मानवरूपी रोबोट के लिए यूआरडीएफ: रोबोट विवरण और मॉडलिंग</h1>
<div className="underline-class"></div>

यूआरडीएफ आरओएस में रोबोट मॉडल का वर्णन करने के लिए मानक प्रारूप है, जो काइनेमेटिक संरचना, जॉइंट्स और भौतिक गुणों को परिभाषित करता है।

<div className="border-line"></div>

<h2 className="second-heading">सीखने के उद्देश्य</h2>
<div className="underline-class"></div>

- • मानवरूपी रोबोट के लिए यूआरडीएफ मॉडल बनाएं
- • लिंक्स, जॉइंट्स और सामग्री परिभाषित करें
- • काइनेमेटिक चेन लागू करें
- • मॉडल को सरल बनाने के लिए एक्सैक्रो का उपयोग करें
- • आरवीआईज़ में मान्यता और दृश्यकरण करें

<div className="border-line"></div>

<h2 className="second-heading">अभ्यास</h2>
<div className="underline-class"></div>

<details>
<summary>अभ्यास 1.5.1: मूल मानवरूपी यूआरडीएफ (⭐, ~30 मिनट)</summary>

<h3 className="third-heading">अभ्यास 1.5.1: मूल मानवरूपी यूआरडीएफ</h3>
<div className="underline-class"></div>

**कठिनाई**: ⭐ | **समय**: 30 मिनट

<h4 className="fourth-heading">स्टार्टर कोड</h4>
<div className="underline-class"></div>

- • टोर्सो आधार के रूप में
- • सिर, हाथ, पैर
- • उचित जॉइंट्स

<h4 className="fourth-heading">सफलता मानदंड</h4>
<div className="underline-class"></div>

- [ ] वैध एक्सएमएल
- [ ] दृश्य/टकराव/जड़त्व गुण
- [ ] आरवीआईज़ में प्रदर्शित होता है

<h4 className="fourth-heading">परीक्षण कमांड</h4>
<div className="underline-class"></div>
```bash
check_urdf basic_humanoid.urdf
ros2 launch rviz2 rviz2
```

<h4 className="fourth-heading">संकेत</h4>
<div className="underline-class"></div>

- • सरल शुरू करें, बढ़ोतरी के साथ जोड़ें
- • उपयुक्त जॉइंट प्रकार का उपयोग करें

</details>

<details>
<summary>अभ्यास 1.5.2: एक्सैक्रो-आधारित मॉडल (⭐⭐, ~45 मिनट)</summary>

<h3 className="third-heading">अभ्यास 1.5.2: एक्सैक्रो-आधारित मॉडल</h3>
<div className="underline-class"></div>

**कठिनाई**: ⭐⭐ | **समय**: 45 मिनट

<h4 className="fourth-heading">स्टार्टर कोड</h4>
<div className="underline-class"></div>

- • पुन: उपयोग योग्य अंग मैक्रो
- • टोर्सो मैक्रो
- • पैरामीटराइज़ जॉइंट्स

<h4 className="fourth-heading">परीक्षण कमांड</h4>
<div className="underline-class"></div>
```bash
xacro humanoid_robot.xacro > humanoid_robot.urdf
check_urdf humanoid_robot.urdf
```

<h4 className="fourth-heading">संकेत</h4>
<div className="underline-class"></div>

- • आयाम के लिए गुणों का उपयोग करें
- • मैक्रो की व्यक्तिगत रूप से जांच करें

</details>

<details>
<summary>अभ्यास 1.5.3: उन्नत विशेषताएं (⭐⭐⭐, ~60 मिनट)</summary>

<h3 className="third-heading">अभ्यास 1.5.3: उन्नत विशेषताएं</h3>
<div className="underline-class"></div>

**कठिनाई**: ⭐⭐⭐ | **समय**: 60 मिनट

<h4 className="fourth-heading">स्टार्टर कोड</h4>
<div className="underline-class"></div>

- • संचरण तत्व
- • गेज़बो प्लगइन
- • वास्तविक जड़त्व गुण

<h4 className="fourth-heading">परीक्षण कमांड</h4>
<div className="underline-class"></div>
```bash
ros2 launch gazebo_ros gazebo
ros2 run tf2_tools view_frames
```

<h4 className="fourth-heading">संकेत</h4>
<div className="underline-class"></div>

- • वास्तविक जड़त्व मान का उपयोग करें
- • प्रत्येक विशेषता की पुष्टि करें

</details>

<div className="border-line"></div>

<h2 className="second-heading">समस्या निवारण</h2>
<div className="underline-class"></div>

<details>
<summary>सामान्य समस्याएं</summary>

<h3 className="third-heading">समस्या निवारण</h3>
<div className="underline-class"></div>

<h4 className="fourth-heading">विकृत मॉडल/गलत जॉइंट्स</h4>
<div className="underline-class"></div>

**समाधान**:
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

<h4 className="fourth-heading">रोबोट आरवीआईज़ में नहीं है</h4>
<div className="underline-class"></div>

**समाधान**:
```bash
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:=$(cat robot.urdf)
ros2 topic echo /joint_states
ros2 run tf2_tools view_frames
```

<h4 className="fourth-heading">सिमुलेशन में रोबोट गिर जाता है</h4>
<div className="underline-class"></div>

**समाधान**:
```xml
<inertial>
  <mass value="1.0"/>
  <origin xyz="0 0 0"/>
  <inertia ixx="0.01" iyy="0.01" izz="0.01"/>
</inertial>
```

<h4 className="fourth-heading">एक्सैक्रो संकलन त्रुटियां</h4>
<div className="underline-class"></div>

**समाधान**:
```xml
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="robot">
<xacro:macro name="macro" params="param1 param2:=default">...</xacro:macro>
<xacro:property name="value" value="1.0"/>
```

</details>

<div className="border-line"></div>

<h2 className="second-heading">यूआरडीएफ मूलभूत</h2>
<div className="underline-class"></div>

<h3 className="third-heading">मुख्य घटक</h3>
<div className="underline-class"></div>

- • **लिंक्स**: दृढ़ निकाय
- • **जॉइंट्स**: डीओएफ के साथ कनेक्शन
- • **सामग्री**: दृश्य गुण
- • **जड़त्व**: द्रव्यमान, सीओएम, जड़त्व
- • **टकराव**: सरलीकृत ज्यामिति
- • **दृश्य**: विस्तृत ज्यामिति

<h3 className="third-heading">मूल संरचना</h3>
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

<h2 className="second-heading">मानवरूपी संरचना</h2>
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

<h3 className="third-heading">जॉइंट प्रकार</h3>
<div className="underline-class"></div>

- • **रेवोल्यूट**: एकल-अक्ष घूर्णन
- • **निरंतर**: असीमित घूर्णन
- • **प्रिज़्मैटिक**: रेखीय गति
- • **स्थिर**: दृढ़ कनेक्शन

<div className="border-line"></div>

<h2 className="second-heading">पूर्ण मानवरूपी उदाहरण</h2>
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

<h2 className="second-heading">एक्सैक्रो मैक्रो</h2>
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

<h2 className="second-heading">उन्नत विशेषताएं</h2>
<div className="underline-class"></div>

<h3 className="third-heading">संचरण</h3>
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

<h3 className="third-heading">गेज़बो तत्व</h3>
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

<h2 className="second-heading">मान्यता और दृश्यकरण</h2>
<div className="underline-class"></div>
```bash
check_urdf robot.urdf
urdf_to_graphiz robot.urdf
ros2 run rviz2 rviz2
```

<h3 className="third-heading">रोबोट स्टेट पब्लिशर</h3>
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

<h2 className="second-heading">सर्वोत्तम प्रथाएं</h2>
<div className="underline-class"></div>

- • मानव जैसे जॉइंट सीमा का पालन करें
- • उचित द्रव्यमान वितरण
- • सरलीकृत टकराव ज्यामिति
- • जटिल मॉडल के लिए एक्सैक्रो का उपयोग करें
- • जॉइंट उद्देश्य दस्तावेज़ करें
- • तार्किक रूप से व्यवस्थित करें

<h2 className="second-heading">सारांश</h2>
<div className="underline-class"></div>

यूआरडीएफ लिंक्स, जॉइंट्स और गुणों के साथ रोबोट संरचना को परिभाषित करता है। बनाए रखने योग्यता के लिए एक्सैक्रो का उपयोग करें और उपयोग से पहले मॉडल की पुष्टि करें।