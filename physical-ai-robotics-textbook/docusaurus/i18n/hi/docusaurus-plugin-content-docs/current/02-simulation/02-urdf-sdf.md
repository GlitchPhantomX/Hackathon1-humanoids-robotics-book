---
sidebar_position: 3
title: 'यूआरडीएफ और एसडीएफ: रोबोट और वातावरण मॉडलिंग'
description: 'गेज़बो सिमुलेशन में रोबोट विवरण के लिए यूआरडीएफ और वातावरण मॉडलिंग के लिए एसडीएफ की समझ'
---



<h1 className="main-heading">यूआरडीएफ और एसडीएफ: रोबोट और वातावरण मॉडलिंग</h1>
<div className="underline-class"></div>

गेज़बो सिमुलेशन में रोबोट मॉडलिंग के लिए यूआरडीएफ और वातावरण मॉडलिंग के लिए एसडीएफ को समझें।

<div className="border-line"></div>

<h2 className="second-heading">सीखने के उद्देश्य</h2>
<div className="underline-class"></div>

- • यूआरडीएफ और एसडीएफ अंतर को समझें
- • एसडीएफ वातावरण मॉडल बनाएं
- • यूआरडीएफ रोबोट को एसडीएफ के साथ एकीकृत करें
- • गेज़बो एक्सटेंशन का उपयोग करें
- • जटिल सिमुलेशन परिदृश्य बनाएं

<div className="border-line"></div>

<h2 className="second-heading">अभ्यास</h2>
<div className="underline-class"></div>

<details>
<summary>अभ्यास 2.2.1: मूल एसडीएफ वातावरण (⭐, ~30 मिनट)</summary>

<h3 className="third-heading">अभ्यास 2.2.1: मूल एसडीएफ वातावरण</h3>
<div className="underline-class"></div>

**कठिनाई**: ⭐ | **समय**: 30 मिनट | **आवश्यकताएँ**: गेज़बो, एक्सएमएल

<h4 className="fourth-heading">स्टार्टर कोड</h4>
<div className="underline-class"></div>

- • भौतिकी कॉन्फ़िगरेशन
- • जमीनी सतह और प्रकाश
- • सरल बॉक्स मॉडल

<h4 className="fourth-heading">सफलता मानदंड</h4>
<div className="underline-class"></div>

- [ ] एसडीएफ फ़ाइल मान्य है
- [ ] गेज़बो में दुनिया लोड होती है
- [ ] भौतिकी ठीक से काम करती है

<h4 className="fourth-heading">परीक्षण कमांड</h4>
<div className="underline-class"></div>
```bash
gz sdf -k your_world.sdf
gazebo your_world.sdf
```

<h4 className="fourth-heading">संकेत</h4>
<div className="underline-class"></div>

- • टेम्पलेट के रूप में डिफ़ॉल्ट दुनिया का उपयोग करें
- • पहले सिंटैक्स की पुष्टि करें

</details>

<details>
<summary>अभ्यास 2.2.2: यूआरडीएफ-एसडीएफ एकीकरण (⭐⭐, ~45 मिनट)</summary>

<h3 className="third-heading">अभ्यास 2.2.2: यूआरडीएफ-एसडीएफ एकीकरण</h3>
<div className="underline-class"></div>

**कठिनाई**: ⭐⭐ | **समय**: 45 मिनट

<h4 className="fourth-heading">स्टार्टर कोड</h4>
<div className="underline-class"></div>

- • यूआरडीएफ रोबोट मॉडल
- • एसडीएफ वर्ल्ड फ़ाइल
- • गेज़बो आरओएस 2 प्लगइन्स
- • जॉइंट ट्रांसमिशन

<h4 className="fourth-heading">परीक्षण कमांड</h4>
<div className="underline-class"></div>
```bash
gz sdf -p robot.urdf > robot.sdf
ros2 launch your_robot_gazebo your_simulation.launch.py
ros2 topic list | grep joint
```

<h4 className="fourth-heading">संकेत</h4>
<div className="underline-class"></div>

- • उचित गेज़बो प्लगइन्स का उपयोग करें
- • कंट्रोलर्स के साथ जॉइंट नाम मिलाएं

</details>

<details>
<summary>अभ्यास 2.2.3: उन्नत वातावरण (⭐⭐⭐, ~60 मिनट)</summary>

<h3 className="third-heading">अभ्यास 2.2.3: उन्नत वातावरण</h3>
<div className="underline-class"></div>

**कठिनाई**: ⭐⭐⭐ | **समय**: 60 मिनट

<h4 className="fourth-heading">स्टार्टर कोड</h4>
<div className="underline-class"></div>

- • कई कमरे/मंजिलें
- • फर्नीचर और बाधाएँ
- • अनुकूलित भौतिकी पैरामीटर
- • गेज़बो सेंसर एक्सटेंशन

<h4 className="fourth-heading">परीक्षण कमांड</h4>
<div className="underline-class"></div>
```bash
gz sdf -k complex_environment.sdf
gazebo --verbose complex_environment.sdf
gz topic -e /stats
```

<h4 className="fourth-heading">संकेत</h4>
<div className="underline-class"></div>

- • वातावरण के लिए स्थिर मॉडल का उपयोग करें
- • संघर्ष ज्यामिति को अनुकूलित करें

</details>

<div className="border-line"></div>

<h2 className="second-heading">समस्या निवारण</h2>
<div className="underline-class"></div>

<details>
<summary>सामान्य समस्याएँ</summary>

<h3 className="third-heading">समस्या निवारण</h3>
<div className="underline-class"></div>

<h4 className="fourth-heading">यूआरडीएफ रूपांतरण विफल</h4>
<div className="underline-class"></div>

**समाधान**:
```bash
find /path -name "*.dae" -o -name "*.stl"
check_urdf robot.urdf
gz sdf -p robot.urdf > robot.sdf
```

<h4 className="fourth-heading">भौतिकी अस्थिरता</h4>
<div className="underline-class"></div>

**समाधान**:
```xml
<inertial><mass value="1.0"/><inertia ixx="0.01" iyy="0.01" izz="0.01"/></inertial>
<physics><max_step_size>0.001</max_step_size></physics>
<joint><axis><dynamics><damping>1.0</damping></dynamics></axis></joint>
```

<h4 className="fourth-heading">आरओएस 2 एकीकरण विफल</h4>
<div className="underline-class"></div>

**समाधान**:
```xml
<plugin name="joint_state_publisher" filename="libgazebo_ros_joint_state_publisher.so">
  <ros><namespace>/robot_name</namespace></ros>
</plugin>
```
```bash
sudo apt install ros-humble-gazebo-ros-pkgs
```

<h4 className="fourth-heading">धीमा प्रदर्शन</h4>
<div className="underline-class"></div>

**समाधान**:
```xml
<collision><geometry><box><size>0.5 0.5 0.5</size></box></geometry></collision>
<sensor><update_rate>30</update_rate></sensor>
<physics><max_step_size>0.01</max_step_size></physics>
```

<h4 className="fourth-heading">जॉइंट सीमा समस्याएँ</h4>
<div className="underline-class"></div>

**समाधान**:
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

<h2 className="second-heading">यूआरडीएफ बनाम एसडीएफ</h2>
<div className="underline-class"></div>

<h3 className="third-heading">यूआरडीएफ</h3>
<div className="underline-class"></div>

**मजबूत पक्ष**:
- • रोबोट किनेमेटिक चेन
- • जॉइंट सीमाएँ और प्रकार
- • आरओएस एकीकरण
- • एक्सकैरो के साथ पैरामीटराइज़ेशन

<h3 className="third-heading">एसडीएफ</h3>
<div className="underline-class"></div>

**मजबूत पक्ष**:
- • पूर्ण सिमुलेशन वातावरण
- • भौतिकी गुण
- • गेज़बो प्लगइन्स और सेंसर
- • कई मॉडल समावेश

<div className="border-line"></div>

<h2 className="second-heading">यूआरडीएफ को एसडीएफ में रूपांतरित करना</h2>
<div className="underline-class"></div>

<h3 className="third-heading">गेज़बो में सीधे यूआरडीएफ</h3>
<div className="underline-class"></div>
```xml
<world name="default">
  <include><uri>model://ground_plane</uri></include>
  <include><uri>file://$(find robot)/urdf/humanoid.urdf</uri></include>
</world>
```

<h3 className="third-heading">यूआरडीएफ को लपेटना</h3>
<div className="underline-class"></div>
```xml
<model name="humanoid_robot">
  <include><uri>model://humanoid_robot_model</uri></include>
  <plugin filename="libgazebo_ros_control.so"/>
</model>
```

<div className="border-line"></div>

<h2 className="second-heading">एसडीएफ वातावरण</h2>
<div className="underline-class"></div>

<h3 className="third-heading">वर्ल्ड संरचना</h3>
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

<h3 className="third-heading">कस्टम मॉडल</h3>
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

<h2 className="second-heading">उन्नत सुविधाएँ</h2>
<div className="underline-class"></div>

<h3 className="third-heading">जॉइंट ट्रांसमिशन</h3>
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

<h3 className="third-heading">गेज़बो एक्सटेंशन</h3>
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

<h2 className="second-heading">मानवरूपी के लिए भौतिकी</h2>
<div className="underline-class"></div>

<h3 className="third-heading">भौतिकी पैरामीटर</h3>
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

<h3 className="third-heading">जॉइंट डैम्पिंग</h3>
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

<h2 className="second-heading">सर्वोत्तम प्रथाएँ</h2>
<div className="underline-class"></div>

- • वर्णनात्मक नामों का उपयोग करें
- • संघर्ष ज्यामिति को सरल बनाएं
- • वातावरण के लिए स्थिर मॉडल का उपयोग करें
- • यूआरडीएफ-एसडीएफ रूपांतरण का परीक्षण करें
- • जटिल मॉडल को दस्तावेज़ीकृत करें

<h2 className="second-heading">आरओएस 2 लॉन्च एकीकरण</h2>
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

<h2 className="second-heading">सारांश</h2>
<div className="underline-class"></div>

यूआरडीएफ रोबोट का वर्णन करता है, एसडीएफ पूर्ण सिमुलेशन वातावरण का वर्णन करता है। वास्तविक मानवरूपी रोबोट सिमुलेशन के लिए दोनों प्रारूपों को जोड़ने के लिए गेज़बो एक्सटेंशन का उपयोग करें।