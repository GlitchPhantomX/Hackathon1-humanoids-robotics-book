---
sidebar_position: 2
title: 'गेज़बो परिचय: रोबोटिक्स के लिए सिमुलेशन'
description: 'रोबोटिक्स विकास के लिए गेज़बो सिमुलेशन वातावरण के साथ शुरूआत'
---

import ReadingTime from '@site/src/components/ReadingTime';

<ReadingTime minutes={4} />

<h1 className="main-heading">गेज़बो परिचय: रोबोटिक्स के लिए सिमुलेशन</h1>
<div className="underline-class"></div>

गेज़बो रोबोटिक्स के लिए एक शक्तिशाली 3डी सिमुलेशन वातावरण है जिसमें वास्तविक भौतिकी, ग्राफिक्स और आरओएस 2 एकीकरण है।

<div className="border-line"></div>

<h2 className="second-heading">सीखने के उद्देश्य</h2>
<div className="underline-class"></div>

- • गेज़बो संरचना को समझें
- • गेज़बो स्थापित और कॉन्फ़िगर करें
- • सिमुलेशन दुनिया बनाएं
- • आरओएस 2 के साथ एकीकरण करें
- • सिमुलेटेड रोबोट को नियंत्रित करें

<div className="border-line"></div>

<h2 className="second-heading">अभ्यास</h2>
<div className="underline-class"></div>

<details>
<summary>अभ्यास 2.1.1: स्थापना और मूल सिमुलेशन (⭐, ~30 मिनट)</summary>

<h3 className="third-heading">अभ्यास 2.1.1: स्थापना और मूल सिमुलेशन</h3>
<div className="underline-class"></div>

**कठिनाई**: ⭐ | **समय**: 30 मिनट | **आवश्यकताएं**: उबंटू, आरओएस 2

<h4 className="fourth-heading">स्टार्टर कोड</h4>
<div className="underline-class"></div>

- • गेज़बो गार्डन स्थापित करें
- • डिफ़ॉल्ट दुनिया लॉन्च करें
- • इंटरफेस का अन्वेषण करें

<h4 className="fourth-heading">सफलता मानदंड</h4>
<div className="underline-class"></div>

- [ ] गेज़बो सफलतापूर्वक लॉन्च होता है
- [ ] कैमरा नियंत्रण काम करता है
- [ ] सिमुलेशन सुचारू रूप से चलता है

<h4 className="fourth-heading">परीक्षण कमांड</h4>
<div className="underline-class"></div>
```bash
sudo apt update
sudo apt install gazebo-garden ros-humble-gazebo-ros-pkgs
gazebo
```

<h4 className="fourth-heading">संकेत</h4>
<div className="underline-class"></div>

- • ग्राफिक्स ड्राइवर्स की जांच करें
- • सिस्टम आवश्यकताओं की पुष्टि करें

</details>

<details>
<summary>अभ्यास 2.1.2: कस्टम दुनिया निर्माण (⭐⭐, ~45 मिनट)</summary>

<h3 className="third-heading">अभ्यास 2.1.2: कस्टम दुनिया निर्माण</h3>
<div className="underline-class"></div>

**कठिनाई**: ⭐⭐ | **समय**: 45 मिनट

<h4 className="fourth-heading">स्टार्टर कोड</h4>
<div className="underline-class"></div>

- • जमीनी समतल और प्रकाश
- • कई वस्तुएं
- • रोबोट मॉडल
- • भौतिकी पैरामीटर

<h4 className="fourth-heading">परीक्षण कमांड</h4>
<div className="underline-class"></div>
```bash
gz sdf -k your_world.sdf
gazebo your_world.sdf
```

<h4 className="fourth-heading">संकेत</h4>
<div className="underline-class"></div>

- • डिफ़ॉल्ट टेम्पलेट से शुरू करें
- • परीक्षण से पहले एसडीएफ की पुष्टि करें

</details>

<details>
<summary>अभ्यास 2.1.3: आरओएस 2 एकीकरण (⭐⭐⭐, ~60 मिनट)</summary>

<h3 className="third-heading">अभ्यास 2.1.3: आरओएस 2 एकीकरण</h3>
<div className="underline-class"></div>

**कठिनाई**: ⭐⭐⭐ | **समय**: 60 मिनट

<h4 className="fourth-heading">स्टार्टर कोड</h4>
<div className="underline-class"></div>

- • यूआरडीएफ रोबोट मॉडल
- • लॉन्च फाइल
- • रोबोट/जॉइंट स्टेट पब्लिशर
- • कंट्रोलर नोड

<h4 className="fourth-heading">परीक्षण कमांड</h4>
<div className="underline-class"></div>
```bash
ros2 launch your_robot_gazebo your_robot.launch.py
ros2 topic list
```

<h4 className="fourth-heading">संकेत</h4>
<div className="underline-class"></div>

- • रोबोट_स्टेट_पब्लिशर का उपयोग करें
- • घटकों को अलग-अलग परीक्षण करें

</details>

<div className="border-line"></div>

<h2 className="second-heading">समस्या निवारण</h2>
<div className="underline-class"></div>

<details>
<summary>सामान्य समस्याएं</summary>

<h3 className="third-heading">समस्या निवारण</h3>
<div className="underline-class"></div>

<h4 className="fourth-heading">गेज़बो शुरू नहीं होता</h4>
<div className="underline-class"></div>

**समाधान**:
```bash
glxinfo | grep "OpenGL version"
sudo apt install mesa-utils
export LIBGL_ALWAYS_SOFTWARE=1
```

<h4 className="fourth-heading">रोबोट जमीन के माध्यम से गिर जाता है</h4>
<div className="underline-class"></div>

**समाधान**:
```xml
<inertial><mass value="1.0"/>
<inertia ixx="0.01" iyy="0.01" izz="0.01"/></inertial>
```

<h4 className="fourth-heading">धीमा सिमुलेशन</h4>
<div className="underline-class"></div>

**समाधान**:
```xml
<sensor><update_rate>30</update_rate></sensor>
<collision><geometry><box><size>0.5 0.5 0.5</size></box></geometry></collision>
<physics><max_step_size>0.01</max_step_size></physics>
```

<h4 className="fourth-heading">आरओएस 2 एकीकरण विफल</h4>
<div className="underline-class"></div>

**समाधान**:
```xml
<plugin name="joint_state_publisher" filename="libgazebo_ros_joint_state_publisher.so"/>
```
```bash
sudo apt install ros-humble-gazebo-ros-pkgs
```

<h4 className="fourth-heading">मॉडल/टेक्सचर गायब</h4>
<div className="underline-class"></div>

**समाधान**:
```bash
echo $GAZEBO_MODEL_PATH
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/path/to/models
gz sdf -k model.sdf
```

</details>

<div className="border-line"></div>

<h2 className="second-heading">गेज़बो क्या है?</h2>
<div className="underline-class"></div>

रोबोट के लिए 3डी गतिशील सिमुलेटर जिसमें हैं:
- • **वास्तविक भौतिकी**: ओडीई, बुलेट, डार्ट
- • **ग्राफिक्स**: ओजीआरई रेंडरिंग
- • **सेंसर**: कैमरा, लाइडार, आईएमयू, जीपीएस
- • **प्लगइन्स**: विस्तार्य संरचना
- • **आरओएस एकीकरण**: बेजोड़ आरओएस 2 समर्थन

<h3 className="third-heading">मुख्य विशेषताएं</h3>
<div className="underline-class"></div>

- • जटिल काइनेमेटिक्स समर्थन
- • वास्तविक भौतिकी और संतुलन
- • सेंसर सिमुलेशन
- • उन्नत टकराव जांच
- • जटिल वातावरण

<div className="border-line"></div>

<h2 className="second-heading">स्थापना</h2>
<div className="underline-class"></div>
```bash
sudo apt update
sudo apt install gazebo-garden ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control
```

<div className="border-line"></div>

<h2 className="second-heading">मूल अवधारणाएं</h2>
<div className="underline-class"></div>

<h3 className="third-heading">दुनिया फाइलें</h3>
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

<h3 className="third-heading">मॉडल फाइलें</h3>
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

<h2 className="second-heading">गेज़बो चलाना</h2>
<div className="underline-class"></div>
```bash
gazebo
gazebo /path/to/world.world
ros2 launch gazebo_ros gazebo.launch.py
ros2 launch gazebo_ros gazebo.launch.py world:=/path/to/world.sdf
```

<h3 className="third-heading">जीयूआई नियंत्रण</h3>
<div className="underline-class"></div>

- • **ऑर्बिट**: दाएं-क्लिक + खींचें
- • **पैन**: शिफ्ट + दाएं-क्लिक + खींचें
- • **जूम**: स्क्रॉल व्हील
- • **फ़ोकस**: ऑब्जेक्ट पर डबल-क्लिक

<div className="border-line"></div>

<h2 className="second-heading">आरओएस 2 एकीकरण</h2>
<div className="underline-class"></div>

<h3 className="third-heading">रोबोट स्पॉन करना</h3>
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

<h3 className="third-heading">गेज़बो प्लगइन्स</h3>
<div className="underline-class"></div>
```xml
<!-- जॉइंट नियंत्रण -->
<plugin name="joint_state_publisher" filename="libgazebo_ros_joint_state_publisher.so">
  <update_rate>30</update_rate>
</plugin>

<!-- कैमरा -->
<plugin name="camera_controller" filename="libgazebo_ros_camera.so">
  <ros><namespace>/robot</namespace></ros>
</plugin>

<!-- आईएमयू -->
<plugin name="imu_plugin" filename="libgazebo_ros_imu.so">
  <ros><remapping>~/out:=/imu/data</remapping></ros>
</plugin>
```

<div className="border-line"></div>

<h2 className="second-heading">सर्वोत्तम प्रथाएं</h2>
<div className="underline-class"></div>

<h3 className="third-heading">प्रदर्शन</h3>
<div className="underline-class"></div>

- • सेंसर अपडेट दर कम करें
- • टकराव ज्यामिति को सरल बनाएं
- • भौतिकी कदम सीमित करें
- • उपयुक्त दुनिया आकार

<h3 className="third-heading">वास्तविकता</h3>
<div className="underline-class"></div>

- • सटीक जड़त्व गुण
- • उचित जॉइंट सीमा
- • वास्तविक सेंसर शोर
- • सही घर्षण मान

<h3 className="third-heading">डिबगिंग</h3>
<div className="underline-class"></div>

- • वायरफ्रेम मोड (प्रेस 'W')
- • संपर्क दृश्यकरण सक्षम करें
- • वास्तविक समय कारक मॉनिटर करें
- • अंतर्निहित उपकरणों का उपयोग करें

<div className="border-line"></div>

<h2 className="second-heading">सारांश</h2>
<div className="underline-class"></div>

गेज़बो रोबोटिक्स विकास के लिए सटीक भौतिकी, वास्तविक सेंसर और आरओएस 2 एकीकरण के साथ शक्तिशाली सिमुलेशन प्रदान करता है जिससे हार्डवेयर तैनाती से पहले।

<h2 className="second-heading">संसाधन</h2>
<div className="underline-class"></div>

- • [गेज़बो दस्तावेज़ीकरण](http://gazebosim.org/)
- • [आरओएस 2 गेज़बो](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Gazebo.html)
- • [गेज़बो ट्यूटोरियल](http://gazebosim.org/tutorials)