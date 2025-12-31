---
sidebar_position: 1
title: "आरओएस 2 संरचना और अवधारणाएं"
id: "01-ros2-architecture"
---

import ReadingTime from '@site/src/components/ReadingTime';

<ReadingTime minutes={5} />

<h1 className="main-heading">आरओएस 2 संरचना और अवधारणाएं</h1>
<div className="underline-class"></div>

**सीखने के उद्देश्य**:
- • आरओएस 2 मुख्य संरचना का वर्णन करें
- • डीडीएस-आधारित संचार मॉडल की व्याख्या करें
- • क्यूओएस प्रोफाइल को समझें
- • नोड्स, टॉपिक्स, सेवाएं, क्रियाएं की पहचान करें
- • आरओएस 2 वातावरण सेट करें

**पूर्वापेक्षाएं**: रोबोटिक्स मूलभूत, पायथन | **समय**: 2-3 घंटे

<div className="border-line"></div>

<h2 className="second-heading">परिचय</h2>
<div className="underline-class"></div>

आरओएस 2 मानवरूपी रोबोट संचार के लिए मिडलवेयर बुनियाद प्रदान करता है। डीडीएस पर निर्मित, यह बेहतर विश्वसनीयता, वास्तविक समय समर्थन और उत्पादन तैनाती क्षमताएं प्रदान करता है।

<div className="border-line"></div>

<h2 className="second-heading">आरओएस 1 बनाम आरओएस 2</h2>
<div className="underline-class"></div>

**मुख्य सुधार**:
- • **वास्तविक समय समर्थन**: महत्वपूर्ण अनुप्रयोगों के लिए बेहतर
- • **बहु-रोबोट प्रणालियां**: बेहतर समन्वय
- • **उत्पादन तैनाती**: सुरक्षा और स्थिरता
- • **मिडलवेयर लचीलापन**: डीडीएस कार्यान्वयन

**मुख्य अंतर**:
- • **संचार**: डीडीएस-आधारित मिडलवेयर
- • **क्यूओएस**: विन्यास योग्य विश्वसनीयता
- • **सुरक्षा**: अंतर्निहित सुविधाएं
- • **जीवनचक्र**: दृढ़ प्रबंधन

<div className="border-line"></div>

<h2 className="second-heading">मुख्य घटक</h2>
<div className="underline-class"></div>

<h3 className="third-heading">नोड्स</h3>
<div className="underline-class"></div>

- • अलग प्रक्रियाएं
- • इंटर-नोड संचार
- • संलग्न कार्यक्षमता
- • बहु-भाषा समर्थन
```python
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')
        self.get_logger().info('Hello from node!')
```

<h3 className="third-heading">डीडीएस मिडलवेयर</h3>
<div className="underline-class"></div>

- • डेटा-केंद्रित संचार
- • प्रकाशक-सदस्य मॉडल
- • स्वचालित खोज
- • विन्यास योग्य क्यूओएस

<h3 className="third-heading">टॉपिक्स</h3>
<div className="underline-class"></div>

- • अतुल्यकालिक पब/सब
- • एक-से-कई संचार
- • संदेश प्रकार (.msg फाइलें)
- • वास्तविक समय सक्षम

<h3 className="third-heading">सेवाएं</h3>
<div className="underline-class"></div>

- • समतुल्यकालिक अनुरोध-प्रतिक्रिया
- • क्लाइंट-सर्वर पैटर्न
- • सेवा प्रकार (.srv फाइलें)
- • ब्लॉकिंग कॉल

<h3 className="third-heading">क्रियाएं</h3>
<div className="underline-class"></div>

- • लक्ष्य-प्रतिक्रिया-परिणाम पैटर्न
- • लंबे समय तक चलने वाले कार्य
- • रद्दीकरण समर्थन
- • क्रिया प्रकार (.action फाइलें)

<div className="border-line"></div>

<h2 className="second-heading">क्यूओएस प्रोफाइल</h2>
<div className="underline-class"></div>

**विश्वसनीयता**: विश्वसनीय (टीसीपी-जैसा) बनाम सर्वोत्तम प्रयास (यूडीपी-जैसा)
**स्थायित्व**: स्थानीय स्थायी बनाम अस्थायी
**इतिहास**: अंतिम रखें (एन संदेश) बनाम सभी रखें
```python
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

qos = QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE
)
```

<div className="border-line"></div>

<h2 className="second-heading">क्लाइंट पुस्तकालय</h2>
<div className="underline-class"></div>

**rclcpp (C++)**: उच्च-प्रदर्शन, वास्तविक समय प्रणालियां
**rclpy (Python)**: प्रोटोटाइपिंग, त्वरित विकास

<div className="border-line"></div>

<h2 className="second-heading">अभ्यास</h2>
<div className="underline-class"></div>

**अभ्यास 1.1.1**: वातावरण सेटअप (⭐⭐, 25-35 मिनट)
- आरओएस 2 स्थापना की पुष्टि करें
- कार्यक्षेत्र और पैकेज बनाएं
- प्रकाशक नोड लागू करें
- परीक्षण और सत्यापित करें

**अभ्यास 1.1.2**: क्यूओएस विन्यास (⭐⭐⭐, 35-45 मिनट)
- विभिन्न क्यूओएस के साथ प्रकाशक बनाएं
- विश्वसनीय बनाम सर्वोत्तम प्रयास का परीक्षण करें
- प्रदर्शन की तुलना करें
- उपयोग मामलों का विश्लेषण करें

**अभ्यास 1.1.3**: संचार विश्लेषण (⭐⭐, 30-40 मिनट)
- कई नोड्स बनाएं
- आरओएस 2 उपकरणों के साथ टोपोलॉजी का विश्लेषण करें
- प्रवाह दस्तावेज़ करें
- दृश्यकरण बनाएं

<div className="border-line"></div>

<h2 className="second-heading">सामान्य समस्याएं</h2>
<div className="underline-class"></div>

**नोड खोज**:
```bash
echo $ROS_DOMAIN_ID
export ROS_DOMAIN_ID=0
ros2 node list
```

**क्यूओएस असंगति**:
```python
qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
# प्रकाशक और सदस्य पर मिलान करें
```

**कार्यक्षेत्र सेटअप**:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
source install/setup.bash
```

**प्रदर्शन**:
```python
qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
```

**जीवनचक्र**:
```python
def destroy_node(self):
    if self.timer:
        self.timer.cancel()
    super().destroy_node()
```

<div className="border-line"></div>

<h2 className="second-heading">सारांश</h2>
<div className="underline-class"></div>

आरओएस 2 विन्यास योग्य क्यूओएस प्रोफाइल के साथ डीडीएस मिडलवेयर का उपयोग करता है। मुख्य घटकों में नोड्स (निष्पादन इकाइयां), टॉपिक्स (पब/सब), सेवाएं (अनुरोध/प्रतिक्रिया), और क्रियाएं (लक्ष्य/प्रतिक्रिया/परिणाम) शामिल हैं।

**मुख्य बातें**:
- • डीडीएस बेहतर विश्वसनीयता प्रदान करता है
- • क्यूओएस ठीक-समायोजित संचार को सक्षम बनाता है
- • वास्तविक समय और बहु-रोबोट प्रणालियों का समर्थन करता है
- • उचित सेटअप आवश्यक है
- • पैटर्न को समझना महत्वपूर्ण है

<h2 className="second-heading">संसाधन</h2>
<div className="underline-class"></div>

- • [आरओएस 2 संरचना](https://docs.ros.org/en/humble/Concepts/About-ROS-2-Architecture.html)
- • [क्यूओएस गाइड](https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service.html)
- • [आरओएस 2 डेमो](https://github.com/ros2/demos)

**नेविगेशन**: [← पिछला](../00-introduction/05-syllabus.md) | [अगला →](./02-nodes-topics.md)

<h2 className="second-heading">त्वरित संदर्भ</h2>
<div className="underline-class"></div>

| घटक | उद्देश्य | सर्वोत्तम अभ्यास |
|-----------|---------|---------------|
| नोड्स | निष्पादन इकाइयां | एकल जिम्मेदारी |
| टॉपिक्स | अतुल्यकालिक संचार | उचित क्यूओएस का उपयोग करें |
| सेवाएं | समतुल्यकालिक संचार | अनुरोध/प्रतिक्रिया |
| क्रियाएं | लंबे कार्य | प्रतिक्रिया/प्रगति |
| क्यूओएस | विन्यास | प्रकाशक/सदस्य प्रोफाइल मिलाएं |