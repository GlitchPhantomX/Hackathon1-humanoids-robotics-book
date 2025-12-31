---
sidebar_position: 2
title: "नोड्स और टॉपिक्स: पब्लिश-सब्सक्राइब पैटर्न"
id: "01-ros2-nodes-topics"
---

import ReadingTime from '@site/src/components/ReadingTime';

<ReadingTime minutes={6} />

<h1 className="main-heading">नोड्स और टॉपिक्स: पब्लिश-सब्सक्राइब पैटर्न</h1>
<div className="underline-class"></div>

**सीखने के उद्देश्य**:
- • पायथन और सी++ में आरओएस 2 नोड्स बनाएं
- • क्यूओएस के साथ प्रकाशक और सदस्य लागू करें
- • टॉपिक-आधारित संचार पैटर्न डिज़ाइन करें
- • संदेश सीरियलाइज़ेशन संभालें
- • संचार समस्याओं का डिबग करें

**पूर्वापेक्षाएं**: आरओएस 2 संरचना, पायथन/सी++ | **समय**: 3-4 घंटे

<div className="border-line"></div>

<h2 className="second-heading">परिचय</h2>
<div className="underline-class"></div>

नोड्स और टॉपिक्स आरओएस 2 की संचार नींव बनाते हैं। पब्लिश-सब्सक्राइब पैटर्न अतुल्यकालिक, अलग किए गए संचार को सक्षम बनाता है जो रोबोटिक्स के लिए आदर्श है।

<div className="border-line"></div>

<h2 className="second-heading">नोड्स</h2>
<div className="underline-class"></div>

आरओएस 2 प्रणालियों में गणना करने वाले प्रक्रियाएं।

<h3 className="third-heading">नोड संरचना</h3>
<div className="underline-class"></div>

- • **आरंभीकरण**: नाम/नामस्थान के साथ सेटअप
- • **एंटिटी सृजना**: प्रकाशक, सदस्य, सेवाएं
- • **प्रसंस्करण लूप**: घटना-संचालित या टाइमर-आधारित
- • **साफ-सफाई**: उचित बंदी

<h3 className="third-heading">पायथन नोड</h3>
<div className="underline-class"></div>
```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.get_logger().info('Node initialized')

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

<h3 className="third-heading">सी++ नोड</h3>
<div className="underline-class"></div>
```cpp
#include "rclcpp/rclcpp.hpp"

class MyNode : public rclcpp::Node {
public:
    MyNode() : Node("my_node") {
        RCLCPP_INFO(this->get_logger(), "Node initialized");
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MyNode>());
    rclcpp::shutdown();
    return 0;
}
```

<div className="border-line"></div>

<h2 className="second-heading">टॉपिक्स</h2>
<div className="underline-class"></div>

अतुल्यकालिक पब्लिश-सब्सक्राइब संचार।

<h3 className="third-heading">प्रकाशक</h3>
<div className="underline-class"></div>

<h4 className="fourth-heading">पायथन</h4>
<div className="underline-class"></div>
```python
from std_msgs.msg import String

class Publisher(Node):
    def __init__(self):
        super().__init__('publisher')
        self.pub = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(0.5, self.publish)

    def publish(self):
        msg = String()
        msg.data = 'Hello'
        self.pub.publish(msg)
```

<h4 className="fourth-heading">सी++</h4>
<div className="underline-class"></div>
```cpp
class Publisher : public rclcpp::Node {
public:
    Publisher() : Node("publisher") {
        pub_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        timer_ = this->create_wall_timer(500ms,
            std::bind(&Publisher::publish, this));
    }
private:
    void publish() {
        auto msg = std_msgs::msg::String();
        msg.data = "Hello";
        pub_->publish(msg);
    }
};
```

<h3 className="third-heading">सदस्य</h3>
<div className="underline-class"></div>

<h4 className="fourth-heading">पायथन</h4>
<div className="underline-class"></div>
```python
class Subscriber(Node):
    def __init__(self):
        super().__init__('subscriber')
        self.sub = self.create_subscription(String, 'topic', self.callback, 10)

    def callback(self, msg):
        self.get_logger().info(f'Heard: {msg.data}')
```

<h4 className="fourth-heading">सी++</h4>
<div className="underline-class"></div>
```cpp
class Subscriber : public rclcpp::Node {
public:
    Subscriber() : Node("subscriber") {
        sub_ = this->create_subscription<std_msgs::msg::String>(
            "topic", 10, [this](auto msg) {
                RCLCPP_INFO(this->get_logger(), "Heard: %s", msg->data.c_str());
            });
    }
};
```

<div className="border-line"></div>

<h2 className="second-heading">क्यूओएस सेटिंग्स</h2>
<div className="underline-class"></div>
```python
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

qos = QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE
)
pub = self.create_publisher(String, 'topic', qos)
```

<h3 className="third-heading">नामस्थान</h3>
<div className="underline-class"></div>
```python
pub = self.create_publisher(String, 'robot1/sensors', 10)
```

<h3 className="third-heading">संदेश प्रकार</h3>
<div className="underline-class"></div>

- • **std_msgs**: मूलभूत प्रकार (स्ट्रिंग, इंट32)
- • **geometry_msgs**: बिंदु, स्थिति, ट्विस्ट
- • **sensor_msgs**: लेजरस्कैन, छवि
- • **nav_msgs**: ओडोमेट्री, पथ

<h4 className="fourth-heading">कस्टम संदेश</h4>
<div className="underline-class"></div>
```
# MyMessage.msg
string name
int32 value
float64[] values
```

<div className="border-line"></div>

<h2 className="second-heading">अभ्यास</h2>
<div className="underline-class"></div>

**अभ्यास 1.2.1**: बहु-नोड नेटवर्क (⭐⭐, 30-40 मिनट)
- सेंसर प्रकाशक बनाएं
- विभिन्न प्रसंस्करण के साथ कई सदस्य
- उचित क्यूओएस विन्यास

**अभ्यास 1.2.2**: कस्टम संदेश (⭐⭐⭐, 45-60 मिनट)
- कस्टम मानवरूपी संदेश परिभाषित करें
- प्रकाशक/सदस्य लागू करें
- जटिल डेटा संरचनाओं को संभालें

**अभ्यास 1.2.3**: प्रदर्शन अनुकूलन (⭐⭐, 35-45 मिनट)
- उच्च/निम्न आवृत्ति प्रकाशक
- उपयुक्त क्यूओएस कॉन्फ़िगर करें
- प्रदर्शन की निगरानी और अनुकूलन करें

<div className="border-line"></div>

<h2 className="second-heading">सामान्य समस्याएं</h2>
<div className="underline-class"></div>

**टॉपिक खोज**:
```bash
echo $ROS_DOMAIN_ID
ros2 topic list
ros2 topic info /topic_name
```

**क्यूओएस असंगतता**:
```python
# प्रकाशक/सदस्य के बीच क्यूओएस मिलाएं
qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
```

**प्रदर्शन**:
```python
# उच्च-आवृत्ति का अनुकूलन करें
qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
```

**संसाधन प्रबंधन**:
```python
def destroy_node(self):
    if self.timer:
        self.timer.cancel()
    super().destroy_node()
```

**सीरियलाइज़ेशन**:
```bash
ros2 interface show std_msgs/msg/String
ros2 topic type /topic_name
```

<div className="border-line"></div>

<h2 className="second-heading">सारांश</h2>
<div className="underline-class"></div>

नोड्स गणना प्रक्रियाएं हैं। टॉपिक्स अतुल्यकालिक पब्लिश-सब्सक्राइब संचार को सक्षम बनाते हैं। क्यूओएस सेटिंग्स वितरण व्यवहार कॉन्फ़िगर करते हैं। कस्टम संदेश जटिल डेटा को संभालते हैं।

**मुख्य बातें**:
- • पब्लिश-सब्सक्राइब ढीला युग्मन सक्षम बनाता है
- • क्यूओएस सेटिंग्स संचार को ट्यून करते हैं
- • उचित जीवनचक्र प्रबंधन आवश्यक है
- • जटिल डेटा के लिए कस्टम संदेश
- • आवृत्ति, विश्वसनीयता, संसाधनों को संतुलित करें

<h2 className="second-heading">संसाधन</h2>
<div className="underline-class"></div>

- • [आरओएस 2 नोड्स/टॉपिक्स](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
- • [क्यूओएस गाइड](https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service.html)

**नेविगेशन**: [← पिछला](./01-architecture.md) | [अगला →](./03-services-actions.md)