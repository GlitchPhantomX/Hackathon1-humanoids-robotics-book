---
sidebar_position: 2
title: "نوڈس اور ٹاپکس: پبلش-سبسکرائیب پیٹرن"
id: "01-ros2-nodes-topics"
---

import ReadingTime from '@site/src/components/ReadingTime';


<ReadingTime minutes={6} />

<h1 className="main-heading">نوڈس اور ٹاپکس: پبلش-سبسکرائیب پیٹرن</h1>
<div className="underline-class"></div>

**سیکھنے کے اہداف**:
- • Python اور C++ میں ROS 2 نوڈس بنانا
- • QoS کے ساتھ پبلیشرز اور سبسکرائیبرز نافذ کرنا
- • ٹاپک-بیسڈ مواصلاتی پیٹرنز ڈیزائن کرنا
- • میسج سیریلائزیشن ہینڈل کرنا
- • مواصلاتی مسائل کی ڈیبگنگ کرنا

**ضروریات**: ROS 2 آرکیٹیکچر، Python/C++ | **وقت**: 3-4 گھنٹے

<div className="border-line"></div>

<h2 className="second-heading">تعارف</h2>
<div className="underline-class"></div>

نوڈس اور ٹاپکس ROS 2 کے مواصلاتی بنیاد کو تشکیل دیتے ہیں۔ پبلش-سبسکرائیب پیٹرن اسینکرونس، ڈیکوپلڈ مواصلات کو فعال کرتا ہے جو روبوٹکس کے لیے مثالی ہے۔

<div className="border-line"></div>

<h2 className="second-heading">نوڈس</h2>
<div className="underline-class"></div>

ROS 2 سسٹم میں کمپیوٹیشن انجام دینے والے عمل۔

<h3 className="third-heading">نوڈ سٹرکچر</h3>
<div className="underline-class"></div>

- • **اولین طور پر سیٹ اپ**: نام/نیم اسپیس کے ساتھ
- • **اینٹیٹی تخلیق**: پبلیشرز، سبسکرائیبرز، سروسز
- • **پروسیسنگ لوپ**: ایونٹ ڈرائیون یا ٹائمر-بیسڈ
- • **کلین اپ**: مناسب شٹ ڈاؤن

<h3 className="third-heading">Python نوڈ</h3>
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

<h3 className="third-heading">C++ نوڈ</h3>
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

<h2 className="second-heading">ٹاپکس</h2>
<div className="underline-class"></div>

اسینکرونس پبلش-سبسکرائیب مواصلات۔

<h3 className="third-heading">پبلیشر</h3>
<div className="underline-class"></div>

<h4 className="fourth-heading">Python</h4>
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

<h4 className="fourth-heading">C++</h4>
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

<h3 className="third-heading">سبسکرائیبر</h3>
<div className="underline-class"></div>

<h4 className="fourth-heading">Python</h4>
<div className="underline-class"></div>
```python
class Subscriber(Node):
    def __init__(self):
        super().__init__('subscriber')
        self.sub = self.create_subscription(String, 'topic', self.callback, 10)

    def callback(self, msg):
        self.get_logger().info(f'Heard: {msg.data}')
```

<h4 className="fourth-heading">C++</h4>
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

<h2 className="second-heading">QoS ترتیبات</h2>
<div className="underline-class"></div>
```python
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

qos = QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE
)
pub = self.create_publisher(String, 'topic', qos)
```

<h3 className="third-heading">نیم اسپیسز</h3>
<div className="underline-class"></div>
```python
pub = self.create_publisher(String, 'robot1/sensors', 10)
```

<h3 className="third-heading">میسج قسمیں</h3>
<div className="underline-class"></div>

- • **std_msgs**: بنیادی قسمیں (String, Int32)
- • **geometry_msgs**: Point, Pose, Twist
- • **sensor_msgs**: LaserScan, Image
- • **nav_msgs**: Odometry, Path

<h4 className="fourth-heading">اپنی میسج</h4>
<div className="underline-class"></div>
```
# MyMessage.msg
string name
int32 value
float64[] values
```

<div className="border-line"></div>

<h2 className="second-heading">مشقیں</h2>
<div className="underline-class"></div>

**مشق 1.2.1**: میلٹی-نوڈ نیٹ ورک (⭐⭐, 30-40 منٹ)
- سینسر پبلیشر بنائیں
- مختلف پروسیسنگ کے ساتھ متعدد سبسکرائیبرز
- مناسب QoS کنفیگریشنز

**مشق 1.2.2**: اپنی میسج (⭐⭐⭐, 45-60 منٹ)
- اپنی ہیومنوائڈ میسج تعریف کریں
- پبلیشر/سبسکرائیبر نافذ کریں
- پیچیدہ ڈیٹا سٹرکچر ہینڈل کریں

**مشق 1.2.3**: کارکردگی کی بہتری (⭐⭐, 35-45 منٹ)
- ہائی/لو فریکوئنسی پبلیشرز
- مناسب QoS کنفیگر کریں
- کارکردگی کو مانیٹر اور بہتر بنائیں

<div className="border-line"></div>

<h2 className="second-heading">عام مسائل</h2>
<div className="underline-class"></div>

**ٹاپک دریافت**:
```bash
echo $ROS_DOMAIN_ID
ros2 topic list
ros2 topic info /topic_name
```

**QoS میچ نہیں**:
```python
# pub/sub کے درمیان QoS میچ کریں
qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
```

**کارکردگی**:
```python
# ہائی فریکوئنسی کو بہتر بنائیں
qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
```

**ریسورس مینجمنٹ**:
```python
def destroy_node(self):
    if self.timer:
        self.timer.cancel()
    super().destroy_node()
```

**سیریلائزیشن**:
```bash
ros2 interface show std_msgs/msg/String
ros2 topic type /topic_name
```

<div className="border-line"></div>

<h2 className="second-heading">خلاصہ</h2>
<div className="underline-class"></div>

نوڈس کمپیوٹیشنل عمل ہیں۔ ٹاپکس اسینکرونس پبلش-سبسکرائیب مواصلات کو فعال کرتے ہیں۔ QoS ترتیبات ترسیل کے رویے کو کنفیگر کرتی ہیں۔ اپنی میسج پیچیدہ ڈیٹا کو ہینڈل کرتی ہیں۔

**اہم نکات**:
- • پبلش-سبسکرائیب لوس کوپلنگ کو فعال کرتا ہے
- • QoS ترتیبات مواصلات کو ٹیون کرتی ہیں
- • مناسب لائف سائیکل مینجمنٹ ضروری ہے
- • پیچیدہ ڈیٹا کے لیے اپنی میسج
- • فریکوئنسی، قابلیت، ریسورس کو توازن دیں

<h2 className="second-heading">وسائل</h2>
<div className="underline-class"></div>

- • [ROS 2 نوڈس/ٹاپکس](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
- • [QoS گائیڈ](https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service.html)

**نیویگیشن**: [← پچھلا](./01-architecture.md) | [اگلا →](./03-services-actions.md)