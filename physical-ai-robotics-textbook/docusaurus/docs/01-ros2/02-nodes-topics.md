---
sidebar_position: 2
title: "Nodes and Topics: The Publish-Subscribe Pattern"
id: "01-ros2-nodes-topics"
---

import ReadingTime from '@site/src/components/ReadingTime';

<ReadingTime minutes={6} />

<h1 className="main-heading">Nodes and Topics: The Publish-Subscribe Pattern</h1>
<div className="underline-class"></div>

**Learning Objectives**:
- • Create ROS 2 nodes in Python and C++
- • Implement publishers and subscribers with QoS
- • Design topic-based communication patterns
- • Handle message serialization
- • Debug communication issues

**Prerequisites**: ROS 2 architecture, Python/C++ | **Time**: 3-4 hours

<div className="border-line"></div>

<h2 className="second-heading">Introduction</h2>
<div className="underline-class"></div>

Nodes and topics form ROS 2's communication foundation. The publish-subscribe pattern enables asynchronous, decoupled communication ideal for robotics.

<div className="border-line"></div>

<h2 className="second-heading">Nodes</h2>
<div className="underline-class"></div>

Processes that perform computation in ROS 2 systems.

<h3 className="third-heading">Node Structure</h3>
<div className="underline-class"></div>

- • **Initialization**: Setup with name/namespace
- • **Entity Creation**: Publishers, subscribers, services
- • **Processing Loop**: Event-driven or timer-based
- • **Cleanup**: Proper shutdown

<h3 className="third-heading">Python Node</h3>
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

<h3 className="third-heading">C++ Node</h3>
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

<h2 className="second-heading">Topics</h2>
<div className="underline-class"></div>

Asynchronous publish-subscribe communication.

<h3 className="third-heading">Publisher</h3>
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

<h3 className="third-heading">Subscriber</h3>
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

<h2 className="second-heading">QoS Settings</h2>
<div className="underline-class"></div>
```python
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

qos = QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE
)
pub = self.create_publisher(String, 'topic', qos)
```

<h3 className="third-heading">Namespaces</h3>
<div className="underline-class"></div>
```python
pub = self.create_publisher(String, 'robot1/sensors', 10)
```

<h3 className="third-heading">Message Types</h3>
<div className="underline-class"></div>

- • **std_msgs**: Basic types (String, Int32)
- • **geometry_msgs**: Point, Pose, Twist
- • **sensor_msgs**: LaserScan, Image
- • **nav_msgs**: Odometry, Path

<h4 className="fourth-heading">Custom Messages</h4>
<div className="underline-class"></div>
```
# MyMessage.msg
string name
int32 value
float64[] values
```

<div className="border-line"></div>

<h2 className="second-heading">Exercises</h2>
<div className="underline-class"></div>

**Exercise 1.2.1**: Multi-Node Network (⭐⭐, 30-40 min)
- Create sensor publisher
- Multiple subscribers with different processing
- Proper QoS configurations

**Exercise 1.2.2**: Custom Messages (⭐⭐⭐, 45-60 min)
- Define custom humanoid messages
- Implement publisher/subscriber
- Handle complex data structures

**Exercise 1.2.3**: Performance Optimization (⭐⭐, 35-45 min)
- High/low frequency publishers
- Configure appropriate QoS
- Monitor and optimize performance

<div className="border-line"></div>

<h2 className="second-heading">Common Issues</h2>
<div className="underline-class"></div>

**Topic Discovery**:
```bash
echo $ROS_DOMAIN_ID
ros2 topic list
ros2 topic info /topic_name
```

**QoS Incompatibility**:
```python
# Match QoS between pub/sub
qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
```

**Performance**:
```python
# Optimize high-frequency
qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
```

**Resource Management**:
```python
def destroy_node(self):
    if self.timer:
        self.timer.cancel()
    super().destroy_node()
```

**Serialization**:
```bash
ros2 interface show std_msgs/msg/String
ros2 topic type /topic_name
```

<div className="border-line"></div>

<h2 className="second-heading">Summary</h2>
<div className="underline-class"></div>

Nodes are computation processes. Topics enable asynchronous publish-subscribe communication. QoS settings configure delivery behavior. Custom messages handle complex data.

**Key Takeaways**:
- • Publish-subscribe enables loose coupling
- • QoS settings tune communication
- • Proper lifecycle management is essential
- • Custom messages for complex data
- • Balance frequency, reliability, resources

<h2 className="second-heading">Resources</h2>
<div className="underline-class"></div>

- • [ROS 2 Nodes/Topics](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
- • [QoS Guide](https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service.html)

**Navigation**: [← Previous](./01-architecture.md) | [Next →](./03-services-actions.md)