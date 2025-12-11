---
sidebar_position: 2
title: "Nodes and Topics: The Publish-Subscribe Pattern"
id: "01-ros2-nodes-topics"
---

import ReadingTime from '@site/src/components/ReadingTime';
import ViewToggle from '@site/src/components/ViewToggle';

<ReadingTime minutes={18} />
<!-- <ViewToggle /> -->

<h1 className="main-heading">Nodes and Topics: The Publish-Subscribe Pattern</h1>
<div className="underline-class"></div>
<div className="full-content">

**Module**: 01 - ROS 2
**Learning Objectives**:
- Create ROS 2 nodes in both Python and C++ with proper structure and lifecycle
- Implement publishers and subscribers with appropriate QoS configurations
- Design effective topic-based communication patterns for humanoid robotics
- Handle message serialization and deserialization efficiently
- Debug common node and topic communication issues

**Prerequisites**: Understanding of ROS 2 architecture and basic Python/C++ programming
**Estimated Time**: 3-4 hours
<div className="border-line"></div>
---

<h2 className="second-heading">
 Introduction
</h2>

Nodes and topics form the foundation of ROS 2's communication system. Understanding how to create nodes and use topics effectively is essential for building distributed robotic systems. The publish-subscribe pattern enables asynchronous communication between nodes, allowing for scalable and decoupled robot architectures. This chapter covers the essential concepts needed to implement effective node-to-node communication in humanoid robotics applications.

The publish-subscribe pattern is particularly well-suited for robotics because it allows sensors to publish data without knowing who will consume it, and controllers to subscribe to data without knowing where it originates. This loose coupling is essential for complex robotic systems where components may be added, removed, or replaced without affecting other parts of the system.

<div className="border-line"></div>
---
<h2 className="second-heading">
Understanding Nodes
</h2>

A node is a process that performs computation in a ROS 2 system. Nodes are the basic building blocks of a ROS 2 application and can be thought of as individual programs that perform specific tasks.
<div className="border-line"></div>

<h2 className="third-heading">
Node Structure
</h2>

Every ROS 2 node, regardless of language, follows a similar structure:
- ▸ **Initialization**: Setting up the node with a name and namespace
- ▸ **Entity Creation**: Creating publishers, subscribers, services, etc.
- ▸ **Processing Loop**: Either event-driven or timer-based execution
- ▸ **Cleanup**: Proper shutdown procedures
<div className="border-line"></div>

<h2 className="third-heading">
Creating Nodes in Python
</h2>
<div className="border-line"></div>

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        # Initialize the node with a name
        super().__init__('my_node_name')

        # Node-specific initialization code goes here
        self.get_logger().info('MyNode has been initialized')

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```
<div className="border-line"></div>
<h2 className="third-heading">
Creating Nodes in C++
</h2>
<div className="border-line"></div>

```cpp
#include "rclcpp/rclcpp.hpp"

class MyNode : public rclcpp::Node
{
public:
    MyNode() : Node("my_node_name")
    {
        RCLCPP_INFO(this->get_logger(), "MyNode has been initialized");
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MyNode>());
    rclcpp::shutdown();
    return 0;
}
```
<div className="border-line"></div>
<h2 className="second-heading">
Topic Communication
</h2>
<div className="border-line"></div>

Topics enable asynchronous communication between nodes using a publish-subscribe pattern. Publishers send messages to a topic, and subscribers receive messages from that topic.
<div className="border-line"></div>

<h2 className="third-heading">
Publishers
</h2>
<div className="border-line"></div>

A publisher sends messages to a topic. Multiple publishers can publish to the same topic, and multiple subscribers can listen to the same topic.
<div className="border-line"></div>

<div className="fourth-heading">
Python Publisher Example
</div>
<div className="border-line"></div>

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
<div className="border-line"></div>

<div className="fourth-heading">
C++ Publisher Example
</div>
<div className="border-line"></div>

```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        timer_ = this->create_wall_timer(
            500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        auto message = std_msgs::msg::String();
        message.data = "Hello World: " + std::to_string(count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};
```

<div className="third-heading">
Subscribers
</div>
<div className="border-line"></div>

Subscribers receive messages from topics. When a message is published to a topic, all subscribers to that topic receive a copy of the message.
<div className="border-line"></div>

<div className="fourth-heading">
Python Subscriber Example
</div>
<div className="border-line"></div>


```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

<div className="fourth-heading">
C++ Subscriber Example
</div>
<div className="border-line"></div>

```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MinimalSubscriber : public rclcpp::Node
{
public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "topic", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
            });
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};
```

<h2 className="second-heading">
Advanced Topic Concepts
</h2>
<div className="border-line"></div>
<h2 className="second-heading">
 Quality of Service (QoS) Settings
</h2>
<div className="border-line"></div>
QoS settings allow you to configure how messages are delivered:
<div className="border-line"></div>

```python
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# Configure for real-time requirements
qos_profile = QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE
)
```

<h2 className="third-heading">
Topic Namespaces
</h2>
<div className="border-line"></div>

Topics can be organized using namespaces for better organization:
<div className="border-line"></div>

```python
# Using a namespace
publisher = self.create_publisher(String, 'robot1/sensor_data', 10)
publisher = self.create_publisher(String, 'robot2/sensor_data', 10)
```

<h2 className="third-heading">
Message Types
</h2>
<div className="border-line"></div>

ROS 2 supports various built-in message types and allows custom message types:
<div className="border-line"></div>

<h2 className="fourth-heading">
Built-in Types
</h2>
<div className="border-line"></div>

- ➤ `std_msgs`: Basic data types (String, Int32, Float64, etc.)
- ➤ `geometry_msgs`: Geometric primitives (Point, Pose, Twist, etc.)
- ➤ `sensor_msgs`: Sensor data types (LaserScan, Image, JointState, etc.)
- ➤ `nav_msgs`: Navigation-related messages (Odometry, Path, etc.)

<div className="border-line"></div>

<h2 className="fourth-heading">
Custom Message Types
</h2>
<div className="border-line"></div>

To create a custom message type, create a `.msg` file in the `msg/` directory:
<div className="border-line"></div>


```
# MyCustomMessage.msg
string name
int32 value
float64[] values
geometry_msgs/Point position
```

<h2 className="second-heading">
Hands-On Exercises
</h2>
<div className="border-line"></div>

:::tip Exercise 1.2.1: Multi-Node Publisher-Subscriber Network

**Objective**: Create a multi-node system with multiple publishers and subscribers to understand topic communication patterns.

**Difficulty**: ⭐⭐ Medium

**Time Estimate**: 30-40 minutes

**Requirements**:
1. Create a sensor publisher node that publishes simulated sensor data
2. Create multiple subscriber nodes that process the same sensor data differently
3. Implement proper QoS configurations for the communication
4. Test the system and verify all nodes can communicate correctly

**Starter Code**:
```python title="sensor_network.py"
#!/usr/bin/env python3
"""
ROS 2 Nodes and Topics Exercise - Multi-Node Network

This script creates a multi-node system with publishers and subscribers.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String, Float32
from sensor_msgs.msg import LaserScan
import random


class SensorPublisher(Node):
    """Publisher node that simulates sensor data."""

    def __init__(self):
        """Initialize the sensor publisher."""
        super().__init__('sensor_publisher')

        # TODO: Create publishers for different sensor topics
        # TODO: Create timer to publish data periodically
        # TODO: Initialize sensor data variables

        self.get_logger().info('Sensor Publisher initialized')

    def publish_sensor_data(self):
        """Publish simulated sensor data."""
        # TODO: Generate and publish sensor messages
        pass


class DataProcessorA(Node):
    """Subscriber node that processes sensor data in one way."""

    def __init__(self):
        """Initialize the first data processor."""
        super().__init__('data_processor_a')

        # TODO: Create subscription to sensor topic
        # TODO: Initialize processing variables

        self.get_logger().info('Data Processor A initialized')

    def sensor_callback(self, msg):
        """Process incoming sensor data."""
        # TODO: Implement processing logic
        # TODO: Log processed results
        pass


class DataProcessorB(Node):
    """Subscriber node that processes sensor data in another way."""

    def __init__(self):
        """Initialize the second data processor."""
        super().__init__('data_processor_b')

        # TODO: Create subscription to sensor topic
        # TODO: Initialize processing variables

        self.get_logger().info('Data Processor B initialized')

    def sensor_callback(self, msg):
        """Process incoming sensor data differently."""
        # TODO: Implement alternative processing logic
        # TODO: Log processed results
        pass


def main(args=None):
    """Main function to run the multi-node system."""
    rclpy.init(args=args)

    # TODO: Create all nodes
    # TODO: Create multi-threaded executor
    # TODO: Add nodes to executor
    # TODO: Spin the executor
    # TODO: Clean up nodes

    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Deliverable**: A working multi-node system with one publisher and multiple subscribers processing the same data differently.

**Success Criteria**:
- [ ] Sensor publisher node created and publishing data
- [ ] Multiple subscriber nodes receiving the same data
- [ ] Different processing logic in each subscriber
- [ ] Proper QoS configurations applied
- [ ] All nodes communicate without errors

**Test Commands**:
```bash
# Create a new package for the exercise
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python sensor_network_pkg --dependencies rclpy std_msgs sensor_msgs

# Navigate to the package directory
cd sensor_network_pkg
mkdir sensor_network_pkg
touch sensor_network_pkg/__init__.py

# Copy your code to sensor_network_pkg/sensor_network.py

# Build and run the system
cd ~/ros2_ws
colcon build --packages-select sensor_network_pkg
source install/setup.bash

# Run the multi-node system
ros2 run sensor_network_pkg sensor_network

# Verify topics are being published
ros2 topic list
ros2 topic echo /sensor_data
```

**Expected Output**:
Multiple subscriber nodes should receive the same sensor data and process it differently, demonstrating the publish-subscribe pattern.

**Challenge**: Add a data aggregator node that subscribes to the outputs of both processors and combines the results.

**Hints**:
<details>
<summary>Click for hint 1</summary>

Use `rclpy.executors.MultiThreadedExecutor()` to run multiple nodes in the same process.
</details>

<details>
<summary>Click for hint 2</summary>

Make sure all nodes use the same QoS profile for reliable communication.
</details>
:::

:::tip Exercise 1.2.2: Custom Message Types and Complex Data Handling

**Objective**: Create and use custom message types to handle complex data structures in ROS 2.

**Difficulty**: ⭐⭐⭐ Hard

**Time Estimate**: 45-60 minutes

**Requirements**:
1. Define custom message types for humanoid robot data (joints, sensors, etc.)
2. Create publisher and subscriber nodes that use these custom messages
3. Implement proper serialization and deserialization of complex data
4. Test the system with realistic humanoid robot data

**Starter Code**:
```python title="custom_messages_demo.py"
#!/usr/bin/env python3
"""
ROS 2 Nodes and Topics Exercise - Custom Message Types

This script demonstrates creating and using custom message types.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
# TODO: Import custom message types after creating them
# from humanoid_msgs.msg import JointStateArray, RobotStatus

# For now, we'll use standard messages to simulate custom types
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Float32MultiArray


class HumanoidPublisher(Node):
    """Publisher node that sends humanoid robot data."""

    def __init__(self):
        """Initialize the humanoid data publisher."""
        super().__init__('humanoid_publisher')

        # TODO: Create publisher for custom humanoid message
        # For now, using JointState as example
        self.joint_pub = self.create_publisher(JointState, 'humanoid_joints', 10)

        # Create timer to publish data
        self.timer = self.create_timer(0.1, self.publish_humanoid_data)
        self.joint_names = ['hip_left', 'knee_left', 'ankle_left',
                           'hip_right', 'knee_right', 'ankle_right',
                           'shoulder_left', 'elbow_left', 'shoulder_right', 'elbow_right']
        self.joint_positions = [0.0] * len(self.joint_names)

        self.get_logger().info('Humanoid Publisher initialized')

    def publish_humanoid_data(self):
        """Publish humanoid robot joint data."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'humanoid_base'
        msg.name = self.joint_names

        # Simulate changing joint positions
        for i in range(len(self.joint_positions)):
            self.joint_positions[i] += random.uniform(-0.1, 0.1)

        msg.position = self.joint_positions[:]
        self.joint_pub.publish(msg)

        self.get_logger().info(f'Published joint states for {len(msg.name)} joints')


class HumanoidSubscriber(Node):
    """Subscriber node that receives and processes humanoid robot data."""

    def __init__(self):
        """Initialize the humanoid data subscriber."""
        super().__init__('humanoid_subscriber')

        # TODO: Create subscription to custom humanoid message
        # For now, using JointState as example
        self.joint_sub = self.create_subscription(
            JointState,
            'humanoid_joints',
            self.joint_callback,
            10)

        self.get_logger().info('Humanoid Subscriber initialized')

    def joint_callback(self, msg):
        """Process incoming humanoid joint data."""
        # Analyze joint positions
        avg_position = sum(msg.position) / len(msg.position) if msg.position else 0.0

        self.get_logger().info(f'Received {len(msg.name)} joints, avg pos: {avg_position:.3f}')

        # TODO: Add more sophisticated analysis of humanoid data


def main(args=None):
    """Main function to run the custom message demo."""
    rclpy.init(args=args)

    # Create publisher and subscriber nodes
    pub_node = HumanoidPublisher()
    sub_node = HumanoidSubscriber()

    # Create executor and add both nodes
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(pub_node)
    executor.add_node(sub_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        pub_node.destroy_node()
        sub_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Deliverable**: A complete system using custom message types to handle complex humanoid robot data.

**Success Criteria**:
- [ ] Custom message type definition files created
- [ ] Publisher node sending custom message data
- [ ] Subscriber node receiving and processing custom messages
- [ ] Proper build configuration for custom messages
- [ ] System works with realistic humanoid robot data

**Test Commands**:
```bash
# Create a custom messages package
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake humanoid_msgs

# Create msg directory and define custom message
mkdir humanoid_msgs/msg
echo "string[] joint_names
float64[] joint_positions
float64[] joint_velocities
float64[] joint_efforts
uint8[] joint_states  # 0=free, 1=locked, 2=torque_controlled
string robot_name
uint8 status  # 0=standby, 1=active, 2=error
float64 battery_level" > humanoid_msgs/msg/HumanoidState.msg

# Update CMakeLists.txt and package.xml to include message generation
# Then build the workspace
cd ~/ros2_ws
colcon build --packages-select humanoid_msgs
source install/setup.bash

# Build the demo package
colcon build --packages-select sensor_network_pkg
source install/setup.bash

# Run the demo
ros2 run sensor_network_pkg custom_messages_demo
```

**Expected Output**:
Custom message types should be properly defined and used by publisher/subscriber nodes to exchange complex humanoid robot data.

**Challenge**: Create a custom service that allows remote configuration of the humanoid robot's joint parameters.

**Hints**:
<details>
<summary>Click for hint 1</summary>

Custom messages go in the `msg/` directory of your package and need to be listed in CMakeLists.txt for generation.
</details>

<details>
<summary>Click for hint 2</summary>

Use the `rosidl_generate_interfaces()` function in CMakeLists.txt to generate code for your custom messages.
</details>
:::

:::tip Exercise 1.2.3: Performance Optimization and QoS Tuning

**Objective**: Optimize node performance and tune QoS settings for different communication requirements.

**Difficulty**: ⭐⭐ Medium

**Time Estimate**: 35-45 minutes

**Requirements**:
1. Create high-frequency and low-frequency publisher/subscriber pairs
2. Configure appropriate QoS settings for each communication pattern
3. Monitor and measure performance metrics
4. Optimize the system for minimal latency and maximum reliability

**Starter Code**:
```python title="performance_optimizer.py"
#!/usr/bin/env python3
"""
ROS 2 Nodes and Topics Exercise - Performance Optimization

This script demonstrates performance optimization and QoS tuning.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from std_msgs.msg import String, UInt32
from sensor_msgs.msg import Image
import time
import random


class PerformancePublisher(Node):
    """Publisher node with configurable performance parameters."""

    def __init__(self):
        """Initialize the performance publisher."""
        super().__init__('performance_publisher')

        # High-frequency topic (e.g., sensor data)
        high_freq_qos = QoSProfile(
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Accept some drops for speed
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        self.high_freq_pub = self.create_publisher(UInt32, 'high_freq_data', high_freq_qos)

        # Low-frequency topic (e.g., configuration updates)
        low_freq_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,  # Ensure delivery
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL  # Available to late joiners
        )
        self.low_freq_pub = self.create_publisher(String, 'low_freq_data', low_freq_qos)

        # Create timers for different frequencies
        self.high_freq_timer = self.create_timer(0.01, self.publish_high_freq)  # 100 Hz
        self.low_freq_timer = self.create_timer(1.0, self.publish_low_freq)     # 1 Hz

        self.high_freq_counter = 0
        self.low_freq_counter = 0

        self.get_logger().info('Performance Publisher initialized')

    def publish_high_freq(self):
        """Publish high-frequency data."""
        msg = UInt32()
        msg.data = self.high_freq_counter
        self.high_freq_pub.publish(msg)

        if self.high_freq_counter % 100 == 0:  # Log every 100 messages at 100Hz
            self.get_logger().info(f'High freq: {msg.data}')

        self.high_freq_counter += 1

    def publish_low_freq(self):
        """Publish low-frequency data."""
        msg = String()
        msg.data = f'Config update #{self.low_freq_counter}'
        self.low_freq_pub.publish(msg)

        self.get_logger().info(f'Low freq: {msg.data}')
        self.low_freq_counter += 1


class PerformanceSubscriber(Node):
    """Subscriber node with configurable performance monitoring."""

    def __init__(self):
        """Initialize the performance subscriber."""
        super().__init__('performance_subscriber')

        # High-frequency subscription
        high_freq_qos = QoSProfile(
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        self.high_freq_sub = self.create_subscription(
            UInt32, 'high_freq_data', self.high_freq_callback, high_freq_qos)

        # Low-frequency subscription
        low_freq_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.low_freq_sub = self.create_subscription(
            String, 'low_freq_data', self.low_freq_callback, low_freq_qos)

        # Performance tracking
        self.last_high_freq_time = time.time()
        self.high_freq_count = 0
        self.last_low_freq_time = time.time()
        self.low_freq_count = 0

        self.get_logger().info('Performance Subscriber initialized')

    def high_freq_callback(self, msg):
        """Process high-frequency data."""
        current_time = time.time()
        self.high_freq_count += 1

        # Calculate frequency every 100 messages
        if self.high_freq_count % 100 == 0:
            elapsed = current_time - self.last_high_freq_time
            freq = 100 / elapsed if elapsed > 0 else 0
            self.get_logger().info(f'High freq rate: {freq:.2f} Hz')
            self.last_high_freq_time = current_time

    def low_freq_callback(self, msg):
        """Process low-frequency data."""
        current_time = time.time()
        self.low_freq_count += 1

        # Calculate frequency
        elapsed = current_time - self.last_low_freq_time
        freq = 1 / elapsed if elapsed > 0 else 0
        self.get_logger().info(f'Low freq rate: {freq:.2f} Hz, msg: {msg.data}')
        self.last_low_freq_time = current_time


def main(args=None):
    """Main function to run the performance optimization demo."""
    rclpy.init(args=args)

    # Create publisher and subscriber nodes
    pub_node = PerformancePublisher()
    sub_node = PerformanceSubscriber()

    # Create executor and add both nodes
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(pub_node)
    executor.add_node(sub_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        pub_node.destroy_node()
        sub_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Deliverable**: A performance-optimized system with appropriately configured QoS settings for different communication requirements.

**Success Criteria**:
- [ ] High-frequency data published at expected rate with appropriate QoS
- [ ] Low-frequency data published reliably with appropriate QoS
- [ ] Performance metrics measured and logged
- [ ] QoS settings optimized for each communication pattern
- [ ] System demonstrates understanding of QoS trade-offs

**Test Commands**:
```bash
# Create performance optimization package
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python performance_pkg --dependencies rclpy std_msgs sensor_msgs

# Add the performance optimizer code to performance_pkg/performance_optimizer.py

# Build and run the performance test
cd ~/ros2_ws
colcon build --packages-select performance_pkg
source install/setup.bash

# Run the performance optimization demo
ros2 run performance_pkg performance_optimizer

# Monitor performance with ROS tools
ros2 topic hz /high_freq_data
ros2 topic hz /low_freq_data
```

**Expected Output**:
High-frequency topics should operate at high rates with potential message drops, while low-frequency topics should ensure reliable delivery with QoS settings optimized for their requirements.

**Challenge**: Implement adaptive QoS that changes based on network conditions and system load.

**Hints**:
<details>
<summary>Click for hint 1</summary>

Use `ros2 topic hz` command to measure actual message rates and compare with expected rates.
</details>

<details>
<summary>Click for hint 2</summary>

For high-frequency data, consider using BEST_EFFORT reliability to avoid blocking when network is congested.
</details>
:::

---

<h2 className="second-heading">
Common Issues and Debugging
</h2>
<div className="border-line"></div>

:::caution Common Problems

**Problem 1: Topic Connection and Discovery Issues**

**Symptoms**:
- Publisher and subscriber nodes cannot communicate despite using the same topic name
- Topic shows as inactive in `ros2 topic list`
- Error messages about domain ID mismatches or network issues
- Nodes running on different machines cannot discover each other

**Cause**: ROS 2 uses DDS for communication which relies on domain IDs, network configuration, and QoS compatibility. If any of these don't match, nodes cannot communicate.

**Solution**:
```bash
# Check and ensure consistent domain ID:
echo $ROS_DOMAIN_ID  # Should be the same on all machines
export ROS_DOMAIN_ID=0  # Set to default if needed

# For multi-machine communication:
# 1. Ensure same ROS_DOMAIN_ID on all machines
# 2. Check firewall settings for DDS ports (7400-7500)
# 3. Verify network connectivity: ping other_machine_ip
# 4. Use specific network interface if needed:
# export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp  # Alternative RMW
```

**Verification**:
```bash
# Test node and topic discovery:
ros2 node list  # Should show all running nodes
ros2 topic list  # Should show all active topics
ros2 topic info /topic_name  # Check topic connections and QoS
ros2 node info /node_name  # Check node connections
```

---

**Problem 2: QoS Profile Incompatibility**

**Symptoms**:
- Messages are not being received despite nodes being connected
- Intermittent message delivery
- Warning messages about incompatible QoS settings
- Performance issues with communication

**Cause**: When publisher and subscriber QoS profiles are incompatible, messages may not be delivered or may be delivered unreliably.

**Solution**:
```python
# Ensure QoS compatibility between publisher and subscriber:

# For high-frequency sensor data (where some drops are acceptable):
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

high_freq_qos = QoSProfile(
    depth=5,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Accept some drops
    history=QoSHistoryPolicy.KEEP_LAST
)

# For critical control commands (where delivery is essential):
critical_qos = QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE,  # Ensure delivery
    history=QoSHistoryPolicy.KEEP_LAST
)

# Publisher side:
publisher = self.create_publisher(String, 'topic_name', critical_qos)

# Subscriber side - must match publisher's QoS:
subscription = self.create_subscription(
    String,
    'topic_name',
    callback,
    critical_qos  # Must match publisher's QoS
)
```

**Verification**:
```bash
# Check QoS settings for a topic:
ros2 topic info /topic_name -v  # Verbose output shows QoS settings
```

---

**Problem 3: Memory and Performance Issues with High-Frequency Publishing**

**Symptoms**:
- High memory usage over time
- Slow message publishing or processing
- Node becoming unresponsive
- Dropped messages under high load
- CPU usage spikes

**Cause**: Improper QoS settings, large message sizes, high frequency publishing, or inefficient message handling can cause performance issues.

**Solution**:
```python
# Optimize publisher settings for high-frequency data:

# For high-frequency sensor data, limit history and use best effort:
high_freq_qos = QoSProfile(
    depth=1,  # Keep only the most recent message
    reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Accept occasional drops
    history=QoSHistoryPolicy.KEEP_LAST
)

# For critical data, use reliable delivery but optimize frequency:
critical_qos = QoSProfile(
    depth=5,  # Small but sufficient queue
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST
)

# Reduce message publishing frequency if possible:
# Instead of publishing every cycle, consider:
if self.counter % 5 == 0:  # Publish every 5th cycle
    self.publisher.publish(msg)
    self.counter = 0
self.counter += 1
```

**Verification**:
```bash
# Monitor performance:
ros2 topic hz /topic_name  # Check actual message frequency
ros2 topic bw /topic_name  # Check bandwidth usage
htop  # Monitor process resource usage
ros2 lifecycle list  # Check node states if using lifecycle nodes
```

---

**Problem 4: Node Lifecycle and Resource Management Issues**

**Symptoms**:
- Nodes not shutting down cleanly
- Resource leaks after node termination
- Memory usage increases over time
- Nodes stuck in certain states
- Unexpected behavior during startup/shutdown

**Cause**: Improper lifecycle management, missing cleanup code, or unhandled exceptions during node destruction.

**Solution**:
```python
# Proper node cleanup and resource management:

class ProperNode(Node):
    def __init__(self):
        super().__init__('proper_node')

        # Initialize resources
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.publisher = self.create_publisher(String, 'topic', 10)
        self.subscription = self.create_subscription(
            String, 'topic', self.callback, 10)

        # Keep references to prevent garbage collection issues
        self.resources = []

    def timer_callback(self):
        # Handle timer events
        pass

    def callback(self, msg):
        # Handle subscription messages
        pass

    def destroy_node(self):
        """Properly clean up resources."""
        # Cancel timers
        if self.timer is not None:
            self.timer.cancel()

        # Clean up other resources
        self.resources.clear()

        # Call parent cleanup
        super().destroy_node()

        self.get_logger().info('Node properly destroyed')
```

**Verification**:
```bash
# Check for zombie processes:
ps aux | grep ros
# Monitor memory usage over time:
watch -n 1 'ps aux | grep your_node_name | grep -v grep'
```

---

**Problem 5: Message Serialization and Deserialization Issues**

**Symptoms**:
- Messages not being properly received or sent
- Error messages about message format or serialization
- Data corruption in transmitted messages
- Type mismatch errors between publisher and subscriber

**Cause**: Incompatible message types, custom message definition issues, or serialization problems.

**Solution**:
```python
# Ensure proper message handling:

# For custom messages, make sure they're properly defined and built:
# 1. Message definition file exists: msg/MyMessage.msg
# 2. CMakeLists.txt includes: find_package(rosidl_default_generators REQUIRED)
# 3. Package.xml includes: <depend>rosidl_default_runtime</depend>
# 4. Built with: colcon build --packages-select your_message_package

# Proper message creation and handling:
def publisher_callback(self):
    msg = MyCustomMessage()  # Make sure import is correct
    msg.field1 = "value"
    msg.field2 = 42
    self.publisher.publish(msg)

def subscription_callback(self, msg):
    # Verify message type and handle appropriately
    if hasattr(msg, 'field1'):
        self.get_logger().info(f'Received: {msg.field1}')
    else:
        self.get_logger().error('Message format mismatch')
```

**Verification**:
```bash
# Check message types:
ros2 interface show std_msgs/msg/String  # View message definition
ros2 topic type /topic_name  # Check topic message type
ros2 msg list  # List all available message types
```
:::

---
<div className="border-line"></div>
<h2 className="second-heading">
Summary
</h2>
<div className="border-line"></div>

In this chapter, you learned:
<div className="border-line"></div>
- ✅ How to create ROS 2 nodes in both Python and C++ with proper structure and lifecycle
- ✅ How to implement publishers and subscribers with appropriate QoS configurations
- ✅ How to design effective topic-based communication patterns for humanoid robotics
- ✅ How to handle message serialization and deserialization efficiently
- ✅ How to debug common node and topic communication issues

<div className="border-line"></div>

**Key Takeaways**:
<div className="border-line"></div>
- ➤ The publish-subscribe pattern enables loose coupling between robot components
- ➤ QoS settings allow fine-tuning of communication behavior for different requirements
- ➤ Proper node lifecycle management is essential for reliable operation
- ➤ Custom message types enable complex data exchange in humanoid robotics
- ➤ Performance optimization requires balancing frequency, reliability, and resource usage
<div className="border-line"></div>
---

<h2 className="second-heading">
Additional Resources
</h2>
<div className="border-line"></div>

**Official Documentation**:
- ➙ [ROS 2 Nodes and Topics](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Single-Package-Define-And-Use-Interface-Python.html)
- [Quality of Service in ROS 2](https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service.html)

**Tutorials**:
- ➙ [Writing a Simple Publisher and Subscriber](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
- ➙ [Custom Message Definitions](https://docs.ros.org/en/humble/Tutorials/Beginner-Tutorials/Creating-Your-First-ROS2-Message.html)

**Example Code**:
- ➙ [ROS 2 Demos Repository](https://github.com/ros2/demos)
<div className="border-line"></div>
---

**Navigation**: [← Previous Chapter](./01-architecture.md) | [Next Chapter →](./03-services-actions.md)

</div>
<div className="summary-content">
<div className="border-line"></div>
<h2 className="second-heading">
Chapter Summary
</h2>
<div className="border-line"></div>
### Key Concepts
<div className="border-line"></div>
- ➣ **Publish-Subscribe Pattern**: Asynchronous communication enabling loose coupling between nodes
- ➣ **Node Structure**: Standard initialization, entity creation, processing, and cleanup patterns
- ➣ **QoS Configuration**: Quality of Service settings for reliability, history, and durability
- ➣ **Message Types**: Built-in and custom message definitions for data exchange
- ➣ **Performance Optimization**: Balancing frequency, reliability, and resource usage
<div className="border-line"></div>

<h2 className="third-heading">
Essential Code Pattern
</h2>

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class CommunicationNode(Node):
    def __init__(self):
        super().__init__('comm_node')
        # Create publisher with appropriate QoS
        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
        self.publisher = self.create_publisher(String, 'topic_name', qos_profile)
        # Create subscriber
        self.subscriber = self.create_subscription(String, 'topic_name', self.callback, qos_profile)

    def callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = CommunicationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```
<div className="border-line"></div>

<h2 className="third-heading">
Quick Reference
</h2>
<div className="border-line"></div>

| Component | Purpose | Best Practice |
|-----------|---------|---------------|
| Publishers | Send messages asynchronously | Use appropriate QoS for reliability |
| Subscribers | Receive messages from topics | Match QoS with publisher |
| QoS Settings | Configure delivery behavior | BEST_EFFORT for high freq, RELIABLE for critical |
| Custom Messages | Complex data structures | Define .msg files, build properly |
| Namespaces | Organize topics | Use descriptive names with slashes |

<div className="border-line"></div>


<h2 className="third-heading">
What You Built
</h2>

- ➛ Multi-node publisher-subscriber system
- ➛ Custom message type handling
- ➛ Performance-optimized communication patterns

### Next Steps
Continue to [Services and Actions](./03-services-actions.md) to learn about synchronous communication and goal-oriented interactions.

</div>