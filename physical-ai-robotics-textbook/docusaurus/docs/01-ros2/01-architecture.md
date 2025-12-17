---
sidebar_position: 1
title: "ROS 2 Architecture and Concepts"
id: "01-ros2-architecture"
---

import ReadingTime from '@site/src/components/ReadingTime';

<ReadingTime minutes={5} />

<h1 className="main-heading">ROS 2 Architecture and Concepts</h1>
<div className="underline-class"></div>

**Learning Objectives**:
- • Describe ROS 2 core architecture
- • Explain DDS-based communication model
- • Understand QoS profiles
- • Identify nodes, topics, services, actions
- • Set up ROS 2 environment

**Prerequisites**: Robotics basics, Python | **Time**: 2-3 hours

<div className="border-line"></div>

<h2 className="second-heading">Introduction</h2>
<div className="underline-class"></div>

ROS 2 provides middleware infrastructure for humanoid robot communication. Built on DDS, it offers enhanced reliability, real-time support, and production deployment capabilities.

<div className="border-line"></div>

<h2 className="second-heading">ROS 1 vs ROS 2</h2>
<div className="underline-class"></div>

**Key Improvements**:
- • **Real-time support**: Better for critical applications
- • **Multi-robot systems**: Enhanced coordination
- • **Production deployment**: Security and stability
- • **Middleware flexibility**: DDS implementations

**Key Differences**:
- • **Communication**: DDS-based middleware
- • **QoS**: Configurable reliability
- • **Security**: Built-in features
- • **Lifecycle**: Robust management

<div className="border-line"></div>

<h2 className="second-heading">Core Components</h2>
<div className="underline-class"></div>

<h3 className="third-heading">Nodes</h3>
<div className="underline-class"></div>

- • Separate processes
- • Inter-node communication
- • Encapsulated functionality
- • Multi-language support
```python
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')
        self.get_logger().info('Hello from node!')
```

<h3 className="third-heading">DDS Middleware</h3>
<div className="underline-class"></div>

- • Data-centric communication
- • Publisher-subscriber model
- • Automatic discovery
- • Configurable QoS

<h3 className="third-heading">Topics</h3>
<div className="underline-class"></div>

- • Asynchronous pub/sub
- • One-to-many communication
- • Message types (.msg files)
- • Real-time capable

<h3 className="third-heading">Services</h3>
<div className="underline-class"></div>

- • Synchronous request-response
- • Client-server pattern
- • Service types (.srv files)
- • Blocking calls

<h3 className="third-heading">Actions</h3>
<div className="underline-class"></div>

- • Goal-feedback-result pattern
- • Long-running tasks
- • Cancellation support
- • Action types (.action files)

<div className="border-line"></div>

<h2 className="second-heading">QoS Profiles</h2>
<div className="underline-class"></div>

**Reliability**: Reliable (TCP-like) vs Best Effort (UDP-like)
**Durability**: Transient Local vs Volatile
**History**: Keep Last (N messages) vs Keep All
```python
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

qos = QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE
)
```

<div className="border-line"></div>

<h2 className="second-heading">Client Libraries</h2>
<div className="underline-class"></div>

**rclcpp (C++)**: High-performance, real-time systems
**rclpy (Python)**: Prototyping, rapid development

<div className="border-line"></div>

<h2 className="second-heading">Exercises</h2>
<div className="underline-class"></div>

**Exercise 1.1.1**: Environment Setup (⭐⭐, 25-35 min)
- Verify ROS 2 installation
- Create workspace and package
- Implement publisher node
- Test and verify

**Exercise 1.1.2**: QoS Configuration (⭐⭐⭐, 35-45 min)
- Create publishers with different QoS
- Test reliable vs best effort
- Compare performance
- Analyze use cases

**Exercise 1.1.3**: Communication Analysis (⭐⭐, 30-40 min)
- Create multiple nodes
- Analyze topology with ROS 2 tools
- Document flows
- Create visualizations

<div className="border-line"></div>

<h2 className="second-heading">Common Issues</h2>
<div className="underline-class"></div>

**Node Discovery**:
```bash
echo $ROS_DOMAIN_ID
export ROS_DOMAIN_ID=0
ros2 node list
```

**QoS Mismatch**:
```python
qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
# Match on publisher and subscriber
```

**Workspace Setup**:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
source install/setup.bash
```

**Performance**:
```python
qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
```

**Lifecycle**:
```python
def destroy_node(self):
    if self.timer:
        self.timer.cancel()
    super().destroy_node()
```

<div className="border-line"></div>

<h2 className="second-heading">Summary</h2>
<div className="underline-class"></div>

ROS 2 uses DDS middleware with configurable QoS profiles. Core components include nodes (execution units), topics (pub/sub), services (req/resp), and actions (goal/feedback/result).

**Key Takeaways**:
- • DDS provides enhanced reliability
- • QoS enables fine-tuned communication
- • Supports real-time and multi-robot systems
- • Proper setup is essential
- • Understanding patterns is crucial

<h2 className="second-heading">Resources</h2>
<div className="underline-class"></div>

- • [ROS 2 Architecture](https://docs.ros.org/en/humble/Concepts/About-ROS-2-Architecture.html)
- • [QoS Guide](https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service.html)
- • [ROS 2 Demos](https://github.com/ros2/demos)

**Navigation**: [← Previous](../00-introduction/05-syllabus.md) | [Next →](./02-nodes-topics.md)

<h2 className="second-heading">Quick Reference</h2>
<div className="underline-class"></div>

| Component | Purpose | Best Practice |
|-----------|---------|---------------|
| Nodes | Execution units | Single responsibility |
| Topics | Async comm | Use appropriate QoS |
| Services | Sync comm | Request/response |
| Actions | Long ops | Feedback/progress |
| QoS | Config | Match pub/sub profiles |