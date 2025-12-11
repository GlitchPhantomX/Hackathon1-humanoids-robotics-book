---
sidebar_position: 1
title: "ROS 2 Architecture and Concepts"
id: "01-ros2-architecture"
---

import ReadingTime from '@site/src/components/ReadingTime';
import ViewToggle from '@site/src/components/ViewToggle';

<ReadingTime minutes={15} />
<!-- <ViewToggle /> -->

<h1 className="main-heading">ROS 2 Architecture and Concepts</h1>
<div className="underline-class"></div>

<div className="full-content">

**Module**: 01 - ROS 2
**Learning Objectives**:
- • Describe the core architecture of ROS 2 and its advantages over ROS 1
- • Explain the DDS-based communication model and its benefits
- • Understand Quality of Service (QoS) profiles and their applications
- • Identify key ROS 2 concepts: nodes, topics, services, and actions
- • Set up and configure a basic ROS 2 environment

**Prerequisites**: Basic understanding of robotics concepts, Python programming skills
**Estimated Time**: 2-3 hours

<div className="border-line"></div>
---

<h2 className="second-heading">
 Introduction
</h2>
<div className="underline-class"></div>

Robot Operating System 2 (ROS 2) provides the middleware infrastructure that enables communication between different components of a humanoid robot system. Understanding its architecture is crucial for building robust and scalable robotic applications. This chapter covers the fundamental concepts that form the foundation of ROS 2, setting the stage for more advanced topics in subsequent chapters.

ROS 2 represents a significant evolution from ROS 1, addressing key limitations in real-time support, multi-robot systems, production deployment, and middleware flexibility. The DDS-based communication model provides enhanced reliability and performance for humanoid robotics applications.

<div className="border-line"></div>
---

<h2 className="second-heading">
 Evolution from ROS 1 to ROS 2
</h2>
<div className="underline-class"></div>

ROS 2 was developed to address the limitations of ROS 1, particularly in the areas of:
- • **Real-time support**: Improved real-time capabilities for critical applications
- • **Multi-robot systems**: Better support for multiple robots working together
- • **Production deployment**: Enhanced security, stability, and lifecycle management
- • **Middleware flexibility**: Support for different communication middleware (DDS implementations)
<div className="border-line"></div>
<h3 className="third-heading">
- Key Differences
</h3>
<div className="underline-class"></div>

- ➤ **Communication Layer**: ROS 2 uses DDS (Data Distribution Service) as its communication middleware
- ➤ **Quality of Service (QoS)**: Configurable reliability and performance settings
- ➤ **Security**: Built-in security features for production environments
- ➤ **Lifecycle management**: More robust node lifecycle management
<div className="border-line"></div>
---
<h2 className="second-heading">
Core Architecture Components
</h2>
<div className="border-line"></div>

<h2 className="third-heading">
Nodes
</h2>

Nodes are the fundamental execution units in ROS 2. Each node:
- ⇨ Runs as a separate process
- ⇨ Can communicate with other nodes
- ⇨ Encapsulates specific functionality
- ⇨ Can be written in different programming languages

<div className="border-line"></div>

```python
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')
        self.get_logger().info('Hello from minimal node!')
```

<h2 className="third-heading">
DDS (Data Distribution Service)
</h2>

DDS is the middleware that enables communication in ROS 2:
- ➤ **Data-Centric**: Focuses on data rather than connection endpoints
- ➤ **Publisher-Subscriber Model**: Asynchronous communication pattern
- ➤ **Discovery**: Automatic discovery of nodes and topics
- ➤ **Quality of Service**: Configurable reliability and performance settings
<div className="border-line"></div>
<h2 className="third-heading">
 Topics
</h2>

Topics enable asynchronous communication between nodes:
- ➤ **Publish-Subscribe Pattern**: One-to-many communication
- ➤ **Message Types**: Defined using .msg files
- ➤ **Reliability**: Can be configured with different QoS policies
- ➤ **Real-time Capable**: Supports real-time communication requirements
<div className="border-line"></div>
<h2 className="third-heading">
Services
</h2>

Services provide synchronous request-response communication:
- ➤ **Client-Server Pattern**: One-to-one communication
- ➤ **Request-Response**: Synchronous communication model
- ➤ **Service Types**: Defined using .srv files
- ➤ **Blocking Calls**: Client waits for server response
<div className="border-line"></div>
<h2 className="third-heading">
Actions
</h2>

Actions provide goal-oriented communication with feedback:
- ➤ **Goal-Feeback-Result Pattern**: Asynchronous with progress updates
- ➤ **Long-running Tasks**: Designed for operations that take time
- ➤ **Cancelation Support**: Ability to cancel ongoing actions
- ➤ **Action Types**: Defined using .action files
<div className="border-line"></div>
---
<h2 className="second-heading">
Quality of Service (QoS) Profiles
</h2>

QoS profiles allow you to configure communication behavior:
<div className="border-line"></div>
<h2 className="third-heading">
Reliability Policy
</h2>

- **Reliable**: All messages are delivered (TCP-like)
- **Best Effort**: Messages may be lost (UDP-like)
<div className="border-line"></div>
<h2 className="third-heading">
Durability Policy
</h2>

- **Transient Local**: Late-joining subscribers receive recent messages
- **Volatile**: No historical data provided to new subscribers
<div className="border-line"></div>
<h2 className="third-heading">
History Policy
</h2>
<div className="border-line"></div>
- **Keep Last**: Maintain last N messages
- **Keep All**: Maintain all messages (use with caution)
<div className="border-line"></div>

```python
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# Configure QoS for reliable communication
qos_profile = QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST
)
```

<h2 className="second-heading">
ROS 2 Client Libraries
</h2>
<div className="border-line"></div>
<h2 className="third-heading">
rclcpp
</h2>

- ➤ **Language**: C++
- ➤ **Use Case**: High-performance applications, real-time systems
- ➤ **Features**: Full ROS 2 feature set, direct access to low-level APIs
<div className="border-line"></div>

<h2 className="second-heading">
rclpy
</h2>

- ➤ **Language**: Python
- ➤ **Use Case**: Prototyping, scripting, rapid development
- ➤ **Features**: Easy to use, extensive Python ecosystem integration
<div className="border-line"></div>
---

<h2 className="second-heading">
Hands-On Exercises
</h2>
<div className="border-line"></div>

:::tip Exercise 1.1.1: ROS 2 Environment Setup and Node Creation

**Objective**: Set up a ROS 2 environment and create your first ROS 2 node.

**Difficulty**: ⭐⭐ Medium

**Time Estimate**: 25-35 minutes

**Requirements**:
1. Verify your ROS 2 installation is working properly
2. Create a new ROS 2 workspace and package
3. Implement a simple publisher node that broadcasts robot status
4. Test the node and verify it's publishing messages correctly

**Starter Code**:
```python title="robot_status_publisher.py"
#!/usr/bin/env python3
"""
ROS 2 Architecture Exercise - Robot Status Publisher

This script implements a simple publisher node that broadcasts robot status.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class RobotStatusPublisher(Node):
    """A simple ROS 2 node that publishes robot status messages."""

    def __init__(self):
        """Initialize the robot status publisher node."""
        super().__init__('robot_status_publisher')

        # TODO: Create a publisher for the 'robot_status' topic
        # TODO: Create a timer to publish messages periodically
        # TODO: Initialize status message with default value

        self.get_logger().info('Robot Status Publisher node initialized')

    def timer_callback(self):
        """Callback function to publish robot status."""
        # TODO: Publish the status message
        # TODO: Log the published message
        pass


def main(args=None):
    """Main function to run the robot status publisher."""
    # TODO: Initialize rclpy
    # TODO: Create the robot status publisher node
    # TODO: Spin the node
    # TODO: Shutdown rclpy when done
    pass


if __name__ == '__main__':
    main()
```

**Deliverable**: A working ROS 2 publisher node that broadcasts robot status messages at regular intervals.

**Success Criteria**:
- [ ] ROS 2 workspace created successfully
- [ ] Custom package with publisher node created
- [ ] Node publishes messages to 'robot_status' topic
- [ ] Messages can be verified using `ros2 topic echo`
- [ ] Node runs without errors

**Test Commands**:
```bash
# Create a new workspace
mkdir -p ~/ros2_architecture_ws/src
cd ~/ros2_architecture_ws

# Create a new package
cd src
ros2 pkg create --build-type ament_python robot_status_pkg --dependencies rclpy std_msgs

# Navigate to the package directory
cd robot_status_pkg
mkdir robot_status_pkg
touch robot_status_pkg/__init__.py

# Copy your publisher code to robot_status_pkg/robot_status_publisher.py

# Build the workspace
cd ~/ros2_architecture_ws
colcon build --packages-select robot_status_pkg

# Source the workspace
source install/setup.bash

# Run the node
ros2 run robot_status_pkg robot_status_publisher

# In another terminal, verify the topic is being published
ros2 topic echo /robot_status
```

**Expected Output**:
The publisher node should continuously publish "Robot operational" messages to the `/robot_status` topic, which can be verified with `ros2 topic echo`.

**Challenge**: Modify the publisher to include timestamp information in the status message and implement a corresponding subscriber node.

**Hints**:
<details>
<summary>Click for hint 1</summary>

Make sure to import the required modules: `rclpy` and `std_msgs.msg.String`
</details>

<details>
<summary>Click for hint 2</summary>

Use `self.create_publisher()` to create a publisher and `self.create_timer()` to create a periodic callback.
</details>
:::

:::tip Exercise 1.1.2: QoS Configuration and Testing

**Objective**: Understand and experiment with Quality of Service (QoS) profiles in ROS 2.

**Difficulty**: ⭐⭐⭐ Hard

**Time Estimate**: 35-45 minutes

**Requirements**:
1. Create a publisher with different QoS profiles (reliable vs. best effort)
2. Create a corresponding subscriber to test different QoS configurations
3. Compare performance and reliability under different QoS settings
4. Analyze when to use different QoS profiles for humanoid robotics

**Starter Code**:
```python title="qos_publisher.py"
#!/usr/bin/env python3
"""
ROS 2 QoS Exercise - Publisher with Different QoS Profiles

This script implements a publisher with configurable QoS settings.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String


class QoSPublisher(Node):
    """A publisher that demonstrates different QoS configurations."""

    def __init__(self, qos_profile_name):
        """Initialize the QoS publisher with a specific profile."""
        super().__init__('qos_publisher')

        # Define different QoS profiles
        if qos_profile_name == 'reliable':
            # TODO: Create a reliable QoS profile
            pass
        elif qos_profile_name == 'best_effort':
            # TODO: Create a best effort QoS profile
            pass
        elif qos_profile_name == 'transient':
            # TODO: Create a transient local QoS profile
            pass
        else:
            # Default to reliable
            pass

        # TODO: Create publisher with the selected QoS profile
        # TODO: Create a timer to publish messages
        # TODO: Track message count for testing

    def timer_callback(self):
        """Callback to publish messages with QoS testing."""
        # TODO: Publish message with current count
        # TODO: Log the message being published
        pass


def main(args=None, qos_profile='reliable'):
    """Main function to run the QoS publisher."""
    rclpy.init(args=args)
    qos_publisher = QoSPublisher(qos_profile)
    rclpy.spin(qos_publisher)
    qos_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    import sys
    qos_profile = sys.argv[1] if len(sys.argv) > 1 else 'reliable'
    main(qos_profile=qos_profile)
```

```python title="qos_subscriber.py"
#!/usr/bin/env python3
"""
ROS 2 QoS Exercise - Subscriber with Different QoS Profiles

This script implements a subscriber that works with different QoS settings.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String


class QoSSubscriber(Node):
    """A subscriber that demonstrates different QoS configurations."""

    def __init__(self, qos_profile_name):
        """Initialize the QoS subscriber with a specific profile."""
        super().__init__('qos_subscriber')

        # Define different QoS profiles to match the publisher
        # TODO: Implement QoS profile selection based on parameter
        # TODO: Create subscription with the selected QoS profile
        # TODO: Initialize message tracking variables

    def message_callback(self, msg):
        """Callback to handle incoming messages."""
        # TODO: Process the received message
        # TODO: Track message statistics
        # TODO: Log message information
        pass


def main(args=None, qos_profile='reliable'):
    """Main function to run the QoS subscriber."""
    rclpy.init(args=args)
    qos_subscriber = QoSSubscriber(qos_profile)
    rclpy.spin(qos_subscriber)
    qos_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    import sys
    qos_profile = sys.argv[1] if len(sys.argv) > 1 else 'reliable'
    main(qos_profile=qos_profile)
```

**Deliverable**: A complete publisher-subscriber pair that demonstrates different QoS configurations and their effects.

**Success Criteria**:
- [ ] Publisher and subscriber work with reliable QoS profile
- [ ] Publisher and subscriber work with best effort QoS profile
- [ ] Performance differences are documented and analyzed
- [ ] Understanding of when to use each QoS profile is demonstrated
- [ ] Code handles different QoS configurations properly

**Test Commands**:
```bash
# Build the workspace with QoS package
cd ~/ros2_architecture_ws
ros2 pkg create --build-type ament_python qos_test_pkg --dependencies rclpy std_msgs

# Add the QoS publisher and subscriber code to the package
# Build the workspace
colcon build --packages-select qos_test_pkg
source install/setup.bash

# Test reliable QoS
# Terminal 1: ros2 run qos_test_pkg qos_publisher reliable
# Terminal 2: ros2 run qos_test_pkg qos_subscriber reliable

# Test best effort QoS
# Terminal 1: ros2 run qos_test_pkg qos_publisher best_effort
# Terminal 2: ros2 run qos_test_pkg qos_subscriber best_effort

# Monitor performance differences
ros2 topic hz /qos_test_topic
```

**Expected Output**:
Different QoS profiles should show different performance characteristics, with reliable QoS ensuring all messages are received but potentially with higher latency.

**Challenge**: Implement a QoS analyzer that automatically tests multiple QoS combinations and reports performance metrics.

**Hints**:
<details>
<summary>Click for hint 1</summary>

Match the QoS profiles between publisher and subscriber for best results.
</details>

<details>
<summary>Click for hint 2</summary>

Use `QoSHistoryPolicy.KEEP_LAST` with a specific depth to limit memory usage.
</details>
:::

:::tip Exercise 1.1.3: Node Communication Analysis

**Objective**: Analyze and visualize the communication patterns between ROS 2 nodes.

**Difficulty**: ⭐⭐ Medium

**Time Estimate**: 30-40 minutes

**Requirements**:
1. Create multiple nodes with different communication patterns (publisher, subscriber, service client/server)
2. Use ROS 2 tools to analyze the network topology
3. Document the communication flows and dependencies
4. Create a visualization of the node graph

**Starter Code**:
```python title="communication_analysis.py"
#!/usr/bin/env python3
"""
ROS 2 Architecture Exercise - Communication Analysis

This script creates multiple nodes to demonstrate different communication patterns.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from example_interfaces.srv import AddTwoInts


class CommunicationAnalyzer(Node):
    """A node that demonstrates various ROS 2 communication patterns."""

    def __init__(self):
        """Initialize the communication analyzer node."""
        super().__init__('communication_analyzer')

        # TODO: Create publishers for different topics
        # TODO: Create subscribers for monitoring
        # TODO: Create service server
        # TODO: Create service client
        # TODO: Create timer for periodic analysis

        self.get_logger().info('Communication Analyzer node initialized')

    def timer_callback(self):
        """Periodic callback for analysis."""
        # TODO: Publish periodic status messages
        # TODO: Call services periodically
        # TODO: Log communication statistics
        pass

    def add_service_callback(self, request, response):
        """Service callback for adding two integers."""
        # TODO: Implement service logic
        # TODO: Log service calls
        return response


def main(args=None):
    """Main function to run the communication analyzer."""
    rclpy.init(args=args)
    comm_analyzer = CommunicationAnalyzer()

    # TODO: Create service client and call service
    # TODO: Implement the analysis logic

    rclpy.spin(comm_analyzer)
    comm_analyzer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Deliverable**: A comprehensive analysis of ROS 2 communication patterns with visualization and documentation.

**Success Criteria**:
- [ ] Multiple nodes with different communication patterns created
- [ ] ROS 2 tools used to analyze network topology
- [ ] Communication flows documented with diagrams
- [ ] Node graph visualization created
- [ ] Understanding of different communication patterns demonstrated

**Test Commands**:
```bash
# Create analysis package
cd ~/ros2_architecture_ws/src
ros2 pkg create --build-type ament_python comm_analysis_pkg --dependencies rclpy std_msgs example_interfaces

# Build and source
cd ~/ros2_architecture_ws
colcon build --packages-select comm_analysis_pkg
source install/setup.bash

# Run the communication analyzer
ros2 run comm_analysis_pkg communication_analysis

# Analyze the network with ROS 2 tools
ros2 node list
ros2 topic list
ros2 service list
ros2 run rqt_graph rqt_graph  # If available
ros2 topic info /topic_name
```

**Expected Output**:
A clear visualization of the communication patterns between nodes, showing publishers, subscribers, services, and their relationships.

**Challenge**: Create an automated tool that generates communication pattern reports for any ROS 2 system.

**Hints**:
<details>
<summary>Click for hint 1</summary>

Use `ros2 node info <node_name>` to get detailed information about node connections.
</details>

<details>
<summary>Click for hint 2</summary>

The `rqt_graph` tool provides a visual representation of the ROS graph if available.
</details>
:::

---
<h2 className="second-heading">
 Common Issues and Debugging
</h2>
<div className="border-line"></div>

:::caution Common Problems

**Problem 1: Node Discovery and Communication Issues**

**Symptoms**:
- Nodes cannot find each other on the same machine
- Topics are not connecting between nodes
- Services are not being discovered
- Error messages about domain ID mismatches

**Cause**: ROS 2 uses DDS for communication which relies on domain IDs and network configuration. If nodes have different domain IDs or network issues, they cannot communicate.

**Solution**:
```bash
# Check and set the ROS domain ID:
echo $ROS_DOMAIN_ID  # Check current domain ID
export ROS_DOMAIN_ID=0  # Set to default domain

# For multi-machine communication:
# Ensure same ROS_DOMAIN_ID on all machines
# Check firewall settings for DDS ports (7400-7500)
# Verify network connectivity with ping
```

**Verification**:
```bash
# Test node discovery:
ros2 node list  # Should show all running nodes
ros2 topic list  # Should show all active topics
ros2 topic info /topic_name  # Check topic connections
```

---

**Problem 2: QoS Profile Mismatch**

**Symptoms**:
- Messages are not being received despite nodes being connected
- Intermittent message delivery
- Performance issues with communication
- Warning messages about incompatible QoS

**Cause**: When publisher and subscriber QoS profiles are incompatible, messages may not be delivered or may be delivered unreliably.

**Solution**:
```python
# Ensure QoS compatibility between publisher and subscriber:
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

# Publisher side - use consistent QoS:
qos_profile = QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE  # Match on both ends
)

publisher = self.create_publisher(String, 'topic_name', qos_profile)

# Subscriber side - match the QoS:
subscription = self.create_subscription(
    String,
    'topic_name',
    callback,
    qos_profile  # Must match publisher
)
```

**Verification**:
```bash
# Check QoS settings for a topic:
ros2 topic info /topic_name -v  # Verbose output shows QoS settings
```

---

**Problem 3: Workspace and Package Setup Issues**

**Symptoms**:
- Import errors when trying to use custom messages/services
- Package not found errors
- Build failures with colcon
- Environment not properly sourced

**Cause**: Incorrect workspace setup, missing dependencies, or environment not properly configured.

**Solution**:
```bash
# Proper workspace setup:
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
source install/setup.bash  # Always source after build

# Check package dependencies:
cd src/your_package
# Ensure package.xml has correct dependencies
rosdep install --from-paths . --ignore-src -r -y

# Verify environment:
printenv | grep ROS  # Check ROS environment variables
```

**Verification**:
```bash
# Test package availability:
ros2 pkg list | grep your_package_name
python3 -c "import your_package"  # Test Python import
```

---

**Problem 4: Memory and Performance Issues with Publishers**

**Symptoms**:
- High memory usage over time
- Slow message publishing
- Node becoming unresponsive
- Dropped messages under load

**Cause**: Improper QoS settings, large message sizes, or inefficient message handling can cause performance issues.

**Solution**:
```python
# Optimize publisher settings:
from rclpy.qos import QoSProfile, QoSHistoryPolicy

# Limit message history to prevent memory buildup:
qos_profile = QoSProfile(
    depth=5,  # Keep only recent messages
    history=QoSHistoryPolicy.KEEP_LAST  # Only keep last N messages
)

# For high-frequency topics, consider best effort:
high_freq_qos = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.BEST_EFFORT  # Accept occasional drops
)
```

**Verification**:
```bash
# Monitor performance:
ros2 topic hz /topic_name  # Check message frequency
ros2 topic bw /topic_name  # Check bandwidth usage
htop  # Monitor process resource usage
```

---

**Problem 5: Lifecycle Management Issues**

**Symptoms**:
- Nodes not shutting down cleanly
- Resource leaks after node termination
- Nodes stuck in certain states
- Unexpected behavior during startup/shutdown

**Cause**: Improper lifecycle management, missing cleanup code, or unhandled exceptions during node destruction.

**Solution**:
```python
# Proper node cleanup:
class ProperNode(Node):
    def __init__(self):
        super().__init__('proper_node')
        # Initialize resources
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        # Handle timer events
        pass

    def destroy_node(self):
        """Properly clean up resources."""
        # Cancel timers
        if self.timer is not None:
            self.timer.cancel()

        # Destroy subscriptions/publishers
        super().destroy_node()
```

**Verification**:
```bash
# Check for zombie processes:
ps aux | grep ros
# Ensure proper cleanup by monitoring resource usage
```
:::

---
<h2 className="second-heading">
Summary
</h2>
<div className="border-line"></div>

In this chapter, you learned:
<div className="border-line"></div>

- ✅ How ROS 2's architecture provides a robust foundation for humanoid robotics applications
- ✅ The key differences between ROS 1 and ROS 2, particularly the DDS-based communication
- ✅ How Quality of Service (QoS) profiles enable configurable communication behavior
- ✅ The core components: nodes, topics, services, and actions
- ✅ How to set up and configure a basic ROS 2 environment
<div className="border-line"></div>

**Key Takeaways**:
<div className="border-line"></div>
- • ROS 2 uses DDS as its communication middleware, providing enhanced reliability and performance
- • Quality of Service (QoS) profiles allow fine-tuning of communication behavior
- • The architecture supports real-time systems, multi-robot coordination, and production deployment
- • Proper environment setup and node management are essential for reliable operation
- • Understanding communication patterns is crucial for building scalable robotic systems

---
<div className="border-line"></div>
<h2 className="second-heading">
 Additional Resources
</h2> 
<div className="border-line"></div>

**Official Documentation**:
- ➝ [ROS 2 Architecture](https://docs.ros.org/en/humble/Concepts/About-ROS-2-Architecture.html)
- ➝ [Quality of Service](https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service.html)

**Tutorials**:
- ➝ [ROS 2 Beginner Python](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
- ➝ [QoS Configuration Guide](https://docs.ros.org/en/humble/Tutorials/Advanced/Quality-Of-Service.html)

**Example Code**:
- ➝ [ROS 2 Demos Repository](https://github.com/ros2/demos)

---

**Navigation**: [← Previous Chapter](../00-introduction/05-syllabus.md) | [Next Chapter →](./02-nodes-topics.md)

</div>

<div className="summary-content">
<div className="border-line"></div>
<h2 className="second-heading">
Chapter Summary
</h2> 
<div className="border-line"></div>
<h2 className="third-heading">
 Module Overview
</h2> 
<div className="border-line"></div>


<h2 className="third-heading">
Key Concepts
</h2> 

- ▸ **DDS Middleware**: Data Distribution Service provides the foundation for ROS 2 communication
- ▸ **Quality of Service**: Configurable policies for reliability, durability, and history
- ▸ **Communication Patterns**: Topics (pub/sub), Services (req/resp), Actions (goal/feedback/result)
- ▸ **Node Architecture**: Separate processes that encapsulate specific functionality
- ▸ **Client Libraries**: rclpy (Python) and rclcpp (C++) for different use cases
<div className="border-line"></div>


<h2 className="third-heading">
Essential Code Pattern
</h2> 
<div className="border-line"></div>

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MyNode(Node):
    def __init__(self):
        super().__init__('node_name')
        self.publisher = self.create_publisher(String, 'topic_name', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello, ROS 2!'
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

<h2 className="third-heading">
Quick Reference
</h2> 
<div className="border-line"></div>

| Component | Purpose | Best Practice |
|-----------|---------|---------------|
| Nodes | Execution units | One process per node, single responsibility |
| Topics | Async communication | Use appropriate QoS for reliability |
| Services | Sync communication | For request/response patterns |
| Actions | Long operations | For tasks with feedback/progress |
| QoS | Communication config | Match publisher/subscriber profiles |

<div className="border-line"></div>
<h2 className="third-heading">
What You Built
</h2> 

- ▹ Basic ROS 2 publisher node
- ▹ Understanding of QoS profiles
- ▹ Communication pattern analysis skills
<div className="border-line"></div>
<h2 className="third-heading">
Next Steps
</h2> 
<div className="border-line"></div>

Continue to [Nodes and Topics](./02-nodes-topics.md) to dive deeper into ROS 2 communication patterns.

</div>