---
sidebar_position: 6
title: 'ROS 2 Integration: Simulation to Real Robotics'
description: 'Integrating simulation environments with ROS 2 for comprehensive robotics development'
---

import ReadingTime from '@site/src/components/ReadingTime';
import ViewToggle from '@site/src/components/ViewToggle';

<ReadingTime minutes={18} />
<!-- <ViewToggle /> -->

<h1 className="main-heading">ROS 2 Integration: Connecting Simulation to Real Robotics</h1>

<div className="underline-class"></div>

<h2 className="second-heading">Learning Objectives</h2>

<div className="border-line"></div>

By the end of this chapter, you will be able to:
- • Integrate simulation environments with ROS 2 for bidirectional communication
- • Implement sensor and actuator bridges between simulation and ROS 2
- • Use ROS 2 tools for simulation control and monitoring
- • Design simulation workflows mirroring real-world operations
- • Validate simulation-to-reality transfer for humanoid robots

<div className="border-line"></div>

<h2 className="second-heading">Exercises</h2>

<div className="border-line"></div>

<details>
<summary>Exercise 2.5.1: Basic ROS 2-Simulation Bridge (⭐, ~25 min)</summary>

<h3 className="third-heading">Exercise 2.5.1: Basic ROS 2-Simulation Bridge Setup</h3>

<div className="border-line"></div>

**Difficulty**: ⭐ | **Time**: 25 minutes

<h4 className="fourth-heading">Task</h4>

<div className="border-line"></div>

Create robot model with ROS 2 integration plugins

<h4 className="fourth-heading">Test Commands</h4>

<div className="border-line"></div>

```bash
# Build and launch
gz sdf -k your_robot.model.sdf
gz sim -r your_world.sdf

# Check topics
ros2 topic list | grep my_robot
ros2 topic echo /my_robot/camera/image_raw

# Send commands
ros2 topic pub /my_robot/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}}'
```

<h4 className="fourth-heading">Success Criteria</h4>

<div className="border-line"></div>

- [ ] Robot loads with ROS 2 plugins
- [ ] Camera data publishes
- [ ] IMU data publishes
- [ ] Joint states publish
- [ ] Robot responds to commands

</details>

<details>
<summary>Exercise 2.5.2: Advanced Sensor Processing (⭐⭐, ~45 min)</summary>

<h3 className="third-heading">Exercise 2.5.2: Advanced Sensor Integration</h3>

<div className="border-line"></div>

**Difficulty**: ⭐⭐ | **Time**: 45 minutes

<h4 className="fourth-heading">Task</h4>

<div className="border-line"></div>

Create complete sensor processing pipeline

<h4 className="fourth-heading">Test Commands</h4>

<div className="border-line"></div>

```bash
# Launch pipeline
ros2 launch my_robot_perception sensor_processing.launch.py

# Test processing
ros2 topic echo /my_robot/camera/processed_image
ros2 topic echo /my_robot/obstacles

# Visualize
ros2 run rviz2 rviz2 -d config/sensor_fusion.rviz
```

<h4 className="fourth-heading">Success Criteria</h4>

<div className="border-line"></div>

- [ ] Object detection works
- [ ] Obstacle detection identifies objects
- [ ] IMU data integrated
- [ ] Sensor fusion consistent
- [ ] Data visualized in RViz2

</details>

<details>
<summary>Exercise 2.5.3: Multi-Robot Coordination (⭐⭐⭐, ~60 min)</summary>

<h3 className="third-heading">Exercise 2.5.3: Multi-Robot Coordination</h3>

<div className="border-line"></div>

**Difficulty**: ⭐⭐⭐ | **Time**: 60 minutes

<h4 className="fourth-heading">Task</h4>

<div className="border-line"></div>

Implement multi-robot simulation with coordination

<h4 className="fourth-heading">Test Commands</h4>

<div className="border-line"></div>

```bash
# Launch multi-robot
ros2 launch my_robot_multi simulation.launch.py

# Check robots
ros2 topic list | grep robot_
ros2 topic echo /multi_robot/coordinator/status

# Send commands
ros2 topic pub /robot_0/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}}'
```

<h4 className="fourth-heading">Success Criteria</h4>

<div className="border-line"></div>

- [ ] Multiple robots operate
- [ ] Coordination prevents collisions
- [ ] Tasks distributed properly
- [ ] Communication functional
- [ ] Performance stable

</details>

<div className="border-line"></div>

<h2 className="second-heading">Troubleshooting</h2>

<div className="border-line"></div>

<details>
<summary>Troubleshooting: ROS 2 Integration Issues</summary>

<h3 className="third-heading">Troubleshooting: ROS 2 Integration Issues</h3>

<div className="border-line"></div>

<h4 className="fourth-heading">Problem: Nodes cannot connect to simulation</h4>

<div className="border-line"></div>

**Symptoms**:
- • No sensor data published
- • Robot not responding
- • Topics not appearing

<div className="border-line"></div>

**Solutions**:
```xml
<!-- Correct plugin config -->
<plugin name="camera_controller" filename="libgazebo_ros_camera.so">
  <ros>
    <namespace>/my_robot</namespace>
    <remapping>~/image_raw:=/camera/image_raw</remapping>
  </ros>
</plugin>
```

```bash
# Verify
ros2 topic list
sudo apt install ros-humble-gazebo-ros-pkgs
```

<div className="border-line"></div>

<h4 className="fourth-heading">Problem: Sensor data incorrect</h4>

<div className="border-line"></div>

**Symptoms**:
- • No data in topics
- • Values out of range
- • High latency

<div className="border-line"></div>

**Solutions**:
```xml
<!-- Proper sensor config -->
<sensor name="camera" type="camera">
  <update_rate>30</update_rate>
  <always_on>true</always_on>
</sensor>
```

```bash
# Check frames
ros2 run tf2_tools view_frames
ros2 topic echo /my_robot/camera/image_raw
```

<div className="border-line"></div>

<h4 className="fourth-heading">Problem: Performance issues</h4>

<div className="border-line"></div>

**Symptoms**:
- • Low real-time factor
- • High CPU usage
- • Queue overflow warnings

<div className="border-line"></div>

**Solutions**:
```python
# Efficient QoS
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

sensor_qos = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.BEST_EFFORT
)
```

```xml
<!-- Reduce update rates -->
<sensor name="camera" type="camera">
  <update_rate>15</update_rate>
</sensor>
```

<div className="border-line"></div>

<h4 className="fourth-heading">Problem: Time synchronization issues</h4>

<div className="border-line"></div>

**Symptoms**:
- • Inconsistent timestamps
- • Delayed TF transformations

<div className="border-line"></div>

**Solutions**:
```python
# In all nodes
self.declare_parameter('use_sim_time', True)

# In launch files
Node(
    parameters=[{'use_sim_time': True}]
)
```

</details>

<div className="border-line"></div>

<h2 className="second-heading">ROS 2-Simulation Integration</h2>

<div className="border-line"></div>

<h3 className="third-heading">Integration Pipeline</h3>

<div className="border-line"></div>

ROS 2 simulation integration creates a closed loop:
- • Simulation provides realistic sensor data
- • ROS 2 nodes process with real algorithms
- • Control commands sent to simulated robots
- • Simulation responds to commands

Benefits:
- • Algorithm development without hardware
- • Safe environment testing
- • Control system validation
- • Parallel hardware/software development

<div className="border-line"></div>

<h3 className="third-heading">Gazebo-ROS 2 Bridge</h3>

<div className="border-line"></div>

```xml
<!-- Robot with ROS 2 plugins -->
<model name="ros2_robot">
  <sensor name="camera" type="camera">
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/my_robot</namespace>
        <remapping>~/image_raw:=/camera/image_raw</remapping>
      </ros>
    </plugin>
  </sensor>

  <sensor name="imu" type="imu">
    <plugin name="imu_controller" filename="libgazebo_ros_imu.so">
      <ros>
        <namespace>/my_robot</namespace>
      </ros>
    </plugin>
  </sensor>

  <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
    <ros>
      <namespace>/my_robot</namespace>
    </ros>
    <command_topic>cmd_vel</command_topic>
    <odometry_topic>odom</odometry_topic>
  </plugin>
</model>
```

<div className="border-line"></div>

<h2 className="second-heading">Sensor Integration</h2>

<div className="border-line"></div>

<h3 className="third-heading">Camera Integration</h3>

<div className="border-line"></div>

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraProcessor(Node):
    def __init__(self):
        super().__init__('camera_processor')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image, '/my_robot/camera/image_raw', self.callback, 10)
        self.pub = self.create_publisher(
            Image, '/my_robot/camera/processed', 10)
    
    def callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        edges = cv2.Canny(cv_image, 50, 150)
        self.pub.publish(self.bridge.cv2_to_imgmsg(edges, 'mono8'))
```

<div className="border-line"></div>

<h3 className="third-heading">LIDAR Integration</h3>

<div className="border-line"></div>

```python
from sensor_msgs.msg import LaserScan
import numpy as np

class LIDARProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor')
        self.sub = self.create_subscription(
            LaserScan, '/my_robot/scan', self.callback, 10)
    
    def callback(self, msg):
        ranges = np.array(msg.ranges)
        valid = ranges[(ranges >= msg.range_min) & 
                      (ranges <= msg.range_max)]
        min_dist = np.min(valid) if len(valid) > 0 else float('inf')
        self.get_logger().info(f'Min distance: {min_dist:.2f}m')
```

<div className="border-line"></div>

<h3 className="third-heading">IMU Integration</h3>

<div className="border-line"></div>

```python
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import math

class IMUProcessor(Node):
    def __init__(self):
        super().__init__('imu_processor')
        self.sub = self.create_subscription(
            Imu, '/my_robot/imu/data', self.callback, 10)
        self.pub = self.create_publisher(Vector3, '/orientation', 10)
    
    def callback(self, msg):
        q = msg.orientation
        roll, pitch, yaw = self.quat_to_euler(q.x, q.y, q.z, q.w)
        
        orientation = Vector3()
        orientation.x = roll
        orientation.y = pitch
        orientation.z = yaw
        self.pub.publish(orientation)
```

<div className="border-line"></div>

<h2 className="second-heading">Actuator Integration</h2>

<div className="border-line"></div>

<h3 className="third-heading">Joint Control</h3>

<div className="border-line"></div>

```python
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math

class JointController(Node):
    def __init__(self):
        super().__init__('joint_controller')
        self.pub = self.create_publisher(
            JointTrajectory, '/joint_trajectory', 10)
        self.timer = self.create_timer(0.1, self.control_loop)
        self.time = 0.0
    
    def control_loop(self):
        traj = JointTrajectory()
        traj.joint_names = ['hip', 'knee']
        
        point = JointTrajectoryPoint()
        point.positions = [0.1 * math.sin(self.time),
                          0.2 * math.sin(self.time * 2)]
        traj.points = [point]
        
        self.pub.publish(traj)
        self.time += 0.1
```

<div className="border-line"></div>

<h3 className="third-heading">Navigation Integration</h3>

<div className="border-line"></div>

```python
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class NavigationController(Node):
    def __init__(self):
        super().__init__('nav_controller')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.target = (5.0, 5.0)
    
    def odom_callback(self, msg):
        dx = self.target[0] - msg.pose.pose.position.x
        dy = self.target[1] - msg.pose.pose.position.y
        distance = math.sqrt(dx*dx + dy*dy)
        
        cmd = Twist()
        cmd.linear.x = min(1.0, distance * 0.5)
        self.cmd_pub.publish(cmd)
```

<div className="border-line"></div>

<h2 className="second-heading">Launch Files</h2>

<div className="border-line"></div>

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='my_robot_perception',
            executable='camera_processor',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='my_robot_control',
            executable='navigation_controller',
            parameters=[{'use_sim_time': True}]
        )
    ])
```

<div className="border-line"></div>

<h2 className="second-heading">Best Practices</h2>

<div className="border-line"></div>

<h3 className="third-heading">Performance Optimization</h3>

<div className="border-line"></div>

- • **Topic rate management**: Control update rates
- • **Data filtering**: Publish essential data only
- • **QoS configuration**: Use appropriate QoS settings
- • **Resource allocation**: Monitor CPU/memory usage

<div className="border-line"></div>

<h3 className="third-heading">Data Consistency</h3>

<div className="border-line"></div>

- • **Time synchronization**: Use simulation time
- • **Frame consistency**: Maintain TF tree
- • **Data validation**: Validate sensor data
- • **Error handling**: Robust error handling

<div className="border-line"></div>

<h2 className="second-heading">Common Issues</h2>

<div className="border-line"></div>

**Connection problems**
- • Verify Gazebo-ROS plugins loaded
- • Check namespace configurations
- • Confirm DDS domain settings

**Sensor data not publishing**
- • Check plugin configuration
- • Verify topic names
- • Confirm sensor models defined

**Performance issues**
- • Reduce sensor update rates
- • Limit active sensors
- • Optimize QoS settings

**High CPU usage**
- • Monitor node execution
- • Reduce message publications
- • Use multi-threaded executors

<div className="border-line"></div>

<h2 className="second-heading">Summary</h2>

<div className="border-line"></div>

ROS 2 integration with simulation creates powerful development platforms for humanoid robotics. Properly connecting sensors, actuators, and control systems establishes comprehensive testing environments bridging simulation and reality. Success requires robust communication patterns, performance optimization, and maintaining consistency between virtual and real-world behaviors.