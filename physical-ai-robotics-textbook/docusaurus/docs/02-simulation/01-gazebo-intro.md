---
sidebar_position: 2
title: 'Gazebo Introduction: Simulation for Robotics'
description: 'Getting started with Gazebo simulation environment for robotics development'
---

import ReadingTime from '@site/src/components/ReadingTime';

<ReadingTime minutes={4} />

<h1 className="main-heading">Gazebo Introduction: Simulation for Robotics</h1>
<div className="underline-class"></div>

Gazebo is a powerful 3D simulation environment for robotics with realistic physics, graphics, and ROS 2 integration.

<div className="border-line"></div>

<h2 className="second-heading">Learning Objectives</h2>
<div className="underline-class"></div>

- • Understand Gazebo architecture
- • Install and configure Gazebo
- • Create simulation worlds
- • Integrate with ROS 2
- • Control simulated robots

<div className="border-line"></div>

<h2 className="second-heading">Exercises</h2>
<div className="underline-class"></div>

<details>
<summary>Exercise 2.1.1: Installation & Basic Simulation (⭐, ~30 min)</summary>

<h3 className="third-heading">Exercise 2.1.1: Installation & Basic Simulation</h3>
<div className="underline-class"></div>

**Difficulty**: ⭐ | **Time**: 30 min | **Requirements**: Ubuntu, ROS 2

<h4 className="fourth-heading">Starter Code</h4>
<div className="underline-class"></div>

- • Install Gazebo Garden
- • Launch default world
- • Explore interface

<h4 className="fourth-heading">Success Criteria</h4>
<div className="underline-class"></div>

- [ ] Gazebo launches successfully
- [ ] Camera controls work
- [ ] Simulation runs smoothly

<h4 className="fourth-heading">Test Commands</h4>
<div className="underline-class"></div>
```bash
sudo apt update
sudo apt install gazebo-garden ros-humble-gazebo-ros-pkgs
gazebo
```

<h4 className="fourth-heading">Hints</h4>
<div className="underline-class"></div>

- • Check graphics drivers
- • Verify system requirements

</details>

<details>
<summary>Exercise 2.1.2: Custom World Creation (⭐⭐, ~45 min)</summary>

<h3 className="third-heading">Exercise 2.1.2: Custom World Creation</h3>
<div className="underline-class"></div>

**Difficulty**: ⭐⭐ | **Time**: 45 min

<h4 className="fourth-heading">Starter Code</h4>
<div className="underline-class"></div>

- • Ground plane and lighting
- • Multiple objects
- • Robot model
- • Physics parameters

<h4 className="fourth-heading">Test Commands</h4>
<div className="underline-class"></div>
```bash
gz sdf -k your_world.sdf
gazebo your_world.sdf
```

<h4 className="fourth-heading">Hints</h4>
<div className="underline-class"></div>

- • Start with default template
- • Validate SDF before testing

</details>

<details>
<summary>Exercise 2.1.3: ROS 2 Integration (⭐⭐⭐, ~60 min)</summary>

<h3 className="third-heading">Exercise 2.1.3: ROS 2 Integration</h3>
<div className="underline-class"></div>

**Difficulty**: ⭐⭐⭐ | **Time**: 60 min

<h4 className="fourth-heading">Starter Code</h4>
<div className="underline-class"></div>

- • URDF robot model
- • Launch file
- • Robot/Joint state publishers
- • Controller node

<h4 className="fourth-heading">Test Commands</h4>
<div className="underline-class"></div>
```bash
ros2 launch your_robot_gazebo your_robot.launch.py
ros2 topic list
```

<h4 className="fourth-heading">Hints</h4>
<div className="underline-class"></div>

- • Use robot_state_publisher
- • Test components individually

</details>

<div className="border-line"></div>

<h2 className="second-heading">Troubleshooting</h2>
<div className="underline-class"></div>

<details>
<summary>Common Issues</summary>

<h3 className="third-heading">Troubleshooting</h3>
<div className="underline-class"></div>

<h4 className="fourth-heading">Gazebo Fails to Start</h4>
<div className="underline-class"></div>

**Solutions**:
```bash
glxinfo | grep "OpenGL version"
sudo apt install mesa-utils
export LIBGL_ALWAYS_SOFTWARE=1
```

<h4 className="fourth-heading">Robot Falls Through Ground</h4>
<div className="underline-class"></div>

**Solutions**:
```xml
<inertial><mass value="1.0"/>
<inertia ixx="0.01" iyy="0.01" izz="0.01"/></inertial>
```

<h4 className="fourth-heading">Slow Simulation</h4>
<div className="underline-class"></div>

**Solutions**:
```xml
<sensor><update_rate>30</update_rate></sensor>
<collision><geometry><box><size>0.5 0.5 0.5</size></box></geometry></collision>
<physics><max_step_size>0.01</max_step_size></physics>
```

<h4 className="fourth-heading">ROS 2 Integration Fails</h4>
<div className="underline-class"></div>

**Solutions**:
```xml
<plugin name="joint_state_publisher" filename="libgazebo_ros_joint_state_publisher.so"/>
```
```bash
sudo apt install ros-humble-gazebo-ros-pkgs
```

<h4 className="fourth-heading">Missing Models/Textures</h4>
<div className="underline-class"></div>

**Solutions**:
```bash
echo $GAZEBO_MODEL_PATH
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/path/to/models
gz sdf -k model.sdf
```

</details>

<div className="border-line"></div>

<h2 className="second-heading">What is Gazebo?</h2>
<div className="underline-class"></div>

3D dynamic simulator for robots with:
- • **Realistic Physics**: ODE, Bullet, DART
- • **Graphics**: OGRE rendering
- • **Sensors**: Camera, LIDAR, IMU, GPS
- • **Plugins**: Extensible architecture
- • **ROS Integration**: Seamless ROS 2 support

<h3 className="third-heading">Key Features</h3>
<div className="underline-class"></div>

- • Complex kinematics support
- • Realistic physics and balance
- • Sensor simulation
- • Advanced collision detection
- • Complex environments

<div className="border-line"></div>

<h2 className="second-heading">Installation</h2>
<div className="underline-class"></div>
```bash
sudo apt update
sudo apt install gazebo-garden ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control
```

<div className="border-line"></div>

<h2 className="second-heading">Basic Concepts</h2>
<div className="underline-class"></div>

<h3 className="third-heading">World Files</h3>
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

<h3 className="third-heading">Model Files</h3>
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

<h2 className="second-heading">Running Gazebo</h2>
<div className="underline-class"></div>
```bash
gazebo
gazebo /path/to/world.world
ros2 launch gazebo_ros gazebo.launch.py
ros2 launch gazebo_ros gazebo.launch.py world:=/path/to/world.sdf
```

<h3 className="third-heading">GUI Controls</h3>
<div className="underline-class"></div>

- • **Orbit**: Right-click + drag
- • **Pan**: Shift + right-click + drag
- • **Zoom**: Scroll wheel
- • **Focus**: Double-click object

<div className="border-line"></div>

<h2 className="second-heading">ROS 2 Integration</h2>
<div className="underline-class"></div>

<h3 className="third-heading">Spawning Robots</h3>
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

<h3 className="third-heading">Gazebo Plugins</h3>
<div className="underline-class"></div>
```xml
<!-- Joint control -->
<plugin name="joint_state_publisher" filename="libgazebo_ros_joint_state_publisher.so">
  <update_rate>30</update_rate>
</plugin>

<!-- Camera -->
<plugin name="camera_controller" filename="libgazebo_ros_camera.so">
  <ros><namespace>/robot</namespace></ros>
</plugin>

<!-- IMU -->
<plugin name="imu_plugin" filename="libgazebo_ros_imu.so">
  <ros><remapping>~/out:=/imu/data</remapping></ros>
</plugin>
```

<div className="border-line"></div>

<h2 className="second-heading">Best Practices</h2>
<div className="underline-class"></div>

<h3 className="third-heading">Performance</h3>
<div className="underline-class"></div>

- • Reduce sensor update rates
- • Simplify collision geometry
- • Limit physics steps
- • Appropriate world size

<h3 className="third-heading">Realism</h3>
<div className="underline-class"></div>

- • Accurate inertial properties
- • Proper joint limits
- • Realistic sensor noise
- • Correct friction values

<h3 className="third-heading">Debugging</h3>
<div className="underline-class"></div>

- • Wireframe mode (Press 'W')
- • Enable contact visualization
- • Monitor real-time factor
- • Use built-in tools

<div className="border-line"></div>

<h2 className="second-heading">Summary</h2>
<div className="underline-class"></div>

Gazebo provides powerful simulation with accurate physics, realistic sensors, and ROS 2 integration for robotics development before hardware deployment.

<h2 className="second-heading">Resources</h2>
<div className="underline-class"></div>

- • [Gazebo Documentation](http://gazebosim.org/)
- • [ROS 2 Gazebo](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Gazebo.html)
- • [Gazebo Tutorials](http://gazebosim.org/tutorials)