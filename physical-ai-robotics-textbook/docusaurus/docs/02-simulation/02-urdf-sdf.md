---
sidebar_position: 3
title: 'URDF and SDF: Robot and Environment Modeling'
description: 'Understanding URDF for robot description and SDF for simulation environment modeling'
---

<h1 className="main-heading">URDF and SDF: Robot and Environment Modeling</h1>
<div className="underline-class"></div>

Understand URDF for robot modeling and SDF for environment modeling in Gazebo simulation.

<div className="border-line"></div>

<h2 className="second-heading">Learning Objectives</h2>
<div className="underline-class"></div>

- • Understand URDF and SDF differences
- • Create SDF environment models
- • Integrate URDF robots with SDF
- • Use Gazebo extensions
- • Create complex simulation scenarios

<div className="border-line"></div>

<h2 className="second-heading">Exercises</h2>
<div className="underline-class"></div>

<details>
<summary>Exercise 2.2.1: Basic SDF Environment (⭐, ~30 min)</summary>

<h3 className="third-heading">Exercise 2.2.1: Basic SDF Environment</h3>
<div className="underline-class"></div>

**Difficulty**: ⭐ | **Time**: 30 min | **Requirements**: Gazebo, XML

<h4 className="fourth-heading">Starter Code</h4>
<div className="underline-class"></div>

- • Physics configuration
- • Ground plane and lighting
- • Simple box model

<h4 className="fourth-heading">Success Criteria</h4>
<div className="underline-class"></div>

- [ ] SDF file is valid
- [ ] World loads in Gazebo
- [ ] Physics works properly

<h4 className="fourth-heading">Test Commands</h4>
<div className="underline-class"></div>
```bash
gz sdf -k your_world.sdf
gazebo your_world.sdf
```

<h4 className="fourth-heading">Hints</h4>
<div className="underline-class"></div>

- • Use default world as template
- • Validate syntax first

</details>

<details>
<summary>Exercise 2.2.2: URDF-SDF Integration (⭐⭐, ~45 min)</summary>

<h3 className="third-heading">Exercise 2.2.2: URDF-SDF Integration</h3>
<div className="underline-class"></div>

**Difficulty**: ⭐⭐ | **Time**: 45 min

<h4 className="fourth-heading">Starter Code</h4>
<div className="underline-class"></div>

- • URDF robot model
- • SDF world file
- • Gazebo ROS 2 plugins
- • Joint transmissions

<h4 className="fourth-heading">Test Commands</h4>
<div className="underline-class"></div>
```bash
gz sdf -p robot.urdf > robot.sdf
ros2 launch your_robot_gazebo your_simulation.launch.py
ros2 topic list | grep joint
```

<h4 className="fourth-heading">Hints</h4>
<div className="underline-class"></div>

- • Use proper Gazebo plugins
- • Match joint names with controllers

</details>

<details>
<summary>Exercise 2.2.3: Advanced Environment (⭐⭐⭐, ~60 min)</summary>

<h3 className="third-heading">Exercise 2.2.3: Advanced Environment</h3>
<div className="underline-class"></div>

**Difficulty**: ⭐⭐⭐ | **Time**: 60 min

<h4 className="fourth-heading">Starter Code</h4>
<div className="underline-class"></div>

- • Multiple rooms/floors
- • Furniture and obstacles
- • Optimized physics parameters
- • Gazebo sensor extensions

<h4 className="fourth-heading">Test Commands</h4>
<div className="underline-class"></div>
```bash
gz sdf -k complex_environment.sdf
gazebo --verbose complex_environment.sdf
gz topic -e /stats
```

<h4 className="fourth-heading">Hints</h4>
<div className="underline-class"></div>

- • Use static models for environment
- • Optimize collision geometry

</details>

<div className="border-line"></div>

<h2 className="second-heading">Troubleshooting</h2>
<div className="underline-class"></div>

<details>
<summary>Common Issues</summary>

<h3 className="third-heading">Troubleshooting</h3>
<div className="underline-class"></div>

<h4 className="fourth-heading">URDF Conversion Fails</h4>
<div className="underline-class"></div>

**Solutions**:
```bash
find /path -name "*.dae" -o -name "*.stl"
check_urdf robot.urdf
gz sdf -p robot.urdf > robot.sdf
```

<h4 className="fourth-heading">Physics Instability</h4>
<div className="underline-class"></div>

**Solutions**:
```xml
<inertial><mass value="1.0"/><inertia ixx="0.01" iyy="0.01" izz="0.01"/></inertial>
<physics><max_step_size>0.001</max_step_size></physics>
<joint><axis><dynamics><damping>1.0</damping></dynamics></axis></joint>
```

<h4 className="fourth-heading">ROS 2 Integration Fails</h4>
<div className="underline-class"></div>

**Solutions**:
```xml
<plugin name="joint_state_publisher" filename="libgazebo_ros_joint_state_publisher.so">
  <ros><namespace>/robot_name</namespace></ros>
</plugin>
```
```bash
sudo apt install ros-humble-gazebo-ros-pkgs
```

<h4 className="fourth-heading">Slow Performance</h4>
<div className="underline-class"></div>

**Solutions**:
```xml
<collision><geometry><box><size>0.5 0.5 0.5</size></box></geometry></collision>
<sensor><update_rate>30</update_rate></sensor>
<physics><max_step_size>0.01</max_step_size></physics>
```

<h4 className="fourth-heading">Joint Limit Issues</h4>
<div className="underline-class"></div>

**Solutions**:
```xml
<joint type="revolute">
  <limit lower="-1.57" upper="1.57" effort="100" velocity="3.0"/>
</joint>
<transmission name="joint_trans">
  <joint name="joint_name">
    <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
  </joint>
</transmission>
```

</details>

<div className="border-line"></div>

<h2 className="second-heading">URDF vs SDF</h2>
<div className="underline-class"></div>

<h3 className="third-heading">URDF</h3>
<div className="underline-class"></div>

**Strengths**:
- • Robot kinematic chains
- • Joint limits and types
- • ROS integration
- • Parameterization with Xacro

<h3 className="third-heading">SDF</h3>
<div className="underline-class"></div>

**Strengths**:
- • Complete simulation environments
- • Physics properties
- • Gazebo plugins and sensors
- • Multiple model inclusion

<div className="border-line"></div>

<h2 className="second-heading">Converting URDF to SDF</h2>
<div className="underline-class"></div>

<h3 className="third-heading">Direct URDF in Gazebo</h3>
<div className="underline-class"></div>
```xml
<world name="default">
  <include><uri>model://ground_plane</uri></include>
  <include><uri>file://$(find robot)/urdf/humanoid.urdf</uri></include>
</world>
```

<h3 className="third-heading">Wrapping URDF</h3>
<div className="underline-class"></div>
```xml
<model name="humanoid_robot">
  <include><uri>model://humanoid_robot_model</uri></include>
  <plugin filename="libgazebo_ros_control.so"/>
</model>
```

<div className="border-line"></div>

<h2 className="second-heading">SDF Environment</h2>
<div className="underline-class"></div>

<h3 className="third-heading">World Structure</h3>
<div className="underline-class"></div>
```xml
<world name="my_world">
  <physics type="ode">
    <max_step_size>0.001</max_step_size>
    <gravity>0 0 -9.8</gravity>
  </physics>
  <light type="directional"><direction>-0.4 0.2 -0.9</direction></light>
  <include><uri>model://ground_plane</uri></include>
</world>
```

<h3 className="third-heading">Custom Models</h3>
<div className="underline-class"></div>
```xml
<model name="room">
  <link name="floor">
    <collision><geometry><box><size>10 10 0.1</size></box></geometry></collision>
    <visual><geometry><box><size>10 10 0.1</size></box></geometry></visual>
  </link>
  <link name="wall">
    <pose>0 5 1.5 0 0 0</pose>
    <collision><geometry><box><size>10 0.1 3</size></box></geometry></collision>
  </link>
</model>
```

<div className="border-line"></div>

<h2 className="second-heading">Advanced Features</h2>
<div className="underline-class"></div>

<h3 className="third-heading">Joint Transmissions</h3>
<div className="underline-class"></div>
```xml
<joint name="elbow_joint" type="revolute">
  <axis><xyz>1 0 0</xyz>
    <limit><lower>-2.0</lower><upper>1.0</upper></limit>
  </axis>
</joint>
<transmission name="elbow_trans">
  <joint name="elbow_joint">
    <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
  </joint>
</transmission>
```

<h3 className="third-heading">Gazebo Extensions</h3>
<div className="underline-class"></div>
```xml
<gazebo>
  <plugin name="joint_state_publisher" filename="libgazebo_ros_joint_state_publisher.so">
    <update_rate>30</update_rate>
  </plugin>
</gazebo>
<gazebo reference="foot">
  <mu1>0.8</mu1><mu2>0.8</mu2>
</gazebo>
```

<div className="border-line"></div>

<h2 className="second-heading">Physics for Humanoids</h2>
<div className="underline-class"></div>

<h3 className="third-heading">Physics Parameters</h3>
<div className="underline-class"></div>
```xml
<physics type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_update_rate>1000</real_time_update_rate>
  <ode>
    <solver><type>quick</type><iters>20</iters></solver>
    <constraints><cfm>1e-5</cfm><erp>0.2</erp></constraints>
  </ode>
</physics>
```

<h3 className="third-heading">Joint Damping</h3>
<div className="underline-class"></div>
```xml
<joint name="knee_joint" type="revolute">
  <axis>
    <limit><lower>0</lower><upper>2.5</upper></limit>
    <dynamics><damping>1.0</damping><friction>0.1</friction></dynamics>
  </axis>
</joint>
```

<div className="border-line"></div>

<h2 className="second-heading">Best Practices</h2>
<div className="underline-class"></div>

- • Use descriptive names
- • Simplify collision geometry
- • Use static models for environment
- • Test URDF-SDF conversion
- • Document complex models

<h2 className="second-heading">ROS 2 Launch Integration</h2>
<div className="underline-class"></div>
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('gazebo_ros'),
            '/launch/gazebo.launch.py'
        ]),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    )
    return LaunchDescription([gazebo_launch])
```

<h2 className="second-heading">Summary</h2>
<div className="underline-class"></div>

URDF describes robots, SDF describes complete simulation environments. Use Gazebo extensions to bridge both formats for realistic humanoid robot simulation.