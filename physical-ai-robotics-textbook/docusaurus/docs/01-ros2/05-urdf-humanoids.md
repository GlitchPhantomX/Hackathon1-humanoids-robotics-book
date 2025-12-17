---
sidebar_position: 6
title: 'URDF for Humanoid Robots: Robot Description and Modeling'
description: 'Understanding URDF (Unified Robot Description Format) for modeling humanoid robots in ROS 2'
---

import ReadingTime from '@site/src/components/ReadingTime';

<ReadingTime minutes={6} />

<h1 className="main-heading">URDF for Humanoid Robots: Robot Description and Modeling</h1>
<div className="underline-class"></div>

URDF is the standard format for describing robot models in ROS, defining kinematic structure, joints, and physical properties.

<div className="border-line"></div>

<h2 className="second-heading">Learning Objectives</h2>
<div className="underline-class"></div>

- • Create URDF models for humanoid robots
- • Define links, joints, and materials
- • Implement kinematic chains
- • Use Xacro to simplify models
- • Validate and visualize in RViz

<div className="border-line"></div>

<h2 className="second-heading">Exercises</h2>
<div className="underline-class"></div>

<details>
<summary>Exercise 1.5.1: Basic Humanoid URDF (⭐, ~30 min)</summary>

<h3 className="third-heading">Exercise 1.5.1: Basic Humanoid URDF</h3>
<div className="underline-class"></div>

**Difficulty**: ⭐ | **Time**: 30 min

<h4 className="fourth-heading">Starter Code</h4>
<div className="underline-class"></div>

- • Torso as base
- • Head, arms, legs
- • Proper joints

<h4 className="fourth-heading">Success Criteria</h4>
<div className="underline-class"></div>

- [ ] Valid XML
- [ ] Visual/collision/inertial properties
- [ ] Displays in RViz

<h4 className="fourth-heading">Test Commands</h4>
<div className="underline-class"></div>
```bash
check_urdf basic_humanoid.urdf
ros2 launch rviz2 rviz2
```

<h4 className="fourth-heading">Hints</h4>
<div className="underline-class"></div>

- • Start simple, add incrementally
- • Use appropriate joint types

</details>

<details>
<summary>Exercise 1.5.2: Xacro-based Model (⭐⭐, ~45 min)</summary>

<h3 className="third-heading">Exercise 1.5.2: Xacro-based Model</h3>
<div className="underline-class"></div>

**Difficulty**: ⭐⭐ | **Time**: 45 min

<h4 className="fourth-heading">Starter Code</h4>
<div className="underline-class"></div>

- • Reusable limb macros
- • Torso macro
- • Parameterized joints

<h4 className="fourth-heading">Test Commands</h4>
<div className="underline-class"></div>
```bash
xacro humanoid_robot.xacro > humanoid_robot.urdf
check_urdf humanoid_robot.urdf
```

<h4 className="fourth-heading">Hints</h4>
<div className="underline-class"></div>

- • Use properties for dimensions
- • Test macros individually

</details>

<details>
<summary>Exercise 1.5.3: Advanced Features (⭐⭐⭐, ~60 min)</summary>

<h3 className="third-heading">Exercise 1.5.3: Advanced Features</h3>
<div className="underline-class"></div>

**Difficulty**: ⭐⭐⭐ | **Time**: 60 min

<h4 className="fourth-heading">Starter Code</h4>
<div className="underline-class"></div>

- • Transmission elements
- • Gazebo plugins
- • Realistic inertial properties

<h4 className="fourth-heading">Test Commands</h4>
<div className="underline-class"></div>
```bash
ros2 launch gazebo_ros gazebo
ros2 run tf2_tools view_frames
```

<h4 className="fourth-heading">Hints</h4>
<div className="underline-class"></div>

- • Use realistic inertial values
- • Validate each feature

</details>

<div className="border-line"></div>

<h2 className="second-heading">Troubleshooting</h2>
<div className="underline-class"></div>

<details>
<summary>Common Issues</summary>

<h3 className="third-heading">Troubleshooting</h3>
<div className="underline-class"></div>

<h4 className="fourth-heading">Distorted Model/Incorrect Joints</h4>
<div className="underline-class"></div>

**Solutions**:
```xml
<joint name="joint" type="revolute">
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
</joint>
```
```bash
check_urdf robot.urdf
urdf_to_graphiz robot.urdf
```

<h4 className="fourth-heading">Robot Not in RViz</h4>
<div className="underline-class"></div>

**Solutions**:
```bash
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:=$(cat robot.urdf)
ros2 topic echo /joint_states
ros2 run tf2_tools view_frames
```

<h4 className="fourth-heading">Robot Falls in Simulation</h4>
<div className="underline-class"></div>

**Solutions**:
```xml
<inertial>
  <mass value="1.0"/>
  <origin xyz="0 0 0"/>
  <inertia ixx="0.01" iyy="0.01" izz="0.01"/>
</inertial>
```

<h4 className="fourth-heading">Xacro Compilation Errors</h4>
<div className="underline-class"></div>

**Solutions**:
```xml
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="robot">
<xacro:macro name="macro" params="param1 param2:=default">...</xacro:macro>
<xacro:property name="value" value="1.0"/>
```

</details>

<div className="border-line"></div>

<h2 className="second-heading">URDF Basics</h2>
<div className="underline-class"></div>

<h3 className="third-heading">Key Components</h3>
<div className="underline-class"></div>

- • **Links**: Rigid bodies
- • **Joints**: Connections with DOF
- • **Materials**: Visual properties
- • **Inertial**: Mass, COM, inertia
- • **Collision**: Simplified geometry
- • **Visual**: Detailed geometry

<h3 className="third-heading">Basic Structure</h3>
<div className="underline-class"></div>
```xml
<robot name="humanoid">
  <link name="base_link">
    <visual><geometry><box size="0.5 0.2 0.2"/></geometry></visual>
    <collision><geometry><box size="0.5 0.2 0.2"/></geometry></collision>
    <inertial><mass value="1.0"/></inertial>
  </link>
  
  <joint name="base_to_torso" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.3"/>
  </joint>
</robot>
```

<div className="border-line"></div>

<h2 className="second-heading">Humanoid Structure</h2>
<div className="underline-class"></div>
```
base_link
└── torso
    ├── head
    ├── left_arm → left_forearm → left_hand
    ├── right_arm → right_forearm → right_hand
    ├── left_leg → left_lower_leg → left_foot
    └── right_leg → right_lower_leg → right_foot
```

<h3 className="third-heading">Joint Types</h3>
<div className="underline-class"></div>

- • **Revolute**: Single-axis rotation
- • **Continuous**: Unlimited rotation
- • **Prismatic**: Linear motion
- • **Fixed**: Rigid connections

<div className="border-line"></div>

<h2 className="second-heading">Complete Humanoid Example</h2>
<div className="underline-class"></div>
```xml
<robot name="humanoid">
  <link name="torso">
    <inertial><mass value="5.0"/><inertia ixx="0.5" iyy="0.5" izz="0.5"/></inertial>
    <visual><geometry><box size="0.3 0.3 0.6"/></geometry></visual>
  </link>
  
  <joint name="torso_to_head" type="revolute">
    <parent link="torso"/><child link="head"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.0" upper="1.0" effort="10.0" velocity="1.0"/>
  </joint>
  
  <link name="head">
    <visual><geometry><sphere radius="0.15"/></geometry></visual>
  </link>
  
  <joint name="torso_to_left_arm" type="revolute">
    <parent link="torso"/><child link="left_upper_arm"/>
    <origin xyz="0.25 0 0.3"/>
    <axis xyz="0 1 0"/>
  </joint>
</robot>
```

<div className="border-line"></div>

<h2 className="second-heading">Xacro Macros</h2>
<div className="underline-class"></div>
```xml
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid">
  <xacro:property name="torso_height" value="0.6"/>
  
  <xacro:macro name="limb" params="name parent">
    <joint name="${parent}_to_${name}" type="revolute">
      <parent link="${parent}"/><child link="${name}"/>
      <axis xyz="0 1 0"/>
    </joint>
    <link name="${name}">
      <visual><geometry><cylinder radius="0.05" length="0.3"/></geometry></visual>
    </link>
  </xacro:macro>
  
  <xacro:limb name="left_arm" parent="torso"/>
  <xacro:limb name="right_arm" parent="torso"/>
</robot>
```

<div className="border-line"></div>

<h2 className="second-heading">Advanced Features</h2>
<div className="underline-class"></div>

<h3 className="third-heading">Transmissions</h3>
<div className="underline-class"></div>
```xml
<transmission name="tran1">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="torso_to_head">
    <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
  </joint>
  <actuator name="motor1"><mechanicalReduction>1</mechanicalReduction></actuator>
</transmission>
```

<h3 className="third-heading">Gazebo Elements</h3>
<div className="underline-class"></div>
```xml
<gazebo reference="torso">
  <material>Gazebo/Gray</material>
</gazebo>

<gazebo>
  <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
    <robotNamespace>/humanoid</robotNamespace>
  </plugin>
</gazebo>
```

<div className="border-line"></div>

<h2 className="second-heading">Validation & Visualization</h2>
<div className="underline-class"></div>
```bash
check_urdf robot.urdf
urdf_to_graphiz robot.urdf
ros2 run rviz2 rviz2
```

<h3 className="third-heading">Robot State Publisher</h3>
<div className="underline-class"></div>
```python
from sensor_msgs.msg import JointState

class RobotStatePublisher(Node):
    def __init__(self):
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
    
    def publish_joint_states(self):
        msg = JointState()
        msg.name = ['joint1', 'joint2']
        msg.position = [0.0, 1.57]
        self.joint_pub.publish(msg)
```

<div className="border-line"></div>

<h2 className="second-heading">Best Practices</h2>
<div className="underline-class"></div>

- • Follow human-like joint limits
- • Proper mass distribution
- • Simplified collision geometry
- • Use Xacro for complex models
- • Document joint purposes
- • Organize logically

<h2 className="second-heading">Summary</h2>
<div className="underline-class"></div>

URDF defines robot structure with links, joints, and properties. Use Xacro for maintainability and validate models before use.