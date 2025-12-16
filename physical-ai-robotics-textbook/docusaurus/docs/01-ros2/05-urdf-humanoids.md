---
sidebar_position: 6
title: 'URDF for Humanoid Robots: Robot Description and Modeling'
description: 'Understanding URDF (Unified Robot Description Format) for modeling humanoid robots in ROS 2'
---

import ReadingTime from '@site/src/components/ReadingTime';
import ViewToggle from '@site/src/components/ViewToggle';

<ReadingTime minutes={20} />


<h1 className="main-heading">URDF for Humanoid Robots: Robot Description and Modeling</h1>
<div className="underline-class"></div>

Unified Robot Description Format (URDF) is the standard way to describe robot models in ROS. For humanoid robots, URDF becomes particularly important as it defines the complex kinematic structure, joint configurations, and physical properties necessary for simulation and control.

<div className="border-line"></div>
---

<h2 className="second-heading">
 Learning Objectives
</h2>
<div className="underline-class"></div>

By the end of this chapter, you will be able to:
- • Create URDF models for humanoid robots with multiple degrees of freedom
- • Define links, joints, and materials for complex robot structures
- • Implement kinematic chains for arms, legs, and torso
- • Use Xacro to simplify complex humanoid URDF models
- • Validate and visualize humanoid robot models in RViz

<div className="border-line"></div>
---

<h2 className="second-heading">
 Exercises
</h2>
<div className="underline-class"></div>

<details>
<summary>Exercise 1.5.1: Basic Humanoid URDF Creation (⭐, ~30 min)</summary>

### Exercise 1.5.1: Basic Humanoid URDF Creation
**Difficulty**: ⭐ (Beginner)
**Time Estimate**: 30 minutes
**Requirements**: ROS 2 environment, text editor, basic XML knowledge

#### Starter Code
Create a simple humanoid URDF file named `basic_humanoid.urdf` with the following structure:
- Base link as torso
- Head connected to torso
- Two arms (upper and lower) connected to torso
- Two legs (upper and lower) connected to torso

#### Success Criteria
- [ ] URDF file is well-formed XML
- [ ] All links have proper visual, collision, and inertial properties
- [ ] Joints connect links with appropriate types (revolute, fixed)
- [ ] Robot model displays correctly in RViz
- [ ] URDF validates without errors using `check_urdf`

#### Test Commands
```bash
# Validate URDF
check_urdf basic_humanoid.urdf

# Visualize in RViz
ros2 launch rviz2 rviz2
```

#### Expected Output
- Robot model should appear in RViz with proper kinematic structure
- No errors when validating the URDF file
- TF tree should show all connected links

#### Challenges
- Add realistic joint limits based on human anatomy
- Implement proper mass distribution for stability

#### Hints
- Start with a simple torso and add limbs incrementally
- Use appropriate joint types for different body parts (e.g., revolute for shoulders, fixed for head)
- Ensure all origins and axes are correctly defined

</details>

<details>
<summary>Exercise 1.5.2: Xacro-based Humanoid Model (⭐⭐, ~45 min)</summary>

### Exercise 1.5.2: Xacro-based Humanoid Model
**Difficulty**: ⭐⭐ (Intermediate)
**Time Estimate**: 45 minutes
**Requirements**: Basic Xacro knowledge, ROS 2 environment

#### Starter Code
Create a Xacro file `humanoid_robot.xacro` that uses macros to define:
- A reusable limb macro that can create arms or legs
- A torso macro with attachment points
- Parameterized joint limits and dimensions

#### Success Criteria
- [ ] Xacro file compiles without errors
- [ ] Macros properly parameterize different limb types
- [ ] Robot model maintains realistic proportions
- [ ] Generated URDF validates correctly
- [ ] Robot visualizes properly in RViz

#### Test Commands
```bash
# Compile Xacro to URDF
xacro humanoid_robot.xacro > humanoid_robot.urdf

# Validate the generated URDF
check_urdf humanoid_robot.urdf

# Launch RViz to visualize
ros2 run rviz2 rviz2
```

#### Expected Output
- Xacro compilation should succeed without errors
- Generated URDF should be functionally equivalent to hand-written URDF
- Robot should display correctly with all limbs properly connected

#### Challenges
- Implement symmetry macros for left/right limbs
- Add gazebo-specific tags for simulation

#### Hints
- Use properties for common dimensions and values
- Create separate macros for different limb types
- Test each macro individually before combining

</details>

<details>
<summary>Exercise 1.5.3: Advanced Humanoid Features (⭐⭐⭐, ~60 min)</summary>

### Exercise 1.5.3: Advanced Humanoid Features
**Difficulty**: ⭐⭐⭐ (Advanced)
**Time Estimate**: 60 minutes
**Requirements**: Complete URDF/Xacro knowledge, simulation experience

#### Starter Code
Enhance your humanoid model with:
- Transmission elements for joint control
- Gazebo-specific plugins and materials
- Realistic inertial properties
- Robot state publisher node for visualization

#### Success Criteria
- [ ] Robot model includes proper transmission definitions
- [ ] Gazebo simulation runs without errors
- [ ] Joint states publish correctly for visualization
- [ ] Robot maintains stability in simulation
- [ ] All advanced features integrate properly

#### Test Commands
```bash
# Launch simulation with your robot
ros2 launch gazebo_ros gazebo

# Publish joint states
python3 robot_state_publisher.py

# Check TF tree
ros2 run tf2_tools view_frames
```

#### Expected Output
- Robot should be controllable in simulation
- Joint states should update properly
- TF tree should show complete kinematic chain
- Robot should maintain realistic movement patterns

#### Challenges
- Implement dynamic balance control
- Add sensor integration (IMU, cameras) to the model

#### Hints
- Start with a simple transmission and expand
- Use realistic inertial values based on actual humanoid robots
- Validate each new feature before adding the next

</details>

<details>
<summary>Exercise Summary</summary>

### Exercise Summary
This chapter covered creating URDF models for humanoid robots with multiple degrees of freedom. You learned to define links, joints, and materials for complex robot structures, implement kinematic chains, and use Xacro to simplify complex models. The exercises provided hands-on experience with basic URDF creation, Xacro macros, and advanced features including simulation integration.

</details>

## Troubleshooting

<details>
<summary>Troubleshooting: Common URDF Issues</summary>

### Troubleshooting: Common URDF Issues

#### Problem: Robot model appears distorted or joints move incorrectly
**Symptoms**:
- Robot limbs appear in wrong positions
- Joints rotate around incorrect axes
- Unexpected joint movements

**Causes**:
- Incorrect joint origins (xyz coordinates)
- Wrong joint axis definitions
- Improper parent-child link relationships

**Solutions**:
1. Verify joint origins and axes in your URDF:
   ```xml
   <joint name="example_joint" type="revolute">
     <parent link="parent_link"/>
     <child link="child_link"/>
     <origin xyz="0 0 0.1" rpy="0 0 0"/>  <!-- Check these values -->
     <axis xyz="0 1 0"/>                   <!-- Ensure correct axis -->
   </joint>
   ```
2. Use `check_urdf` to validate your model:
   ```bash
   check_urdf your_robot.urdf
   ```
3. Visualize the kinematic tree:
   ```bash
   urdf_to_graphiz your_robot.urdf
   ```

**Verification Steps**:
- [ ] URDF validates without errors
- [ ] Joint axes align with intended movement
- [ ] Robot displays correctly in RViz

#### Problem: Robot doesn't appear correctly in RViz
**Symptoms**:
- Robot model is invisible in RViz
- Parts of the robot are missing
- TF tree is incomplete

**Causes**:
- Robot state publisher not running
- Incorrect robot description parameter
- Missing or incorrect TF publications

**Solutions**:
1. Ensure robot_state_publisher is running:
   ```bash
   ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:=$(cat your_robot.urdf)
   ```
2. Check that joint state messages are being published:
   ```bash
   ros2 topic echo /joint_states
   ```
3. Verify TF tree connectivity:
   ```bash
   ros2 run tf2_tools view_frames
   ```

**Verification Steps**:
- [ ] Robot model appears in RViz
- [ ] All links show in TF tree
- [ ] Joint states are updating

#### Problem: Robot falls through the ground or behaves unexpectedly in simulation
**Symptoms**:
- Robot falls through the ground plane
- Unstable or erratic movements
- Robot parts collide with themselves

**Causes**:
- Incorrect inertial properties
- Poor collision geometry
- Unbalanced mass distribution

**Solutions**:
1. Verify inertial properties for each link:
   ```xml
   <link name="example_link">
     <inertial>
       <mass value="1.0"/>                    <!-- Appropriate mass -->
       <origin xyz="0 0 0"/>                  <!-- Center of mass -->
       <inertia ixx="0.01" ixy="0" ixz="0"    <!-- Proper inertia values -->
                iyy="0.01" iyz="0" izz="0.01"/>
     </inertial>
   </link>
   ```
2. Check collision geometry matches visual geometry
3. Ensure the base link has appropriate mass and inertia

**Verification Steps**:
- [ ] Robot maintains stable position in simulation
- [ ] Proper collision detection occurs
- [ ] Robot responds appropriately to physics

#### Problem: Xacro compilation errors
**Symptoms**:
- Xacro compilation fails
- Syntax errors in macro definitions
- Undefined properties or variables

**Causes**:
- Missing xmlns declaration
- Incorrect macro syntax
- Undefined properties

**Solutions**:
1. Ensure proper XML namespace:
   ```xml
   <robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="robot_name">
   ```
2. Verify macro syntax:
   ```xml
   <xacro:macro name="example_macro" params="param1 param2:=default_value">
     <!-- macro content -->
   </xacro:macro>
   ```
3. Define all properties before use:
   ```xml
   <xacro:property name="my_value" value="1.0"/>
   ```

**Verification Steps**:
- [ ] Xacro compiles without errors
- [ ] Generated URDF is valid
- [ ] Robot model displays correctly

</details>

<div className="border-line"></div>
---

<h2 className="second-heading">
 Understanding URDF for Humanoid Robots
</h2>
<div className="underline-class"></div>

URDF (Unified Robot Description Format) is an XML-based format for representing robots. For humanoid robots, URDF must capture the complex kinematic structure that mimics human anatomy with multiple limbs and joints.

<h3 className="third-heading">
 Key Components of Humanoid URDF
</h3>
<div className="underline-class"></div>

- • **Links**: Rigid bodies that represent robot parts (torso, limbs, head)
- • **Joints**: Connections between links with specific degrees of freedom
- • **Materials**: Visual properties for rendering
- • **Inertial Properties**: Mass, center of mass, and inertia tensor
- • **Collision Models**: Simplified geometry for collision detection
- • **Visual Models**: Detailed geometry for visualization

<h3 className="third-heading">
 Basic URDF Structure
</h3>
<div className="underline-class"></div>

```xml
<?xml version="1.0"?>
<robot name="humanoid_robot">
  <!-- Links define rigid bodies -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.2 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.2 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Joints define connections between links -->
  <joint name="base_to_torso" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
  </joint>

  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.3 0.6"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
  </link>
</robot>
```

<h2 className="second-heading">
 Humanoid Robot Kinematic Structure
</h2>
<div className="underline-class"></div>

Humanoid robots typically follow a hierarchical structure with the torso as the main body and limbs branching out.

<h3 className="third-heading">
 Standard Humanoid Structure
</h3>
<div className="underline-class"></div>

```
base_link (or world)
└── torso
    ├── head
    ├── left_arm
    │   ├── left_forearm
    │   └── left_hand
    ├── right_arm
    │   ├── right_forearm
    │   └── right_hand
    ├── left_leg
    │   ├── left_lower_leg
    │   └── left_foot
    └── right_leg
        ├── right_lower_leg
        └── right_foot
```

<h3 className="third-heading">
 Joint Types for Humanoid Robots
</h3>
<div className="underline-class"></div>

- • **Revolute Joints**: Single-axis rotation (shoulders, elbows, knees)
- • **Continuous Joints**: Unlimited rotation (waist, neck)
- • **Prismatic Joints**: Linear motion (if needed)
- • **Fixed Joints**: Rigid connections (head to camera mount)

<h2 className="second-heading">
 Detailed Humanoid URDF Example
</h2>
<div className="underline-class"></div>

Let's create a complete humanoid robot model:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Base link -->
  <link name="base_link">
    <inertial>
      <mass value="10.0"/>
      <origin xyz="0 0 0.5"/>
      <inertia ixx="1.0" ixy="0" ixz="0" iyy="1.0" iyz="0" izz="1.0"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.5"/>
      <geometry>
        <box size="0.2 0.2 1.0"/>
      </geometry>
      <material name="dark_gray">
        <color rgba="0.3 0.3 0.3 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.5"/>
      <geometry>
        <box size="0.2 0.2 1.0"/>
      </geometry>
    </collision>
  </link>

  <!-- Torso -->
  <joint name="base_to_torso" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 1.0" rpy="0 0 0"/>
  </joint>

  <link name="torso">
    <inertial>
      <mass value="5.0"/>
      <origin xyz="0 0 0.3"/>
      <inertia ixx="0.5" ixy="0" ixz="0" iyy="0.5" iyz="0" izz="0.5"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.3"/>
      <geometry>
        <box size="0.3 0.3 0.6"/>
      </geometry>
      <material name="light_gray">
        <color rgba="0.7 0.7 0.7 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.3"/>
      <geometry>
        <box size="0.3 0.3 0.6"/>
      </geometry>
    </collision>
  </link>

  <!-- Head -->
  <joint name="torso_to_head" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.6" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.0" upper="1.0" effort="10.0" velocity="1.0"/>
  </joint>

  <link name="head">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 0.1"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.1"/>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
      <material name="skin">
        <color rgba="0.8 0.6 0.4 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.1"/>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
    </collision>
  </link>

  <!-- Left Arm -->
  <joint name="torso_to_left_shoulder" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.25 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
  </joint>

  <link name="left_upper_arm">
    <inertial>
      <mass value="1.5"/>
      <origin xyz="0 0 -0.15"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.15"/>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15"/>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
    </collision>
  </link>

  <joint name="left_shoulder_to_elbow" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_lower_arm"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
  </joint>

  <link name="left_lower_arm">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 -0.15"/>
      <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.005"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.15"/>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15"/>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
    </collision>
  </link>

  <!-- Right Arm (similar to left) -->
  <joint name="torso_to_right_shoulder" type="revolute">
    <parent link="torso"/>
    <child link="right_upper_arm"/>
    <origin xyz="-0.25 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
  </joint>

  <link name="right_upper_arm">
    <inertial>
      <mass value="1.5"/>
      <origin xyz="0 0 -0.15"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.15"/>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15"/>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
    </collision>
  </link>

  <joint name="right_shoulder_to_elbow" type="revolute">
    <parent link="right_upper_arm"/>
    <child link="right_lower_arm"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
  </joint>

  <link name="right_lower_arm">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 -0.15"/>
      <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.005"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.15"/>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15"/>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
    </collision>
  </link>

  <!-- Left Leg -->
  <joint name="torso_to_left_hip" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_leg"/>
    <origin xyz="0.1 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="20.0" velocity="1.0"/>
  </joint>

  <link name="left_upper_leg">
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 -0.25"/>
      <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.02"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.25"/>
      <geometry>
        <cylinder radius="0.06" length="0.5"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.25"/>
      <geometry>
        <cylinder radius="0.06" length="0.5"/>
      </geometry>
    </collision>
  </link>

  <joint name="left_hip_to_knee" type="revolute">
    <parent link="left_upper_leg"/>
    <child link="left_lower_leg"/>
    <origin xyz="0 0 -0.5" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="0" upper="1.57" effort="20.0" velocity="1.0"/>
  </joint>

  <link name="left_lower_leg">
    <inertial>
      <mass value="1.5"/>
      <origin xyz="0 0 -0.25"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.25"/>
      <geometry>
        <cylinder radius="0.05" length="0.5"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.25"/>
      <geometry>
        <cylinder radius="0.05" length="0.5"/>
      </geometry>
    </collision>
  </link>

  <!-- Right Leg (similar to left) -->
  <joint name="torso_to_right_hip" type="revolute">
    <parent link="torso"/>
    <child link="right_upper_leg"/>
    <origin xyz="-0.1 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="20.0" velocity="1.0"/>
  </joint>

  <link name="right_upper_leg">
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 -0.25"/>
      <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.02"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.25"/>
      <geometry>
        <cylinder radius="0.06" length="0.5"/>
      </geometry>
      <material name="yellow">
        <color rgba="1 1 0 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.25"/>
      <geometry>
        <cylinder radius="0.06" length="0.5"/>
      </geometry>
    </collision>
  </link>

  <joint name="right_hip_to_knee" type="revolute">
    <parent link="right_upper_leg"/>
    <child link="right_lower_leg"/>
    <origin xyz="0 0 -0.5" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="0" upper="1.57" effort="20.0" velocity="1.0"/>
  </joint>

  <link name="right_lower_leg">
    <inertial>
      <mass value="1.5"/>
      <origin xyz="0 0 -0.25"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.25"/>
      <geometry>
        <cylinder radius="0.05" length="0.5"/>
      </geometry>
      <material name="yellow">
        <color rgba="1 1 0 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.25"/>
      <geometry>
        <cylinder radius="0.05" length="0.5"/>
      </geometry>
    </collision>
  </link>
</robot>
```

<h2 className="second-heading">
 Using Xacro for Complex Humanoid Models
</h2>
<div className="underline-class"></div>

Xacro (XML Macros) allows you to create more maintainable and reusable URDF models by using variables, macros, and includes.

<h3 className="third-heading">
 Basic Xacro Example
</h3>
<div className="underline-class"></div>

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid_xacro">

  <!-- Define properties -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="torso_height" value="0.6" />
  <xacro:property name="torso_width" value="0.3" />
  <xacro:property name="torso_depth" value="0.3" />

  <!-- Macro for creating a limb -->
  <xacro:macro name="limb" params="name parent side type">
    <joint name="${parent}_to_${name}" type="revolute">
      <parent link="${parent}"/>
      <child link="${name}"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
    </joint>

    <link name="${name}">
      <inertial>
        <mass value="1.0"/>
        <origin xyz="0 0 -0.15"/>
        <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
      </inertial>
      <visual>
        <origin xyz="0 0 -0.15"/>
        <geometry>
          <cylinder radius="0.05" length="0.3"/>
        </geometry>
        <material name="blue">
          <color rgba="0 0 1 1"/>
        </material>
      </visual>
      <collision>
        <origin xyz="0 0 -0.15"/>
        <geometry>
          <cylinder radius="0.05" length="0.3"/>
        </geometry>
      </collision>
    </link>
  </xacro:macro>

  <!-- Create torso -->
  <link name="torso">
    <inertial>
      <mass value="5.0"/>
      <origin xyz="0 0 ${torso_height/2}"/>
      <inertia ixx="0.5" ixy="0" ixz="0" iyy="0.5" iyz="0" izz="0.5"/>
    </inertial>
    <visual>
      <origin xyz="0 0 ${torso_height/2}"/>
      <geometry>
        <box size="${torso_depth} ${torso_width} ${torso_height}"/>
      </geometry>
      <material name="light_gray">
        <color rgba="0.7 0.7 0.7 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 ${torso_height/2}"/>
      <geometry>
        <box size="${torso_depth} ${torso_width} ${torso_height}"/>
      </geometry>
    </collision>
  </link>

  <!-- Use the macro to create arms -->
  <xacro:limb name="left_arm" parent="torso" side="left" type="arm"/>
  <xacro:limb name="right_arm" parent="torso" side="right" type="arm"/>

</robot>
```

<h2 className="second-heading">
 Advanced Humanoid Features
</h2>
<div className="underline-class"></div>

<h3 className="third-heading">
 Transmission Elements
</h3>
<div className="underline-class"></div>

For controlling joints in simulation and real robots, you need to define transmissions:

```xml
<!-- Transmission for controlling the joint -->
<transmission name="tran1">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="torso_to_head">
    <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
  </joint>
  <actuator name="motor1">
    <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

<h3 className="third-heading">
 Gazebo-Specific Elements
</h3>
<div className="underline-class"></div>

To use the model in Gazebo simulation:

```xml
<!-- Gazebo material -->
<gazebo reference="torso">
  <material>Gazebo/Gray</material>
</gazebo>

<!-- Gazebo plugin for ROS control -->
<gazebo>
  <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
    <robotNamespace>/humanoid</robotNamespace>
  </plugin>
</gazebo>
```

<h2 className="second-heading">
 Validating and Visualizing URDF Models
</h2>
<div className="underline-class"></div>

<h3 className="third-heading">
 Checking URDF Validity
</h3>
<div className="underline-class"></div>

```bash
# Check if URDF is well-formed
check_urdf /path/to/your/robot.urdf

# View the robot model
urdf_to_graphiz /path/to/your/robot.urdf
```

<h3 className="third-heading">
 Visualizing in RViz
</h3>
<div className="underline-class"></div>

```bash
# Launch RViz with robot model
ros2 run rviz2 rviz2

# Add RobotModel display and set Robot Description to your robot's parameter
```

<h3 className="third-heading">
 Using Robot State Publisher
</h3>
<div className="underline-class"></div>

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math

class RobotStatePublisher(Node):
    def __init__(self):
        super().__init__('robot_state_publisher')
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.publish_joint_states)

    def publish_joint_states(self):
        msg = JointState()
        msg.name = ['torso_to_head', 'torso_to_left_shoulder', 'left_shoulder_to_elbow']
        msg.position = [math.sin(self.get_clock().now().nanoseconds * 1e-9),
                       math.cos(self.get_clock().now().nanoseconds * 1e-9),
                       math.sin(self.get_clock().now().nanoseconds * 1e-9) * 0.5]
        msg.header.stamp = self.get_clock().now().to_msg()
        self.joint_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RobotStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

<h2 className="second-heading">
 Best Practices for Humanoid URDF
</h2>
<div className="underline-class"></div>

<h3 className="third-heading">
 Kinematic Design
</h3>
<div className="underline-class"></div>

- • Follow human-like joint limits and ranges of motion
- • Ensure proper mass distribution for stability
- • Consider center of mass for balance

<h3 className="third-heading">
 Performance Considerations
</h3>
<div className="underline-class"></div>

- • Use simplified collision geometry where possible
- • Balance visual detail with performance
- • Optimize joint limits for realistic movement

<h3 className="third-heading">
 Maintainability
</h3>
<div className="underline-class"></div>

- • Use Xacro for complex models to avoid duplication
- • Organize links and joints logically
- • Document joint purposes and limitations


<!-- <ViewToggle /> -->