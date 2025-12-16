---
sidebar_position: 3
title: 'URDF and SDF: Robot and Environment Modeling'
description: 'Understanding URDF for robot description and SDF for simulation environment modeling'
---

<h1 className="main-heading">URDF and SDF: Robot and Environment Modeling</h1>
<div className="underline-class"></div>

This chapter explores the relationship between URDF (Unified Robot Description Format) for robot modeling and SDF (Simulation Description Format) for environment modeling in Gazebo simulation. Understanding both formats is essential for creating realistic humanoid robot simulations.

<div className="border-line"></div>
---

<h2 className="second-heading">
 Learning Objectives
</h2>
<div className="underline-class"></div>

By the end of this chapter, you will be able to:
- • Understand the differences and relationships between URDF and SDF
- • Create SDF models for simulation environments
- • Integrate URDF robots with SDF environments
- • Use Gazebo-specific extensions in URDF models
- • Create complex simulation scenarios with multiple models

<div className="border-line"></div>
---

<h2 className="second-heading">
 Exercises
</h2>
<div className="underline-class"></div>

<details>
<summary>Exercise 2.2.1: Basic SDF Environment Creation (⭐, ~30 min)</summary>

<h3 className="third-heading">
 Exercise 2.2.1: Basic SDF Environment Creation
</h3>
<div className="underline-class"></div>
**Difficulty**: ⭐ (Beginner)
**Time Estimate**: 30 minutes
**Requirements**: Gazebo installation, text editor, basic XML knowledge

<h4 className="fourth-heading">
 Starter Code
</h4>
<div className="underline-class"></div>
Create a simple SDF world file that includes:
- • Basic physics configuration
- • Ground plane and lighting
- • A simple box model
- • Proper SDF version declaration
<div className="border-line"></div>

<h4 className="fourth-heading">
 Success Criteria
</h4>
<div className="underline-class"></div>
- [ ] SDF file is valid XML and SDF format
- [ ] World loads successfully in Gazebo
- [ ] Physics simulation works properly
- [ ] All elements appear correctly in simulation
- [ ] File validates with gz sdf tools
<div className="border-line"></div>

<h4 className="fourth-heading">
 Test Commands
</h4>
<div className="underline-class"></div>
```bash
# Validate SDF file
gz sdf -k your_world.sdf

<h1 className="main-heading">Load in Gazebo</h1>

gazebo your_world.sdf

<h1 className="main-heading">Or via ROS 2</h1>
 
ros2 launch gazebo_ros gazebo.launch.py world:=/path/to/your_world.sdf

<h4 className="fourth-heading">
 Expected Output
</h4>
<div className="underline-class"></div>
- • World should load without errors
- • All elements should be visible
- • Physics simulation should be stable

<h4 className="fourth-heading">
 Challenges
</h4>
<div className="underline-class"></div>
- • Add multiple objects with different shapes
- • Include a simple robot model

<h4 className="fourth-heading">
 Hints
</h4>
<div className="underline-class"></div>
- • Start with the default world as a template
- • Use proper SDF version declaration
- • Validate syntax before testing

</details>

<details>
<summary>Exercise 2.2.2: URDF-SDF Integration (⭐⭐, ~45 min)</summary>

<h3 className="third-heading">
 Exercise 2.2.2: URDF-SDF Integration
</h3>
<div className="underline-class"></div>
**Difficulty**: ⭐⭐ (Intermediate)
**Time Estimate**: 45 minutes
**Requirements**: Understanding of both URDF and SDF, Gazebo installation

<h4 className="fourth-heading">
 Starter Code
</h4>
<div className="underline-class"></div>
Create a simulation system that integrates:
- • A URDF robot model
- • An SDF world file
- • Gazebo plugins for ROS 2 integration
- • Proper joint transmissions for control

<h4 className="fourth-heading">
 Success Criteria
</h4>
<div className="underline-class"></div>
- [ ] URDF robot loads correctly in SDF world
- [ ] ROS 2 integration works properly
- [ ] Joint states are published correctly
- [ ] Robot responds to control commands
- [ ] All components work together seamlessly

<h4 className="fourth-heading">
 Test Commands
</h4>
<div className="underline-class"></div>

```bash

# Convert URDF to SDF for validation
gz sdf -p robot.urdf > robot.sdf

<h4 className="fourth-heading">
 Test Commands
</h4>
<div className="underline-class"></div>


# Launch simulation
ros2 launch your_robot_gazebo your_simulation.launch.py

# Check ROS 2 topics
ros2 topic list | grep joint
ros2 topic list | grep robot
```

<h4 className="fourth-heading">
 Expected Output
</h4>
<div className="underline-class"></div>
- • Robot should appear in simulation environment
- • ROS 2 nodes should communicate properly
- • Robot should respond to commands

<h4 className="fourth-heading">
 Challenges
</h4>
<div className="underline-class"></div>
- • Add sensor integration (camera, IMU)
- • Implement multiple robots in the same world

<h4 className="fourth-heading">
 Hints
</h4>
<div className="underline-class"></div>
- • Use proper Gazebo plugins for ROS 2 integration
- • Ensure joint names match between URDF and controllers
- • Test components individually before integration

</details>

<details>
<summary>Exercise 2.2.3: Advanced Environment Modeling (⭐⭐⭐, ~60 min)</summary>

<h3 className="third-heading">
 Exercise 2.2.3: Advanced Environment Modeling
</h3>
<div className="underline-class"></div>
**Difficulty**: ⭐⭐⭐ (Advanced)
**Time Estimate**: 60 minutes
**Requirements**: Complete understanding of SDF, physics concepts, simulation

<h4 className="fourth-heading">
 Starter Code
</h4>
<div className="underline-class"></div>
Create a complex humanoid simulation environment with:
- • Multiple rooms/floors
- • Furniture and obstacles
- • Physics parameters optimized for humanoid robots
- • Gazebo-specific extensions for sensors and controllers
- • Proper collision and visual properties

<h4 className="fourth-heading">
 Success Criteria
</h4>
<div className="underline-class"></div>
- [ ] Environment model is complete and detailed
- [ ] Physics simulation is stable and realistic
- [ ] All elements have proper collision and visual properties
- [ ] Environment supports humanoid robot simulation
- [ ] Performance is acceptable for real-time simulation

<h4 className="fourth-heading">
 Test Commands
</h4>
<div className="underline-class"></div>
```bash
# Validate complex SDF
gz sdf -k complex_environment.sdf

# Test performance
gazebo --verbose complex_environment.sdf

# Monitor real-time factor

gz topic -e /stats

<h4 className="fourth-heading">
 Expected Output
</h4>
<div className="underline-class"></div>
- • Complex environment should load successfully
- • Simulation should run with good real-time factor
- • All elements should behave correctly

<h4 className="fourth-heading">
 Challenges
</h4>
<div className="underline-class"></div>
- • Add dynamic elements (moving objects)
- • Include realistic lighting and shadows

<h4 className="fourth-heading">
 Hints
</h4>
<div className="underline-class"></div>
- • Use static models for unchanging environment elements
- • Optimize collision geometry for performance
- • Test physics parameters for stability

</details>

<details>
<summary>Exercise Summary</summary>

<h3 className="third-heading">
 Exercise Summary
</h3>
<div className="underline-class"></div>
This chapter covered understanding URDF for robot description and SDF for simulation environment modeling. You learned to create SDF models, integrate URDF robots with SDF environments, and use Gazebo-specific extensions. The exercises provided hands-on experience with basic SDF creation, URDF-SDF integration, and advanced environment modeling.

</details>

<div className="border-line"></div>
---

<h2 className="second-heading">
 Troubleshooting
</h2>
<div className="underline-class"></div>

<details>
<summary>Troubleshooting: URDF/SDF Integration Issues</summary>

<h3 className="third-heading">
 Troubleshooting: URDF/SDF Integration Issues
</h3>
<div className="underline-class"></div>

<h4 className="fourth-heading">
 Problem: URDF doesn't convert properly to SDF in Gazebo
</h4>
<div className="underline-class"></div>
**Symptoms**:
- • Robot model fails to load in Gazebo
- • Missing links or joints in simulation
- • Error messages about invalid URDF

**Causes**:
- • Missing or incorrect mesh files
- • Invalid joint limits or types
- • Incorrect inertial properties
- • Malformed XML syntax

**Solutions**:
1. Verify all referenced files exist:
   ```bash
   # Check if mesh files exist
   find /path/to/robot/description -name "*.dae" -o -name "*.stl" -o -name "*.obj"
   ```
2. Validate URDF syntax:
   ```bash
   check_urdf /path/to/robot.urdf
   ```
3. Check file paths in URDF:
   ```xml
   <mesh filename="package://my_robot_description/meshes/link.dae"/>
   ```
4. Convert and validate as SDF:
   ```bash
   gz sdf -p robot.urdf > robot.sdf
   gz sdf -k robot.sdf
   ```

**Verification Steps**:
- [ ] All mesh files exist and are accessible
- [ ] URDF validates without errors
- [ ] Robot appears correctly in Gazebo

<h4 className="fourth-heading">
 Problem: Physics instability in humanoid robot simulation
</h4>
<div className="underline-class"></div>
**Symptoms**:
- • Robot wobbles or vibrates unnaturally
- • Joints move erratically
- • Robot falls apart or explodes in simulation

**Causes**:
- • Incorrect inertial properties
- • Improper physics parameters
- • Joint limits or dynamics issues

**Solutions**:
1. Verify inertial properties for each link:
   ```xml
   <link name="link_name">
     <inertial>
       <mass value="1.0"/>                    <!-- Appropriate mass -->
       <origin xyz="0 0 0"/>                  <!-- Center of mass -->
       <inertia ixx="0.01" ixy="0" ixz="0"    <!-- Proper inertia values -->
                iyy="0.01" iyz="0" izz="0.01"/>
     </inertial>
   </link>
   ```
2. Adjust physics parameters in world file:
   ```xml
   <physics name="humanoid_physics" type="ode">
     <max_step_size>0.001</max_step_size>      <!-- Smaller for stability -->
     <real_time_update_rate>1000</real_time_update_rate>
     <ode>
       <solver>
         <iters>20</iters>                     <!-- More iterations -->
         <sor>1.3</sor>
       </solver>
       <constraints>
         <cfm>1e-5</cfm>                      <!-- Low CFM for stability -->
         <erp>0.2</erp>
       </constraints>
     </ode>
   </physics>
   ```
3. Check joint dynamics:
   ```xml
   <joint name="joint_name" type="revolute">
     <axis>
       <dynamics>
         <damping>1.0</damping>               <!-- Appropriate damping -->
         <friction>0.1</friction>
       </dynamics>
     </axis>
   </joint>
   ```

**Verification Steps**:
- [ ] Robot maintains stable position in simulation
- [ ] Joints move smoothly without excessive vibration
- [ ] Physics simulation is stable

<h4 className="fourth-heading">
 Problem: ROS 2 integration fails or nodes don't communicate
</h4>
<div className="underline-class"></div>
**Symptoms**:
- • Robot doesn't respond to ROS 2 commands
- • Missing topics or services
- • Plugin loading errors

**Causes**:
- • Incorrect plugin names or filenames
- • Namespace mismatches
- • Missing Gazebo-ROS packages

**Solutions**:
1. Verify plugin names and filenames:
   ```xml
   <gazebo>
     <plugin name="joint_state_publisher" filename="libgazebo_ros_joint_state_publisher.so">
       <ros>
         <namespace>/robot_name</namespace>
       </ros>
       <update_rate>30</update_rate>
     </plugin>
   </gazebo>
   ```
2. Check ROS 2 namespaces match:
   ```xml
   <gazebo>
     <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
       <ros>
         <namespace>/robot_name</namespace>  <!-- Must match your ROS nodes -->
       </ros>
     </plugin>
   </gazebo>
   ```
3. Ensure required packages are installed:
   ```bash
   sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control
   ```

**Verification Steps**:
- [ ] ROS 2 topics are available and publishing
- [ ] Robot responds to ROS 2 commands
- [ ] TF tree is properly maintained

<h4 className="fourth-heading">
 Problem: Simulation runs slowly or performance issues
</h4>
<div className="underline-class"></div>
**Symptoms**:
- • Low real-time factor (< 0.5)
- • Choppiness or lag in simulation
- • High CPU or GPU usage

**Causes**:
- • Complex collision geometry
- • High sensor update rates
- • Resource-intensive physics calculations

**Solutions**:
1. Simplify collision geometry:
   ```xml
   <!-- Use simpler shapes instead of complex meshes -->
   <collision name="collision">
     <geometry>
       <box><size>0.5 0.5 0.5</size></box>  <!-- Simple box -->
     </geometry>
   </collision>
   ```
2. Reduce sensor update rates:
   ```xml
   <sensor name="camera" type="camera">
     <update_rate>30</update_rate>  <!-- Lower from default 60+ -->
   </sensor>
   ```
3. Optimize physics parameters:
   ```xml
   <physics name="performance_physics" type="ode">
     <max_step_size>0.01</max_step_size>      <!-- Larger step size -->
     <real_time_update_rate>100</real_time_update_rate>  <!-- Lower rate -->
   </physics>
   ```

**Verification Steps**:
- [ ] Real-time factor is above 0.8
- [ ] Simulation runs smoothly
- [ ] Acceptable CPU/GPU usage

<h4 className="fourth-heading">
 Problem: Joint limits or transmission issues
</h4>
<div className="underline-class"></div>
**Symptoms**:
- • Robot joints move beyond physical limits
- • Controllers don't work properly
- • Joint values are incorrect

**Causes**:
- • Missing or incorrect joint limits
- • Improper transmission definitions
- • Mismatched hardware interfaces

**Solutions**:
1. Verify joint limits in URDF/SDF:
   ```xml
   <joint name="joint_name" type="revolute">
     <limit lower="-1.57" upper="1.57" effort="100" velocity="3.0"/>
   </joint>
   ```
2. Check transmission definitions:
   ```xml
   <transmission name="joint_trans">
     <type>transmission_interface/SimpleTransmission</type>
     <joint name="joint_name">
       <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
     </joint>
     <actuator name="joint_motor">
       <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
       <mechanicalReduction>1</mechanicalReduction>
     </actuator>
   </transmission>
   ```
3. Validate with ros2_control:
   ```xml
   <ros2_control name="GazeboSystem" type="system">
     <hardware>
       <plugin>gazebo_ros2_control/GazeboSystem</plugin>
     </hardware>
     <!-- Define joints and interfaces -->
   </ros2_control>
   ```

**Verification Steps**:
- [ ] Joints respect defined limits
- [ ] Controllers work properly with robot
- [ ] Joint values are within expected ranges

</details>

<div className="border-line"></div>
---

<h2 className="second-heading">
 Understanding URDF vs SDF
</h2>
<div className="underline-class"></div>

<h3 className="third-heading">
 URDF (Unified Robot Description Format)
</h3>
<div className="underline-class"></div>
URDF is primarily used for describing robot kinematics, dynamics, and visual appearance. It's ROS-centric and focuses on robot-specific properties.

**URDF Strengths:**
- • Robot kinematic chains
- • Joint limits and types
- • ROS integration
- • Parameterization with Xacro

<h3 className="third-heading">
 SDF (Simulation Description Format)
</h3>
<div className="underline-class"></div>
SDF is used by Gazebo for complete simulation environments, including robots, objects, and physics properties.

**SDF Strengths:**
- • Complete simulation environments
- • Physics properties and materials
- • Gazebo plugins and sensors
- • Multiple model inclusion

<div className="border-line"></div>
---

<h2 className="second-heading">
 Converting URDF to SDF
</h2>
<div className="underline-class"></div>

While URDF and SDF serve different purposes, they can be combined. Gazebo can read URDF files and convert them to SDF internally.

<h3 className="third-heading">
 Direct URDF in Gazebo
</h3>
<div className="underline-class"></div>

```xml
<!-- world_with_urdf.world -->
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="default">
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Include a URDF robot directly -->
    <include>
      <uri>file://$(find my_robot_description)/urdf/humanoid.urdf</uri>
      <pose>0 0 1 0 0 0</pose>
    </include>
  </world>
</sdf>
```

<h3 className="third-heading">
 Wrapping URDF in SDF
</h3>
<div className="underline-class"></div>

```xml
<!-- robot_in_sdf.sdf -->
<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="humanoid_robot">
    <!-- Include the URDF content -->
    <include>
      <uri>model://humanoid_robot_model</uri>
    </include>

    <!-- Add Gazebo-specific plugins -->
    <plugin name="ros_control" filename="libgazebo_ros_control.so">
      <robotNamespace>/humanoid_robot</robotNamespace>
    </plugin>

    <!-- Add custom physics properties -->
    <static>false</static>
  </model>
</sdf>
```

<div className="border-line"></div>
---

<h2 className="second-heading">
 SDF Environment Modeling
</h2>
<div className="underline-class"></div>

<h3 className="third-heading">
 Basic World Structure
</h3>
<div className="underline-class"></div>

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="my_humanoid_world">
    <!-- Physics engine configuration -->
    <physics name="default_physics" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
    </physics>

    <!-- Scene lighting -->
    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.4 0.2 -0.9</direction>
    </light>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Your models go here -->
    <!-- ... -->
  </world>
</sdf>
```

<h3 className="third-heading">
 Creating Custom Models in SDF
</h3>
<div className="underline-class"></div>

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="humanoid_room">
    <!-- Room floor -->
    <link name="floor">
      <pose>0 0 0 0 0 0</pose>
      <collision name="floor_collision">
        <geometry>
          <box>
            <size>10 10 0.1</size>
          </box>
        </geometry>
      </collision>
      <visual name="floor_visual">
        <geometry>
          <box>
            <size>10 10 0.1</size>
          </box>
        </geometry>
        <material>
          <ambient>0.7 0.7 0.7 1</ambient>
          <diffuse>0.7 0.7 0.7 1</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
        </material>
      </visual>
      <inertial>
        <mass>1000.0</mass>
        <inertia>
          <ixx>8333.33</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>8333.33</iyy>
          <iyz>0</iyz>
          <izz>16666.67</izz>
        </inertia>
      </inertial>
    </link>

    <!-- Walls -->
    <link name="wall_north">
      <pose>0 5.05 1.5 0 0 0</pose>
      <collision name="wall_north_collision">
        <geometry>
          <box><size>10 0.1 3</size></box>
        </geometry>
      </collision>
      <visual name="wall_north_visual">
        <geometry>
          <box><size>10 0.1 3</size></box>
        </geometry>
        <material>
          <ambient>0.5 0.5 0.5 1</ambient>
          <diffuse>0.5 0.5 0.5 1</diffuse>
        </material>
      </visual>
    </link>

    <link name="wall_south">
      <pose>0 -5.05 1.5 0 0 0</pose>
      <collision name="wall_south_collision">
        <geometry>
          <box><size>10 0.1 3</size></box>
        </geometry>
      </collision>
      <visual name="wall_south_visual">
        <geometry>
          <box><size>10 0.1 3</size></box>
        </geometry>
        <material>
          <ambient>0.5 0.5 0.5 1</ambient>
          <diffuse>0.5 0.5 0.5 1</diffuse>
        </material>
      </visual>
    </link>

    <!-- Add furniture -->
    <model name="table">
      <pose>2 0 0.4 0 0 0</pose>
      <link name="table_base">
        <collision name="collision">
          <geometry>
            <box><size>1.5 0.8 0.8</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>1.5 0.8 0.8</size></box>
          </geometry>
          <material>
            <ambient>0.8 0.6 0.2 1</ambient>
            <diffuse>0.8 0.6 0.2 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
  </model>
</sdf>
```

<div className="border-line"></div>
---

<h2 className="second-heading">
 Advanced SDF Features for Humanoid Simulation
</h2>
<div className="underline-class"></div>

<h3 className="third-heading">
 Joint Transmissions
</h3>
<div className="underline-class"></div>

In SDF, you can define how joints are controlled:

```xml
<model name="humanoid_robot">
  <!-- ... links ... -->

  <joint name="left_elbow_joint" type="revolute">
    <parent>left_upper_arm</parent>
    <child>left_lower_arm</child>
    <axis>
      <xyz>1 0 0</xyz>
      <limit>
        <lower>-2.0</lower>
        <upper>1.0</upper>
        <effort>100</effort>
        <velocity>3.0</velocity>
      </limit>
    </axis>
  </joint>

  <!-- Transmission for ROS 2 control -->
  <transmission name="left_elbow_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="left_elbow_joint">
      <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    </joint>
    <actuator name="left_elbow_motor">
      <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>
</model>
```

<h3 className="third-heading">
 Gazebo-Specific Extensions
</h3>
<div className="underline-class"></div>

Add Gazebo-specific features to your robot model:

```xml
<model name="humanoid_robot">
  <!-- ... URDF content ... -->

  <!-- Gazebo plugins -->
  <gazebo>
    <plugin name="joint_state_publisher" filename="libgazebo_ros_joint_state_publisher.so">
      <ros>
        <namespace>/humanoid_robot</namespace>
      </ros>
      <update_rate>30</update_rate>
      <joint_name>left_hip_joint</joint_name>
      <joint_name>left_knee_joint</joint_name>
      <joint_name>left_ankle_joint</joint_name>
      <!-- Add all joints you want to publish -->
    </plugin>
  </gazebo>

  <!-- Gazebo material -->
  <gazebo reference="torso_visual">
    <material>Gazebo/Orange</material>
  </gazebo>

  <!-- Physics properties -->
  <gazebo reference="left_foot">
    <mu1>0.8</mu1>
    <mu2>0.8</mu2>
    <kp>1000000.0</kp>
    <kd>100.0</kd>
    <max_contacts>10</max_contacts>
    <fdir1>1 0 0</fdir1>
  </gazebo>
</model>
```

<div className="border-line"></div>
---

<h2 className="second-heading">
 Complete Simulation Example
</h2>
<div className="underline-class"></div>

Let's create a complete humanoid simulation environment:

```xml
<!-- humanoid_lab.world -->
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="humanoid_lab">
    <!-- Physics configuration -->
    <physics name="ode" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
      <ode>
        <solver>
          <type>quick</type>
          <iters>10</iters>
          <sor>1.0</sor>
        </solver>
        <constraints>
          <cfm>0.0</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </ode>
    </physics>

    <!-- Include standard models -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Laboratory environment -->
    <model name="lab_environment" static="true">
      <!-- Room structure -->
      <link name="floor">
        <pose>0 0 0 0 0 0</pose>
        <collision name="floor_collision">
          <geometry>
            <box><size>8 6 0.1</size></box>
          </geometry>
        </collision>
        <visual name="floor_visual">
          <geometry>
            <box><size>8 6 0.1</size></box>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>

      <!-- Walls -->
      <link name="wall_north">
        <pose>0 3.05 1.5 0 0 0</pose>
        <collision name="wall_north_collision">
          <geometry>
            <box><size>8 0.1 3</size></box>
          </geometry>
        </collision>
        <visual name="wall_north_visual">
          <geometry>
            <box><size>8 0.1 3</size></box>
          </geometry>
          <material>
            <ambient>0.7 0.7 0.7 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
          </material>
        </visual>
      </link>

      <link name="wall_south">
        <pose>0 -3.05 1.5 0 0 0</pose>
        <collision name="wall_south_collision">
          <geometry>
            <box><size>8 0.1 3</size></box>
          </geometry>
        </collision>
        <visual name="wall_south_visual">
          <geometry>
            <box><size>8 0.1 3</size></box>
          </geometry>
          <material>
            <ambient>0.7 0.7 0.7 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
          </material>
        </visual>
      </link>

      <link name="wall_east">
        <pose>4.05 0 1.5 0 0 0</pose>
        <collision name="wall_east_collision">
          <geometry>
            <box><size>0.1 6 3</size></box>
          </geometry>
        </collision>
        <visual name="wall_east_visual">
          <geometry>
            <box><size>0.1 6 3</size></box>
          </geometry>
          <material>
            <ambient>0.7 0.7 0.7 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
          </material>
        </visual>
      </link>

      <!-- Furniture -->
      <model name="desk">
        <pose>-2 1 0.7 0 0 0</pose>
        <link name="desk_top">
          <collision name="collision">
            <geometry>
              <box><size>1.5 0.8 0.05</size></box>
            </geometry>
          </collision>
          <visual name="visual">
            <geometry>
              <box><size>1.5 0.8 0.05</size></box>
            </geometry>
            <material>
              <ambient>0.5 0.3 0.1 1</ambient>
              <diffuse>0.5 0.3 0.1 1</diffuse>
            </material>
          </visual>
        </link>
        <link name="desk_leg1">
          <pose>0.6 0.3 -0.375 0 0 0</pose>
          <collision name="collision">
            <geometry>
              <box><size>0.05 0.05 0.75</size></box>
            </geometry>
          </collision>
          <visual name="visual">
            <geometry>
              <box><size>0.05 0.05 0.75</size></box>
            </geometry>
            <material>
              <ambient>0.5 0.3 0.1 1</ambient>
              <diffuse>0.5 0.3 0.1 1</diffuse>
            </material>
          </visual>
        </link>
        <!-- Additional legs would go here -->
      </model>
    </model>

    <!-- Humanoid robot will be spawned here via ROS 2 -->
  </world>
</sdf>
```

<div className="border-line"></div>
---

<h2 className="second-heading">
 Working with SDF and URDF Together
</h2>
<div className="underline-class"></div>

<h3 className="third-heading">
 Converting URDF to SDF Programmatically
</h3>
<div className="underline-class"></div>

You can convert URDF to SDF using Gazebo tools:

```bash
# Convert URDF to SDF
gz sdf -p robot.urdf > robot.sdf

# Or using the older method
ign sdf -p robot.urdf > robot.sdf
```

<h3 className="third-heading">
 Using Xacro with SDF
</h3>
<div className="underline-class"></div>

While Xacro is primarily for URDF, you can use similar parameterization approaches:

```xml
<!-- parametrized_room.sdf.xacro -->
<?xml version="1.0"?>
<sdf version="1.7" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="room_length" value="10.0" />
  <xacro:property name="room_width" value="8.0" />
  <xacro:property name="room_height" value="3.0" />

  <model name="parametrized_room">
    <link name="floor">
      <pose>0 0 0 0 0 0</pose>
      <collision name="floor_collision">
        <geometry>
          <box>
            <size>${room_length} ${room_width} 0.1</size>
          </box>
        </geometry>
      </collision>
      <visual name="floor_visual">
        <geometry>
          <box>
            <size>${room_length} ${room_width} 0.1</size>
          </box>
        </geometry>
        <material>
          <ambient>0.7 0.7 0.7 1</ambient>
          <diffuse>0.7 0.7 0.7 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
```

<div className="border-line"></div>
---

<h2 className="second-heading">
 Physics Considerations for Humanoid Robots
</h2>
<div className="underline-class"></div>
<div className="border-line"></div>
<h3 className="third-heading">
 Appropriate Physics Parameters
</h3>
<div className="underline-class"></div>

For humanoid robots, careful physics tuning is crucial:

```xml
<world name="humanoid_world">
  <physics name="humanoid_physics" type="ode">
    <!-- Smaller step size for stability -->
    <max_step_size>0.001</max_step_size>
    <!-- Higher update rate for responsiveness -->
    <real_time_update_rate>1000</real_time_update_rate>
    <!-- Normal gravity -->
    <gravity>0 0 -9.8</gravity>

    <ode>
      <solver>
        <!-- Quick solver for real-time performance -->
        <type>quick</type>
        <!-- Balance between stability and performance -->
        <iters>20</iters>
        <sor>1.3</sor>
      </solver>
      <constraints>
        <!-- Low CFM for stable contacts -->
        <cfm>1e-5</cfm>
        <!-- ERP controls how errors are corrected -->
        <erp>0.2</erp>
        <!-- Prevent excessive velocity correction -->
        <contact_max_correcting_vel>10.0</contact_max_correcting_vel>
        <!-- Contact layer thickness -->
        <contact_surface_layer>0.001</contact_surface_layer>
      </constraints>
    </ode>
  </physics>
</world>
```

<h3 className="third-heading">
 Joint Damping and Friction
</h3>
<div className="underline-class"></div>

For realistic humanoid movement:

```xml
<joint name="left_knee_joint" type="revolute">
  <parent>left_thigh</parent>
  <child>left_calf</child>
  <axis>
    <xyz>1 0 0</xyz>
    <limit>
      <lower>0</lower>
      <upper>2.5</upper>
      <effort>100</effort>
      <velocity>5.0</velocity>
    </limit>
    <!-- Damping for natural movement -->
    <dynamics>
      <damping>1.0</damping>
      <friction>0.1</friction>
    </dynamics>
  </axis>
</joint>
```

<div className="border-line"></div>
---

<h2 className="second-heading">
 Best Practices
</h2>
<div className="underline-class"></div>

<h3 className="third-heading">
 Model Organization
</h3>
<div className="underline-class"></div>
- • Use descriptive names for models, links, and joints
- • Group related elements logically
- • Use consistent naming conventions
- • Document complex models with comments

<h3 className="third-heading">
 Performance Optimization
</h3>
<div className="underline-class"></div>
- • Simplify collision geometry where possible
- • Use appropriate physics parameters
- • Limit sensor update rates
- • Use static models for unchanging environment elements

<h3 className="third-heading">
 Compatibility
</h3>
<div className="underline-class"></div>
- • Ensure URDF models can be converted to SDF properly
- • Use compatible joint limits between simulation and real robot
- • Test models in both simulation and reality when possible

<div className="border-line"></div>
---

<h2 className="second-heading">
 Integration with ROS 2 Launch Files
</h2>
<div className="underline-class"></div>

You can integrate SDF environments with ROS 2 launch files:

```python
# launch/humanoid_simulation.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch arguments
    world_file = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([
            get_package_share_directory('my_robot_gazebo'),
            'worlds',
            'humanoid_lab.world'
        ]),
        description='Path to the Gazebo world file'
    )

    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('gazebo_ros'),
            '/launch/gazebo.launch.py'
        ]),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'verbose': 'false'
        }.items()
    )

    return LaunchDescription([
        world_file,
        gazebo_launch,
        # Add robot spawning nodes here
    ])
```

