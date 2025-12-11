---
sidebar_position: 5
title: 'World Building: Creating Complex Simulation Environments'
description: 'Designing and implementing complex simulation environments for humanoid robotics'
---

# World Building: Creating Complex Simulation Environments

Creating realistic and challenging simulation environments is crucial for developing and testing humanoid robots. This chapter explores the principles and techniques for building complex simulation worlds that accurately represent real-world scenarios. From indoor environments to outdoor terrains, we'll cover how to create worlds that challenge humanoid robots while providing meaningful testing environments.

## Learning Objectives

By the end of this chapter, you will be able to:
- Design complex simulation environments with appropriate physics properties
- Create indoor and outdoor environments for humanoid robot testing
- Implement dynamic elements and interactive objects in simulation worlds
- Configure lighting and visual effects for realistic environments
- Optimize simulation performance for complex worlds

## Exercises

<details>
<summary>Exercise 2.4.1: Basic Indoor Environment Creation (⭐, ~30 min)</summary>

### Exercise 2.4.1: Basic Indoor Environment Creation
**Difficulty**: ⭐ (Beginner)
**Time Estimate**: 30 minutes
**Requirements**: Gazebo installation, text editor, basic XML knowledge

#### Starter Code
Create a simple indoor environment with:
- Basic room structure (walls, floor, ceiling)
- Simple furniture (table, chair)
- Proper lighting
- Standard physics configuration

#### Success Criteria
- [ ] Room structure is properly defined with collision and visual elements
- [ ] Environment loads successfully in Gazebo
- [ ] Lighting appears realistic and functional
- [ ] Physics simulation works correctly
- [ ] Environment passes basic validation

#### Test Commands
```bash
# Validate SDF file
gz sdf -k your_indoor.world

# Load in Gazebo
gazebo your_indoor.world

# Check performance
gz topic -e /stats
```

#### Expected Output
- Environment should load without errors
- All elements should be visible and positioned correctly
- Physics simulation should be stable

#### Challenges
- Add more complex furniture arrangements
- Implement different lighting configurations

#### Hints
- Use proper static flags for immovable objects
- Validate SDF syntax before testing
- Start with simple shapes and add complexity gradually

</details>

<details>
<summary>Exercise 2.4.2: Advanced Environment with Dynamic Elements (⭐⭐, ~45 min)</summary>

### Exercise 2.4.2: Advanced Environment with Dynamic Elements
**Difficulty**: ⭐⭐ (Intermediate)
**Time Estimate**: 45 minutes
**Requirements**: Understanding of SDF, physics concepts, Gazebo plugins

#### Starter Code
Create an advanced environment with:
- Indoor and outdoor transitions
- Dynamic elements (movable objects)
- Interactive objects for manipulation
- Multiple lighting scenarios
- Physics properties for different surfaces

#### Success Criteria
- [ ] Environment includes both static and dynamic elements
- [ ] Dynamic elements move and interact properly
- [ ] Different surface materials work correctly
- [ ] Lighting transitions smoothly between areas
- [ ] Performance remains acceptable

#### Test Commands
```bash
# Load the advanced environment
gazebo your_advanced.world

# Test dynamic elements
gz model -m moving_platform -x 1 -y 0 -z 1

# Monitor performance metrics
gz topic -e /stats

# Test physics interactions
ros2 topic list | grep gazebo
```

#### Expected Output
- Dynamic elements should move as expected
- Physics interactions should be realistic
- Environment should maintain good performance

#### Challenges
- Add weather effects or environmental changes
- Implement sensor validation scenarios

#### Hints
- Use static flags for non-moving elements to improve performance
- Implement proper surface contact properties
- Test physics parameters to ensure stability

</details>

<details>
<summary>Exercise 2.4.3: Modular World Design System (⭐⭐⭐, ~60 min)</summary>

### Exercise 2.4.3: Modular World Design System
**Difficulty**: ⭐⭐⭐ (Advanced)
**Time Estimate**: 60 minutes
**Requirements**: Complete understanding of SDF, modular design concepts

#### Starter Code
Create a modular world building system that:
- Uses included models to build complex environments
- Implements reusable environment components
- Includes performance optimization techniques
- Supports different testing scenarios

#### Success Criteria
- [ ] Modular components are properly designed and reusable
- [ ] Environment can be assembled from different components
- [ ] Performance is optimized for complex assemblies
- [ ] System supports multiple testing scenarios
- [ ] All components integrate seamlessly

#### Test Commands
```bash
# Test modular components individually
gz sdf -k model://room_module/model.sdf

# Test assembled environment
gazebo modular_environment.world

# Test performance with different configurations
gz stats

# Validate all components
gz model -l
```

#### Expected Output
- Components should load individually and in assembly
- Performance should remain stable with multiple components
- All elements should integrate properly

#### Challenges
- Implement automatic world generation from configuration files
- Add terrain blending between modules

#### Hints
- Design components with standard connection points
- Use consistent coordinate systems
- Implement proper resource management for performance

</details>

<details>
<summary>Exercise Summary</summary>

### Exercise Summary
This chapter covered creating complex simulation environments for humanoid robotics. You learned to design indoor and outdoor environments, implement dynamic elements, configure lighting, and optimize performance. The exercises provided hands-on experience with basic environment creation, advanced features with dynamic elements, and modular world design systems.

</details>

## Troubleshooting

<details>
<summary>Troubleshooting: World Building Issues</summary>

### Troubleshooting: World Building Issues

#### Problem: Simulation runs slowly with complex worlds
**Symptoms**:
- Low real-time factor (< 0.5)
- High CPU/GPU usage
- Choppiness or lag in simulation

**Causes**:
- High polygon count in visual meshes
- Complex collision geometry
- Too many dynamic objects
- Resource-intensive physics calculations

**Solutions**:
1. Optimize visual meshes by reducing polygon count:
   ```xml
   <!-- Use simpler visual geometry -->
   <visual name="visual">
     <geometry>
       <box><size>2 2 0.1</size></box>  <!-- Instead of complex mesh -->
     </geometry>
   </visual>
   ```
2. Simplify collision geometry:
   ```xml
   <!-- Use simple collision shapes -->
   <collision name="collision">
     <geometry>
       <box><size>2 2 0.1</size></box>  <!-- Instead of complex mesh -->
     </geometry>
   </collision>
   ```
3. Mark static objects as static:
   ```xml
   <model name="static_wall" static="true">  <!-- Static flag improves performance -->
     <!-- Model content -->
   </model>
   ```
4. Adjust physics parameters for performance:
   ```xml
   <physics name="performance_physics" type="ode">
     <max_step_size>0.01</max_step_size>      <!-- Larger step size -->
     <real_time_update_rate>100</real_time_update_rate>  <!-- Lower update rate -->
   </physics>
   ```

**Verification Steps**:
- [ ] Real-time factor is above 0.8
- [ ] Simulation runs smoothly
- [ ] Acceptable CPU/GPU usage

#### Problem: Objects fall through surfaces or behave erratically
**Symptoms**:
- Objects fall through floors or walls
- Unstable physics behavior
- Objects explode or move randomly

**Causes**:
- Incorrect collision geometry
- Poor physics parameters
- Improper mass or inertia properties

**Solutions**:
1. Verify collision geometry is properly defined:
   ```xml
   <link name="solid_floor">
     <collision name="collision">
       <geometry>
         <box><size>10 10 0.1</size></box>  <!-- Proper collision shape -->
       </geometry>
       <surface>
         <contact>
           <ode>
             <kp>1e+13</kp>    <!-- High stiffness -->
             <kd>1</kd>        <!-- Appropriate damping -->
           </ode>
         </contact>
       </surface>
     </collision>
   </link>
   ```
2. Check mass and inertia properties:
   ```xml
   <link name="object_link">
     <inertial>
       <mass value="1.0"/>                    <!-- Appropriate mass -->
       <origin xyz="0 0 0"/>                  <!-- Center of mass -->
       <inertia ixx="0.01" ixy="0" ixz="0"    <!-- Proper inertia values -->
                iyy="0.01" iyz="0" izz="0.01"/>
     </inertial>
   </link>
   ```
3. Adjust physics solver parameters:
   ```xml
   <physics name="stable_physics" type="ode">
     <max_step_size>0.001</max_step_size>      <!-- Smaller step for stability -->
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

**Verification Steps**:
- [ ] Objects maintain stable position in simulation
- [ ] Proper collision detection occurs
- [ ] Physics simulation is stable

#### Problem: Lighting appears incorrect or unrealistic
**Symptoms**:
- Dark or overly bright areas
- Shadows appear incorrectly
- Colors look unrealistic

**Causes**:
- Incorrect light positioning or intensity
- Improper material properties
- Missing or incorrect attenuation settings

**Solutions**:
1. Adjust light properties for realism:
   ```xml
   <light name="room_light" type="point">
     <pose>0 0 3 0 0 0</pose>
     <diffuse>0.9 0.9 0.9 1</diffuse>        <!-- Realistic color -->
     <specular>0.3 0.3 0.3 1</specular>
     <attenuation>
       <range>10</range>                      <!-- Appropriate range -->
       <linear>0.09</linear>
       <quadratic>0.032</quadratic>
     </attenuation>
   </light>
   ```
2. Check material properties:
   ```xml
   <visual name="visual">
     <material>
       <ambient>0.3 0.3 0.3 1</ambient>      <!-- Proper ambient lighting -->
       <diffuse>0.7 0.7 0.7 1</diffuse>      <!-- Proper diffuse reflection -->
       <specular>0.2 0.2 0.2 1</specular>    <!-- Proper specular reflection -->
       <emissive>0 0 0 1</emissive>          <!-- No self-emission -->
     </material>
   </visual>
   ```
3. Use appropriate light types for different scenarios:
   ```xml
   <!-- Directional light for outdoor/sun -->
   <light name="sun" type="directional">
     <direction>-0.3 0.1 -0.9</direction>
     <diffuse>0.8 0.8 0.7 1</diffuse>
   </light>
   ```

**Verification Steps**:
- [ ] Lighting appears realistic and natural
- [ ] Colors and shadows look appropriate
- [ ] Visibility is adequate throughout the environment

#### Problem: Objects appear to float or sink into surfaces
**Symptoms**:
- Objects hover above ground
- Objects appear partially embedded in surfaces
- Inconsistent contact behavior

**Causes**:
- Incorrect pose positioning
- Misaligned collision geometry
- Improper contact surface parameters

**Solutions**:
1. Verify pose coordinates are accurate:
   ```xml
   <model name="placed_object">
     <pose>2.0 1.0 0.5 0 0 0</pose>  <!-- Ensure Z position accounts for object height -->
     <!-- Make sure the Z value places the object correctly on the surface -->
   </model>
   ```
2. Check collision geometry alignment:
   ```xml
   <link name="object_link">
     <collision name="collision">
       <geometry>
         <box><size>0.2 0.2 0.2</size></box>
       </geometry>
       <pose>0 0 0 0 0 0</pose>  <!-- Ensure collision aligns with visual -->
     </collision>
     <visual name="visual">
       <geometry>
         <box><size>0.2 0.2 0.2</size></box>
       </geometry>
       <pose>0 0 0 0 0 0</pose>  <!-- Should match collision pose -->
     </visual>
   </link>
   ```
3. Adjust contact surface parameters:
   ```xml
   <collision name="collision">
     <surface>
       <contact>
         <ode>
           <contact_surface_layer>0.001</contact_surface_layer>  <!-- Small surface layer -->
           <max_vel>100.0</max_vel>
         </ode>
       </contact>
     </surface>
   </collision>
   ```

**Verification Steps**:
- [ ] Objects rest properly on surfaces
- [ ] No floating or sinking behavior
- [ ] Contact appears natural and stable

#### Problem: Modular components don't integrate properly
**Symptoms**:
- Gaps between assembled modules
- Misaligned connections
- Performance degradation with multiple modules

**Causes**:
- Inconsistent coordinate systems
- Improper connection points
- Resource conflicts between modules

**Solutions**:
1. Use consistent coordinate systems across modules:
   ```xml
   <!-- Standardized module with consistent connection points -->
   <model name="standard_room_module">
     <pose>0 0 0 0 0 0</pose>  <!-- Clear reference point -->
     <!-- Define standard connection points -->
     <model name="connection_point_front">
       <pose>5 0 0 0 0 0</pose>  <!-- Standardized connection point -->
     </model>
   </model>
   ```
2. Implement proper resource management:
   ```xml
   <!-- Use includes to manage complex components -->
   <include>
     <uri>model://standard_furniture/table</uri>
     <pose>2 1 0 0 0 0</pose>
   </include>
   ```
3. Validate modular integration:
   ```bash
   # Check for model overlaps
   gz model -l

   # Verify connections are properly aligned
   gz topic -e /gazebo/model/pose
   ```

**Verification Steps**:
- [ ] Modules connect seamlessly without gaps
- [ ] Performance remains stable with multiple modules
- [ ] All elements function properly in assembled environment

</details>

## Environment Design Principles

### Realism vs. Performance Trade-offs

When designing simulation environments, it's important to balance realism with performance. Complex environments with detailed geometry and physics can significantly impact simulation speed:

- **Polygon count**: Keep visual mesh complexity reasonable
- **Collision geometry**: Use simplified collision meshes where possible
- **Physics complexity**: Balance realistic physics with computational efficiency
- **Texture detail**: Use appropriate texture resolution for the intended use

### Types of Environments

Different robot applications require different types of environments:

#### Indoor Environments

Indoor environments are common for humanoid robots designed for home or office assistance:

```xml
<!-- Example indoor world with furniture and obstacles -->
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="indoor_lab">
    <!-- Include standard models -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Indoor lighting -->
    <light name="room_light" type="point">
      <pose>0 0 3 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <specular>0.5 0.5 0.5 1</specular>
      <attenuation>
        <range>10</range>
        <linear>0.09</linear>
        <quadratic>0.032</quadratic>
      </attenuation>
    </light>

    <!-- Walls -->
    <model name="wall_1">
      <pose>-5 0 1.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>0.2 10 3</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>0.2 10 3</size></box>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>100</mass>
          <inertia>
            <ixx>10</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>10</iyy>
            <iyz>0</iyz>
            <izz>10</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <!-- Furniture -->
    <model name="table">
      <pose>2 1 0.5 0 0 0</pose>
      <link name="table_top">
        <collision name="collision">
          <geometry>
            <box><size>1.2 0.6 0.05</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>1.2 0.6 0.05</size></box>
          </geometry>
          <material>
            <ambient>0.6 0.4 0.2 1</ambient>
            <diffuse>0.6 0.4 0.2 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>10</mass>
          <inertia>
            <ixx>0.2</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.5</iyy>
            <iyz>0</iyz>
            <izz>0.6</izz>
          </inertia>
        </inertial>
      </link>
      <link name="leg_1">
        <pose>-0.5 -0.25 0 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.03</radius>
              <length>0.9</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.03</radius>
              <length>0.9</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0.6 0.4 0.2 1</ambient>
            <diffuse>0.6 0.4 0.2 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>0.1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.1</iyy>
            <iyz>0</iyz>
            <izz>0.001</izz>
          </inertia>
        </inertial>
      </link>
      <joint name="leg_1_joint" type="fixed">
        <parent>table_top</parent>
        <child>leg_1</child>
      </joint>
    </model>

    <!-- Doorway for navigation challenge -->
    <model name="door_frame">
      <pose>0 4 1 0 0 0</pose>
      <link name="frame">
        <collision name="collision">
          <geometry>
            <mesh>
              <uri>model://door_frame/meshes/door_frame.dae</uri>
            </mesh>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <mesh>
              <uri>model://door_frame/meshes/door_frame.dae</uri>
            </mesh>
          </geometry>
        </visual>
        <inertial>
          <mass>50</mass>
          <inertia>
            <ixx>5</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>5</iyy>
            <iyz>0</iyz>
            <izz>5</izz>
          </inertia>
        </inertial>
      </link>
    </model>
  </world>
</sdf>
```

#### Outdoor Environments

Outdoor environments test robots in more complex terrain conditions:

```xml
<!-- Example outdoor world with varied terrain -->
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="outdoor_park">
    <!-- Include standard models -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Terrain with varying elevation -->
    <model name="terrain">
      <static>true</static>
      <link name="terrain_link">
        <collision name="collision">
          <geometry>
            <heightmap>
              <uri>model://terrain/images/heightmap.png</uri>
              <size>20 20 2</size>
              <pos>0 0 0</pos>
            </heightmap>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <heightmap>
              <uri>model://terrain/images/heightmap.png</uri>
              <size>20 20 2</size>
              <pos>0 0 0</pos>
            </heightmap>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Dirt</name>
            </script>
          </material>
        </visual>
      </link>
    </model>

    <!-- Obstacles -->
    <model name="rock_1">
      <pose>-2 -3 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <mesh>
              <uri>model://rock/meshes/rock_1.dae</uri>
            </mesh>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <mesh>
              <uri>model://rock/meshes/rock_1.dae</uri>
            </mesh>
          </geometry>
        </visual>
        <inertial>
          <mass>5</mass>
          <inertia>
            <ixx>0.1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.1</iyy>
            <iyz>0</iyz>
            <izz>0.1</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <!-- Trees -->
    <model name="tree_1">
      <pose>3 2 0 0 0 0</pose>
      <link name="trunk">
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.2</radius>
              <length>3</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.2</radius>
              <length>3</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0.5 0.3 0.1 1</ambient>
            <diffuse>0.5 0.3 0.1 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>10</mass>
          <inertia>
            <ixx>1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1</iyy>
            <iyz>0</iyz>
            <izz>0.2</izz>
          </inertia>
        </inertial>
      </link>
      <link name="leaves">
        <pose>0 0 2.5 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <sphere>
              <radius>1.5</radius>
            </sphere>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <sphere>
              <radius>1.5</radius>
            </sphere>
          </geometry>
          <material>
            <ambient>0.1 0.6 0.2 1</ambient>
            <diffuse>0.1 0.6 0.2 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>5</mass>
          <inertia>
            <ixx>1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1</iyy>
            <iyz>0</iyz>
            <izz>1</izz>
          </inertia>
        </inertial>
      </link>
      <joint name="leaves_joint" type="fixed">
        <parent>trunk</parent>
        <child>leaves</child>
      </joint>
    </model>
  </world>
</sdf>
```

## Physics Configuration

### Material Properties

Setting appropriate material properties is crucial for realistic simulation:

```xml
<!-- Physics configuration for the world -->
<world name="physics_test_world">
  <!-- Physics engine configuration -->
  <physics name="ode_physics" type="ode">
    <max_step_size>0.001</max_step_size>
    <real_time_factor>1</real_time_factor>
    <real_time_update_rate>1000</real_time_update_rate>
    <gravity>0 0 -9.8</gravity>

    <!-- ODE-specific parameters -->
    <ode>
      <solver>
        <type>quick</type>
        <iters>10</iters>
        <sor>1.3</sor>
      </solver>
      <constraints>
        <cfm>0.0</cfm>
        <erp>0.2</erp>
        <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
        <contact_surface_layer>0.001</contact_surface_layer>
      </constraints>
    </ode>
  </physics>

  <!-- Surface properties for different materials -->
  <model name="floor_with_materials">
    <static>true</static>
    <link name="floor_link">
      <collision name="collision">
        <geometry>
          <box><size>10 10 0.1</size></box>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>0.8</mu>
              <mu2>0.8</mu2>
              <fdir1>0 0 0</fdir1>
              <slip1>0.0</slip1>
              <slip2>0.0</slip2>
            </ode>
            <torsional>
              <coefficient>0.8</coefficient>
              <use_patch_radius>false</use_patch_radius>
              <surface_radius>0.01</surface_radius>
            </torsional>
          </friction>
          <bounce>
            <restitution_coefficient>0.1</restitution_coefficient>
            <threshold>100000</threshold>
          </bounce>
          <contact>
            <ode>
              <soft_cfm>0</soft_cfm>
              <soft_erp>0.2</soft_erp>
              <kp>1e+13</kp>
              <kd>1</kd>
              <max_vel>100.0</max_vel>
              <min_depth>0.001</min_depth>
            </ode>
          </contact>
        </surface>
      </collision>
      <visual name="visual">
        <geometry>
          <box><size>10 10 0.1</size></box>
        </geometry>
        <material>
          <ambient>0.7 0.7 0.7 1</ambient>
          <diffuse>0.7 0.7 0.7 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</world>
```

### Friction and Contact Models

Different surfaces require different friction and contact properties for realistic interaction:

```xml
<!-- Different surface materials -->
<model name="material_test">
  <static>true</static>

  <!-- High friction surface (rubber) -->
  <link name="rubber_surface">
    <pose>-2 0 0 0 0 0</pose>
    <collision name="collision">
      <geometry>
        <box><size>2 2 0.1</size></box>
      </geometry>
      <surface>
        <friction>
          <ode>
            <mu>1.0</mu>
            <mu2>1.0</mu2>
          </ode>
        </friction>
      </surface>
    </collision>
    <visual name="visual">
      <geometry>
        <box><size>2 2 0.1</size></box>
      </geometry>
      <material>
        <ambient>0.2 0.2 0.2 1</ambient>
        <diffuse>0.2 0.2 0.2 1</diffuse>
      </material>
    </visual>
  </link>

  <!-- Low friction surface (ice) -->
  <link name="ice_surface">
    <pose>2 0 0 0 0 0</pose>
    <collision name="collision">
      <geometry>
        <box><size>2 2 0.1</size></box>
      </geometry>
      <surface>
        <friction>
          <ode>
            <mu>0.1</mu>
            <mu2>0.1</mu2>
          </ode>
        </friction>
      </surface>
    </collision>
    <visual name="visual">
      <geometry>
        <box><size>2 2 0.1</size></box>
      </geometry>
      <material>
        <ambient>0.7 0.8 0.9 1</ambient>
        <diffuse>0.7 0.8 0.9 1</diffuse>
      </material>
    </visual>
  </link>
</model>
```

## Dynamic Elements and Interactive Objects

### Moving Objects

Creating dynamic elements that interact with humanoid robots:

```xml
<!-- Moving platform for testing balance -->
<model name="moving_platform">
  <link name="platform">
    <pose>0 0 1 0 0 0</pose>
    <collision name="collision">
      <geometry>
        <box><size>2 1 0.1</size></box>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <box><size>2 1 0.1</size></box>
      </geometry>
      <material>
        <ambient>0.8 0.5 0.2 1</ambient>
        <diffuse>0.8 0.5 0.2 1</diffuse>
      </material>
    </visual>
    <inertial>
      <mass>10</mass>
      <inertia>
        <ixx>1</ixx>
        <ixy>0</ixy>
        <ixz>0</ixz>
        <iyy>2</iyy>
        <iyz>0</iyz>
        <izz>3</izz>
      </inertia>
    </inertial>
  </link>

  <!-- Joint to allow movement -->
  <joint name="platform_joint" type="prismatic">
    <parent>world</parent>
    <child>platform</child>
    <axis>
      <xyz>0 1 0</xyz>
      <limit>
        <lower>-2</lower>
        <upper>2</upper>
        <effort>100</effort>
        <velocity>1</velocity>
      </limit>
    </axis>
  </joint>

  <!-- Plugin to control the platform movement -->
  <plugin name="platform_controller" filename="libgazebo_ros_p3d.so">
    <ros>
      <namespace>/moving_platform</namespace>
    </ros>
    <update_rate>100</update_rate>
    <body_name>platform</body_name>
    <topic_name>platform_position</topic_name>
    <gaussian_noise>0.001</gaussian_noise>
  </plugin>
</model>
```

### Interactive Objects

Objects that humanoid robots can manipulate:

```xml
<!-- Interactive objects for manipulation tasks -->
<model name="interactive_objects">
  <!-- Cup that can be grasped -->
  <model name="cup">
    <pose>1 1 0.5 0 0 0</pose>
    <link name="cup_body">
      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>0.05</radius>
            <length>0.1</length>
          </cylinder>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>0.05</radius>
            <length>0.1</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.8 0.8 0.9 1</ambient>
          <diffuse>0.8 0.8 0.9 1</diffuse>
        </material>
      </visual>
      <inertial>
        <mass>0.1</mass>
        <inertia>
          <ixx>0.0001</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.0001</iyy>
          <iyz>0</iyz>
          <izz>0.00005</izz>
        </inertia>
      </inertial>
    </link>
  </model>

  <!-- Ball for catching/throwing tasks -->
  <model name="ball">
    <pose>-1 -1 1 0 0 0</pose>
    <link name="ball_link">
      <collision name="collision">
        <geometry>
          <sphere>
            <radius>0.05</radius>
          </sphere>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <sphere>
            <radius>0.05</radius>
          </sphere>
        </geometry>
        <material>
          <ambient>0.9 0.2 0.2 1</ambient>
          <diffuse>0.9 0.2 0.2 1</diffuse>
        </material>
      </visual>
      <inertial>
        <mass>0.05</mass>
        <inertia>
          <ixx>0.00005</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.00005</iyy>
          <iyz>0</iyz>
          <izz>0.00005</izz>
        </inertia>
      </inertial>
    </link>
  </model>
</model>
```

## Lighting and Visual Effects

### Environmental Lighting

Proper lighting enhances the visual experience and can affect sensor simulation:

```xml
<!-- Advanced lighting setup -->
<world name="well_lit_environment">
  <!-- Main directional light (sun) -->
  <light name="sun" type="directional">
    <pose>0 0 10 0 0 0</pose>
    <diffuse>0.8 0.8 0.8 1</diffuse>
    <specular>0.2 0.2 0.2 1</specular>
    <attenuation>
      <range>1000</range>
      <constant>0.9</constant>
      <linear>0.01</linear>
      <quadratic>0.001</quadratic>
    </attenuation>
    <direction>-0.5 0.1 -0.9</direction>
  </light>

  <!-- Ambient light -->
  <light name="ambient_light" type="directional">
    <diffuse>0.2 0.2 0.2 1</diffuse>
    <specular>0.1 0.1 0.1 1</specular>
    <direction>0 0 -1</direction>
  </light>

  <!-- Point lights for indoor areas -->
  <light name="point_light_1" type="point">
    <pose>0 0 3 0 0 0</pose>
    <diffuse>0.9 0.9 0.9 1</diffuse>
    <specular>0.5 0.5 0.5 1</specular>
    <attenuation>
      <range>5</range>
      <constant>0.2</constant>
      <linear>0.5</linear>
      <quadratic>0.01</quadratic>
    </attenuation>
  </light>
</world>
```

### Visual Effects

Adding visual effects to enhance realism:

```xml
<!-- Weather effects -->
<world name="weather_test">
  <!-- Include atmosphere for outdoor environments -->
  <atmosphere type="adiabatic">
    <temperature>288.15</temperature>
    <pressure>101325</pressure>
  </atmosphere>

  <!-- Wind effects -->
  <wind>
    <linear_velocity>0.5 0 0</linear_velocity>
  </wind>

  <!-- Particle effects for dust or fog -->
  <model name="particle_effect">
    <static>true</static>
    <link name="particle_link">
      <visual name="visual">
        <particle_emitter name="dust_emitter">
          <type>point</type>
          <size>1 1 1</size>
          <min_velocity>0.1</min_velocity>
          <max_velocity>0.2</max_velocity>
          <lifetime>3</lifetime>
          <min_quantity>10</min_quantity>
          <max_quantity>20</max_quantity>
          <color>
            <r>0.8</r>
            <g>0.7</g>
            <b>0.6</b>
            <a>0.5</a>
          </color>
          <size>0.01 0.01 0.01</size>
          <particle_size>0.05 0.05 0.05</particle_size>
          <material>Gazebo/TransWhite</material>
        </particle_emitter>
      </visual>
    </link>
  </model>
</world>
```

## World Building Best Practices

### Performance Optimization

Creating complex worlds while maintaining performance:

1. **Use static models for unchanging elements**: Mark immovable objects as static
2. **Simplify collision geometry**: Use simpler shapes for collision detection
3. **Limit dynamic elements**: Only make objects dynamic when necessary
4. **Optimize mesh complexity**: Reduce polygon count for visual meshes
5. **Group similar objects**: Use models to group similar elements

### Modular World Design

Building worlds in a modular fashion allows for reusability:

```xml
<!-- Modular world design -->
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="modular_environment">
    <!-- Include common elements -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Include modular components -->
    <include>
      <uri>model://room_module_1</uri>
      <pose>0 0 0 0 0 0</pose>
    </include>

    <include>
      <uri>model://room_module_2</uri>
      <pose>5 0 0 0 0 0</pose>
    </include>

    <!-- Include furniture components -->
    <include>
      <uri>model://table_set</uri>
      <pose>2 1 0 0 0 0</pose>
    </include>

    <!-- Include challenge components -->
    <include>
      <uri>model://obstacle_course</uri>
      <pose>-3 -2 0 0 0 0</pose>
    </include>
  </world>
</sdf>
```

### Testing Different Scenarios

Creating worlds that test different aspects of humanoid robot capabilities:

```python
# Python script to generate different world configurations
import xml.etree.ElementTree as ET
import os

def create_world_with_obstacles(world_name, obstacles):
    """
    Create a world file with specified obstacles
    """
    sdf = ET.Element('sdf', version='1.7')
    world = ET.SubElement(sdf, 'world', name=world_name)

    # Add standard elements
    ground_plane = ET.SubElement(world, 'include')
    uri = ET.SubElement(ground_plane, 'uri')
    uri.text = 'model://ground_plane'

    sun = ET.SubElement(world, 'include')
    uri = ET.SubElement(sun, 'uri')
    uri.text = 'model://sun'

    # Add obstacles based on configuration
    for i, obstacle in enumerate(obstacles):
        model = ET.SubElement(world, 'model', name=f'obstacle_{i}')
        pose = ET.SubElement(model, 'pose')
        pose.text = f"{obstacle['x']} {obstacle['y']} {obstacle['z']} 0 0 0"

        link = ET.SubElement(model, 'link', name='link')
        collision = ET.SubElement(link, 'collision', name='collision')
        geometry = ET.SubElement(collision, 'geometry')
        box = ET.SubElement(geometry, 'box')
        size = ET.SubElement(box, 'size')
        size.text = f"{obstacle['width']} {obstacle['depth']} {obstacle['height']}"

        visual = ET.SubElement(link, 'visual', name='visual')
        geometry = ET.SubElement(visual, 'geometry')
        box = ET.SubElement(geometry, 'box')
        size = ET.SubElement(box, 'size')
        size.text = f"{obstacle['width']} {obstacle['depth']} {obstacle['height']}"

        material = ET.SubElement(visual, 'material')
        ambient = ET.SubElement(material, 'ambient')
        ambient.text = "0.5 0.5 0.5 1"

        inertial = ET.SubElement(link, 'inertial')
        mass = ET.SubElement(inertial, 'mass')
        mass.text = "1.0"

        inertia = ET.SubElement(inertial, 'inertia')
        ixx = ET.SubElement(inertia, 'ixx')
        ixx.text = "0.1"
        ixy = ET.SubElement(inertia, 'ixy')
        ixy.text = "0"
        ixz = ET.SubElement(inertia, 'ixz')
        ixz.text = "0"
        iyy = ET.SubElement(inertia, 'iyy')
        iyy.text = "0.1"
        iyz = ET.SubElement(inertia, 'iyz')
        iyz.text = "0"
        izz = ET.SubElement(inertia, 'izz')
        izz.text = "0.1"

    # Write to file
    tree = ET.ElementTree(sdf)
    tree.write(f'worlds/{world_name}.world', encoding='unicode', xml_declaration=True)

# Example usage
navigation_test = [
    {'x': 2, 'y': 0, 'z': 0.5, 'width': 0.5, 'depth': 0.5, 'height': 1.0},
    {'x': 4, 'y': 1, 'z': 0.5, 'width': 0.3, 'depth': 0.3, 'height': 1.5},
    {'x': 6, 'y': -1, 'z': 0.5, 'width': 0.8, 'depth': 0.2, 'height': 0.8}
]

create_world_with_obstacles('navigation_test', navigation_test)
```

## Troubleshooting Common Issues

### Performance Problems

**Problem**: Simulation runs slowly with complex worlds
**Solutions**:
- Reduce polygon count in visual meshes
- Simplify collision geometry
- Limit the number of dynamic objects
- Adjust physics parameters (step size, update rate)

**Problem**: Objects fall through surfaces
**Solutions**:
- Check collision geometry configuration
- Verify surface contact parameters
- Adjust physics solver parameters
- Increase constraint iterations

### Visual Issues

**Problem**: Lighting appears incorrect
**Solutions**:
- Verify light direction and intensity
- Check material properties
- Adjust attenuation parameters
- Consider using different light types

**Problem**: Objects appear to float or sink
**Solutions**:
- Check pose coordinates
- Verify collision geometry alignment
- Adjust contact surface layer settings
- Review inertia properties

## Summary

Creating complex simulation environments requires careful balance between realism and performance. By understanding physics properties, lighting, and environmental design principles, you can create worlds that effectively test humanoid robots in various scenarios. Modular design approaches allow for reusability and easier maintenance of complex simulation environments.

import ReadingTime from '@site/src/components/ReadingTime';
import ViewToggle from '@site/src/components/ViewToggle';

<ReadingTime minutes={18} />
<ViewToggle />