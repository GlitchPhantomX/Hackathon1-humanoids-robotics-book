---
sidebar_position: 7
title: 'Advanced Simulation: Complex Scenarios and Optimization'
description: 'Advanced simulation techniques for complex robotics scenarios and performance optimization'
---
# <h1 className="main-heading">Advanced Simulation: Complex Scenarios and Optimization</h1>
<div className="underline-class"></div>

As humanoid robotics applications become more sophisticated, simulation environments must evolve to handle complex scenarios, multi-robot systems, and high-fidelity physics. This chapter explores advanced simulation techniques that enable realistic testing of humanoid robots in challenging environments while maintaining optimal performance.

<h2 className="second-heading">
Learning Objectives
</h2>
<div className="underline-class"></div>

By the end of this chapter, you will be able to:
- • Design and implement complex multi-robot simulation scenarios
- • Optimize simulation performance for large-scale environments
- • Implement advanced physics models for realistic interactions
- • Create dynamic environments that respond to robot actions
- • Apply advanced techniques for simulation-to-reality transfer

<h2 className="second-heading">
Exercises
</h2>
<div className="underline-class"></div>

<details>
<summary>Exercise 2.6.1: Multi-Robot Coordination in Complex Environments (⭐⭐, ~40 min)</summary>

<h3 className="third-heading">
- Exercise 2.6.1: Multi-Robot Coordination in Complex Environments
</h3>
<div className="underline-class"></div>
**Difficulty**: ⭐⭐ (Intermediate)
**Time Estimate**: 40 minutes
**Requirements**: Gazebo installation, ROS 2, multi-robot coordination concepts

<h4 className="fourth-heading">
Starter Code
</h4>
<div className="underline-class"></div>
Create a multi-robot simulation with coordination:
- • Two humanoid robots in the same environment
- • Coordination algorithm to avoid collisions
- • Task assignment system
- • Communication between robots
- • Basic path planning for collaboration

<h4 className="fourth-heading">
Success Criteria
</h4>
<div className="underline-class"></div>
- • [ ] Both robots operate simultaneously in the same simulation
- • [ ] Coordination algorithm prevents collisions between robots
- • [ ] Robots successfully complete assigned tasks
- • [ ] Communication system functions properly
- • [ ] Performance remains stable with multiple robots

<h4 className="fourth-heading">
Test Commands
</h4>
<div className="underline-class"></div>
```bash
# Launch multi-robot simulation
gz sim -r multi_robot_lab.sdf

# Check for both robots' topics
ros2 topic list | grep humanoid

# Monitor robot positions
ros2 topic echo /humanoid_1/pose
ros2 topic echo /humanoid_2/pose

# Check coordination status
ros2 topic echo /coordination/status

# Test task assignment
ros2 topic pub /task_assignment std_msgs/msg/String "data: 'humanoid_1:move_to_table'"

# Monitor performance
ros2 run tf2_tools view_frames
ros2 topic hz /humanoid_1/camera/image_raw
```

<h4 className="fourth-heading">
Expected Output
</h4>
<div className="underline-class"></div>
- • Both robots should be visible in simulation
- • Robots should avoid colliding with each other
- • Tasks should be properly assigned and executed
- • Communication should be maintained between robots
- • System performance should remain stable

<h4 className="fourth-heading">
Challenges
</h4>
<div className="underline-class"></div>
- • Implement formation control algorithms
- • Add dynamic task reassignment based on robot capabilities
- • Implement resource sharing between robots

<h4 className="fourth-heading">
Hints
</h4>
<div className="underline-class"></div>
- • Use namespaces to separate robot topics and parameters
- • Implement proper collision avoidance algorithms
- • Design efficient communication protocols to minimize network overhead

</details>

<details>
<summary>Exercise 2.6.2: Advanced Physics Simulation with Realistic Contact Dynamics (⭐⭐, ~50 min)</summary>

<h3 className="third-heading">
- Exercise 2.6.2: Advanced Physics Simulation with Realistic Contact Dynamics
</h3>
<div className="underline-class"></div>
**Difficulty**: ⭐⭐ (Intermediate)
**Time Estimate**: 50 minutes
**Requirements**: Understanding of physics engines, contact dynamics, humanoid robot models

<h4 className="fourth-heading">
Starter Code
</h4>
<div className="underline-class"></div>
Create a humanoid robot with advanced physics properties:
- • Realistic contact models for feet and hands
- • Flexible body dynamics using multiple segments
- • Proper friction and restitution coefficients
- • Advanced surface properties for interactions
- • Balance control with realistic physics

<h4 className="fourth-heading">
Success Criteria
</h4>
<div className="underline-class"></div>
- • [ ] Robot maintains stable balance with advanced physics
- • [ ] Contact dynamics behave realistically
- • [ ] Flexible body segments respond appropriately
- • [ ] Physics parameters match real-world expectations
- • [ ] Robot can perform basic movements without instability

<h4 className="fourth-heading">
Test Commands
</h4>
<div className="underline-class"></div>
```bash
# Validate physics configuration
gz sdf -k advanced_physics_robot.sdf

# Launch simulation with advanced physics
gz sim -r advanced_physics_world.sdf

# Monitor contact forces
gz topic -e /world/advanced_physics_world/contact
gz topic -e /link/foot_link/wrench

# Test robot balance
ros2 topic pub /balance_control std_msgs/msg/Float64 "data: 1.0"

# Monitor joint states with physics effects
ros2 topic echo /robot/joint_states

# Check physics parameters
ros2 service call /get_physics_properties gazebo_msgs/srv/GetPhysicsProperties
```

<h4 className="fourth-heading">
Expected Output
</h4>
<div className="underline-class"></div>
- • Robot should maintain stable balance with realistic physics
- • Contact forces should be properly simulated
- • Joint movements should reflect physics constraints
- • Robot should respond appropriately to external forces
- • Physics parameters should be within realistic ranges

<h4 className="fourth-heading">
Challenges
</h4>
<div className="underline-class"></div>
- • Implement dynamic balance control with physics constraints
- • Add realistic muscle/servo dynamics simulation
- • Model soft tissue effects in humanoid body

<h4 className="fourth-heading">
Hints
</h4>
<div className="underline-class"></div>
- • Start with basic physics parameters and gradually add complexity
- • Use appropriate friction coefficients for different surfaces
- • Validate contact models with real-world measurements

</details>

<details>
<summary>Exercise 2.6.3: Dynamic Environment with Interactive Elements (⭐⭐⭐, ~65 min)</summary>

<h3 className="third-heading">
- Exercise 2.6.3: Dynamic Environment with Interactive Elements
</h3>
<div className="underline-class"></div>
**Difficulty**: ⭐⭐⭐ (Advanced)
**Time Estimate**: 65 minutes
**Requirements**: Advanced Gazebo knowledge, dynamic simulation, environment modeling

<h4 className="fourth-heading">
Starter Code
</h4>
<div className="underline-class"></div>
Create a dynamic environment that responds to robot actions:
- • Interactive objects that can be moved by robots
- • Environmental effects (wind, lighting changes)
- • Adaptive terrain that changes based on robot actions
- • Sensor noise modeling for realistic perception
- • Performance optimization for complex interactions

<h4 className="fourth-heading">
Success Criteria
</h4>
<div className="underline-class"></div>
- • [ ] Environment objects respond realistically to robot interactions
- • [ ] Environmental effects influence robot behavior
- • [ ] Sensor data includes realistic noise models
- • [ ] Performance remains optimized with dynamic elements
- • [ ] Robot can successfully interact with environment

<h4 className="fourth-heading">
Test Commands
</h4>
<div className="underline-class"></div>
```bash
# Launch dynamic environment
gz sim -r interactive_environment.sdf

# Test object manipulation
ros2 topic pub /robot/gripper/command std_msgs/msg/Float64 "data: 0.5"

# Test environmental effects
ros2 topic echo /environment/wind_effect
ros2 topic echo /environment/weather_conditions

# Monitor performance with dynamic elements
gz stats
ros2 topic hz /camera/image_raw

# Test sensor noise
ros2 topic echo /noisy_lidar --field ranges --field header
ros2 topic echo /noisy_imu --field orientation

# Validate environment changes
gz topic -e /world/interactive_environment/model/pose
```

<h4 className="fourth-heading">
Expected Output
</h4>
<div className="underline-class"></div>
- • Objects should move when manipulated by robots
- • Environmental effects should influence robot sensors
- • Sensor data should include realistic noise patterns
- • Performance should remain stable despite dynamic elements
- • Robot should be able to successfully interact with environment

<h4 className="fourth-heading">
Challenges
</h4>
<div className="underline-class"></div>
- • Implement realistic fluid dynamics for liquid simulation
- • Add haptic feedback simulation for robot interactions
- • Create adaptive environments that learn from robot behavior

<h4 className="fourth-heading">
Hints
</h4>
<div className="underline-class"></div>
- • Use appropriate physics parameters for different object types
- • Implement efficient collision detection for dynamic objects
- • Use domain randomization to improve reality transfer

</details>

<details>
<summary>Exercise Summary</summary>

<h3 className="third-heading">
- Exercise Summary
</h3>
<div className="underline-class"></div>
This chapter covered advanced simulation techniques for complex robotics scenarios and performance optimization. You learned to design multi-robot systems, implement advanced physics models, create dynamic environments, and apply techniques for simulation-to-reality transfer. The exercises provided hands-on experience with multi-robot coordination, advanced physics simulation, and dynamic environment creation.

</details>

<h2 className="second-heading">
Troubleshooting
</h2>
<div className="underline-class"></div>

<details>
<summary>Troubleshooting: Advanced Simulation Issues</summary>

<h3 className="third-heading">
- Troubleshooting: Advanced Simulation Issues
</h3>
<div className="underline-class"></div>

<h4 className="fourth-heading">
Problem: Multi-robot simulation performance degrades significantly
</h4>
<div className="underline-class"></div>
**Symptoms**:
- • Low real-time factor (< 0.5) with multiple robots
- • High CPU/GPU usage
- • Robot movements become jerky or delayed
- • Message queue overflows in ROS 2

**Causes**:
- • High computational load from multiple physics calculations
- • Excessive sensor data from multiple robots
- • Network congestion from multiple ROS 2 nodes
- • Inefficient collision detection with many objects

**Solutions**:
1. Optimize sensor update rates for each robot:
   ```xml
   <!-- Reduce update rates for better performance -->
   <sensor name="camera_1" type="camera">
     <update_rate>15</update_rate>  <!-- Reduced from 30 -->
     <!-- ... other configuration ... -->
   </sensor>

   <sensor name="camera_2" type="camera">
     <update_rate>15</update_rate>  <!-- Reduced from 30 -->
     <!-- ... other configuration ... -->
   </sensor>
   ```

2. Implement Level of Detail (LOD) systems:
   ```python
   # Adjust model complexity based on distance
   def adjust_robot_detail(robot_name, distance_to_observer):
       if distance_to_observer > 10:  # Far away
           # Switch to low-detail model
           change_model_mesh(robot_name, "low_detail_mesh.dae")
       elif distance_to_observer > 5:  # Medium distance
           # Switch to medium-detail model
           change_model_mesh(robot_name, "medium_detail_mesh.dae")
       else:  # Close up
           # Use high-detail model
           change_model_mesh(robot_name, "high_detail_mesh.dae")
   ```

3. Use parallel simulation instances for scalability:
   ```bash
   # Run separate simulation instances for different robot groups
   gz sim -r robot_group_1.sdf --world-name=group1 &
   gz sim -r robot_group_2.sdf --world-name=group2 &

   # Use separate ROS 2 domains for each instance
   export ROS_DOMAIN_ID=1
   ros2 launch robot_group_1 bringup.launch.py
   ```

4. Optimize physics parameters for multiple robots:
   ```xml
   <!-- Optimized physics for multi-robot simulation -->
   <physics name="ode_multi_robot" type="ode">
     <max_step_size>0.002</max_step_size>      <!-- Slightly larger for performance -->
     <real_time_update_rate>500</real_time_update_rate>  <!-- Reduced for stability -->
     <gravity>0 0 -9.8</gravity>
     <ode>
       <solver>
         <type>quick</type>
         <iters>20</iters>                     <!-- Reduced iterations -->
         <sor>1.3</sor>
       </solver>
       <constraints>
         <cfm>1e-4</cfm>                      <!-- Slightly higher for stability -->
         <erp>0.1</erp>
       </constraints>
     </ode>
   </physics>
   ```

**Verification Steps**:
- • [ ] Real-time factor is above 0.8 with multiple robots
- • [ ] CPU usage is within acceptable limits
- • [ ] Robot movements are smooth and responsive
- • [ ] No message queue overflow errors

<h4 className="fourth-heading">
Problem: Physics simulation is unstable or unrealistic
</h4>
<div className="underline-class"></div>
**Symptoms**:
- • Robots fall through floors or walls
- • Objects exhibit jittery or explosive behavior
- • Joint limits are not respected
- • Balance control is impossible to achieve

**Causes**:
- • Incorrect physics parameters
- • Inappropriate time step size
- • Poor mass/inertia properties
- • Insufficient constraint iterations

**Solutions**:
1. Tune physics engine parameters for stability:
   ```xml
   <!-- Stable physics configuration -->
   <physics name="ode_stable" type="ode">
     <max_step_size>0.001</max_step_size>      <!-- Small time step for accuracy -->
     <real_time_factor>1.0</real_time_factor>
     <real_time_update_rate>1000</real_time_update_rate>
     <gravity>0 0 -9.8</gravity>
     <ode>
       <solver>
         <type>quick</type>
         <iters>100</iters>                    <!-- More iterations for stability -->
         <sor>1.3</sor>
       </solver>
       <constraints>
         <cfm>1e-6</cfm>                       <!-- Very low CFM for tight constraints -->
         <erp>0.2</erp>                        <!-- Good error reduction -->
       </constraints>
     </ode>
   </physics>
   ```

2. Verify mass and inertia properties:
   ```xml
   <!-- Properly configured humanoid link -->
   <link name="torso_link">
     <inertial>
       <mass>10.0</mass>                       <!-- Appropriate mass -->
       <origin xyz="0 0 0.2" rpy="0 0 0"/>    <!-- COM position -->
       <inertia>
         <ixx>0.3</ixx>                        <!-- Calculated inertia values -->
         <ixy>0</ixy>
         <ixz>0</ixz>
         <iyy>0.4</iyy>
         <iyz>0</iyz>
         <izz>0.5</izz>
       </inertia>
     </inertial>
   </link>
   ```

3. Configure proper surface contact properties:
   ```xml
   <!-- Realistic contact properties -->
   <surface>
     <friction>
       <ode>
         <mu>0.8</mu>                          <!-- Static friction coefficient -->
         <mu2>0.8</mu2>                        <!-- Dynamic friction coefficient -->
         <slip1>0.001</slip1>                  <!-- Slip in primary direction -->
         <slip2>0.001</slip2>                  <!-- Slip in secondary direction -->
       </ode>
       <torsional>
         <coefficient>0.1</coefficient>         <!-- Torsional friction -->
       </torsional>
     </friction>
     <bounce>
       <restitution_coefficient>0.1</restitution_coefficient>  <!-- Low bounce -->
       <threshold>100000</threshold>
     </bounce>
     <contact>
       <ode>
         <soft_cfm>0.001</soft_cfm>            <!-- Soft constraint force mixing -->
         <soft_erp>0.8</soft_erp>              <!-- Error reduction parameter -->
         <kp>1e+7</kp>                         <!-- Proportional stiffness -->
         <kd>1</kd>                            <!-- Derivative damping -->
         <max_vel>100.0</max_vel>              <!-- Maximum contact velocity -->
         <min_depth>0.001</min_depth>          <!-- Penetration depth tolerance -->
       </ode>
     </contact>
   </surface>
   ```

4. Validate joint configurations:
   ```xml
   <!-- Properly configured joint -->
   <joint name="hip_joint" type="revolute">
     <parent>torso_link</parent>
     <child>thigh_link</child>
     <axis>
       <xyz>1 0 0</xyz>                        <!-- Rotation axis -->
       <limit>
         <lower>-1.57</lower>                  <!-- Joint limits -->
         <upper>1.57</upper>
         <effort>100</effort>                  <!-- Maximum torque -->
         <velocity>2</velocity>                <!-- Maximum velocity -->
       </limit>
       <dynamics>
         <damping>1.0</damping>                <!-- Joint damping -->
         <friction>0.1</friction>              <!-- Joint friction -->
       </dynamics>
     </axis>
   </joint>
   ```

**Verification Steps**:
- • [ ] Robots maintain stable positions without falling through surfaces
- • [ ] Objects behave predictably without explosive movements
- • [ ] Joint limits are properly enforced
- • [ ] Balance control is achievable

<h4 className="fourth-heading">
Problem: Dynamic environments cause simulation instability
</h4>
<div className="underline-class"></div>
**Symptoms**:
- • Simulation becomes unstable when objects are moved
- • Physics calculations fail with dynamic elements
- • Performance drops significantly with interactive objects
- • Robot control becomes erratic in dynamic environments

**Causes**:
- • Complex collision geometries with dynamic objects
- • Inadequate constraint solving for moving objects
- • Resource conflicts with dynamic elements
- • Poor update rates for dynamic systems

**Solutions**:
1. Simplify collision geometries for dynamic objects:
   ```xml
   <!-- Simplified collision geometry for dynamic object -->
   <link name="movable_object">
     <collision name="collision">
       <geometry>
         <box><size>0.2 0.2 0.2</size></box>    <!-- Simple box instead of complex mesh -->
       </geometry>
       <!-- Keep visual geometry complex but simplify collision -->
       <surface>
         <friction>
           <ode>
             <mu>0.5</mu>
             <mu2>0.5</mu2>
           </ode>
         </friction>
       </surface>
     </collision>
     <visual name="visual">
       <geometry>
         <mesh><uri>model://complex_object/meshes/object.dae</uri></mesh>  <!-- Complex visual -->
       </geometry>
     </visual>
   </link>
   ```

2. Implement adaptive update rates:
   ```python
   # Dynamic update rate based on simulation complexity
   class AdaptiveController:
       def __init__(self):
           self.base_rate = 100  # Hz
           self.current_rate = self.base_rate

       def adjust_update_rate(self, object_count, robot_count, physics_complexity):
           # Calculate complexity factor
           complexity_factor = (object_count * 0.1 + robot_count * 0.5 + physics_complexity) / 10

           # Adjust rate inversely proportional to complexity
           adjusted_rate = max(10, self.base_rate / (1 + complexity_factor))

           # Update controller rate
           self.current_rate = adjusted_rate
           return adjusted_rate
   ```

3. Use proper joint controllers for interactive elements:
   ```xml
   <!-- Controlled interactive element -->
   <model name="interactive_door">
     <joint name="door_hinge" type="revolute">
       <parent>door_frame</parent>
       <child>door_panel</child>
       <axis>
         <xyz>0 1 0</xyz>
         <limit>
           <lower>-1.57</lower>
           <upper>0</upper>
           <effort>50</effort>                 <!-- Adequate effort for smooth operation -->
           <velocity>0.5</velocity>
         </limit>
         <dynamics>
           <damping>5.0</damping>              <!-- Damping to prevent oscillation -->
           <friction>1.0</friction>
         </dynamics>
       </axis>
     </joint>

     <!-- Joint controller plugin -->
     <plugin name="door_controller" filename="libgazebo_ros_joint_trajectory.so">
       <ros>
         <namespace>/environment</namespace>
       </ros>
       <command_topic>door_trajectory</command_topic>
       <joint_name>door_hinge</joint_name>
     </plugin>
   </model>
   ```

4. Optimize for dynamic environments:
   ```xml
   <!-- Physics optimized for dynamic environments -->
   <physics name="ode_dynamic" type="ode">
     <max_step_size>0.001</max_step_size>
     <real_time_factor>1.0</real_time_factor>
     <real_time_update_rate>1000</real_time_update_rate>
     <gravity>0 0 -9.8</gravity>
     <ode>
       <solver>
         <type>quick</type>
         <iters>50</iters>                     <!-- Balance between stability and performance -->
         <sor>1.3</sor>
       </solver>
       <quick_step>
         <sor>1.3</sor>
         <num_iterations>50</num_iterations>    <!-- Quick step iterations -->
         <w>1.2</w>
       </quick_step>
       <constraints>
         <cfm>1e-5</cfm>
         <erp>0.2</erp>
       </constraints>
     </ode>
   </physics>
   ```

**Verification Steps**:
- • [ ] Dynamic objects behave stably without causing simulation issues
- • [ ] Performance remains acceptable with interactive elements
- • [ ] Robot control remains responsive in dynamic environments
- • [ ] No physics errors or instabilities with moving objects

<h4 className="fourth-heading">
Problem: Simulation-to-reality transfer fails
</h4>
<div className="underline-class"></div>
**Symptoms**:
- • Behaviors that work in simulation fail on real robots
- • Control parameters need extensive retuning for real hardware
- • Sensor data distributions differ significantly between sim and reality
- • Robot dynamics behave differently in simulation vs. reality

**Causes**:
- • Inaccurate sensor models
- • Unrealistic physics parameters
- • Missing real-world effects in simulation
- • Domain gap between simulation and reality

**Solutions**:
1. Implement realistic sensor noise models:
   ```python
   # Realistic LIDAR noise model
   def add_realistic_lidar_noise(ranges, distance_dependent=True):
       noisy_ranges = []
       for i, range_val in enumerate(ranges):
           if range_val > 0:  # Valid range
               # Distance-dependent noise (more noise at longer distances)
               if distance_dependent:
                   noise_std = 0.01 + 0.005 * range_val  # Base + distance-dependent
               else:
                   noise_std = 0.02  # Constant noise

               # Add Gaussian noise
               noise = np.random.normal(0, noise_std)
               noisy_range = max(0.05, range_val + noise)  # Ensure minimum range
               noisy_ranges.append(noisy_range)
           else:
               noisy_ranges.append(range_val)  # Keep invalid ranges as-is

       return noisy_ranges
   ```

2. Apply domain randomization:
   ```python
   # Domain randomization parameters
   class DomainRandomizer:
       def __init__(self):
           self.param_ranges = {
               'gravity': (-10.1, -9.5),           # Gravity variation
               'friction': (0.4, 1.2),             # Friction variation
               'mass_multiplier': (0.9, 1.1),      # Mass variation
               'inertia_multiplier': (0.95, 1.05), # Inertia variation
               'motor_torque': (0.8, 1.2),         # Motor strength variation
           }

       def randomize_parameters(self):
           randomized_params = {}
           for param, (min_val, max_val) in self.param_ranges.items():
               randomized_params[param] = random.uniform(min_val, max_val)
           return randomized_params
   ```

3. Implement realistic actuator dynamics:
   ```xml
   <!-- Realistic actuator model -->
   <transmission name="joint_transmission">
     <type>transmission_interface/SimpleTransmission</type>
     <joint name="hip_joint">
       <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
     </joint>
     <actuator name="hip_actuator">
       <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
       <mechanicalReduction>1</mechanicalReduction>
       <!-- Add actuator dynamics -->
       <dynamics>
         <damping>0.1</damping>                  <!-- Motor damping -->
         <friction>0.05</friction>              <!-- Static friction -->
         <spring_stiffness>1000</spring_stiffness>  <!-- Compliance -->
         <spring_damper>50</spring_damper>      <!-- Compliance damping -->
       </dynamics>
     </actuator>
   </transmission>
   ```

4. Validate simulation against real hardware:
   ```python
   # Validation node to compare sim vs real
   class SimulationValidator(Node):
       def __init__(self):
           super().__init__('simulation_validator')

           # Subscribe to both simulated and real sensor data
           self.sim_sub = self.create_subscription(
               SensorData, '/sim/sensor_data', self.sim_callback, 10)
           self.real_sub = self.create_subscription(
               SensorData, '/real/sensor_data', self.real_callback, 10)

           # Timer for validation
           self.validation_timer = self.create_timer(1.0, self.validate_models)

       def validate_models(self):
           # Compare distributions of sensor data
           sim_stats = self.get_statistics(self.sim_buffer)
           real_stats = self.get_statistics(self.real_buffer)

           # Calculate similarity metrics
           similarity = self.calculate_similarity(sim_stats, real_stats)

           if similarity < 0.8:  # Threshold for acceptable similarity
               self.get_logger().warn(f'Low sim-to-real similarity: {similarity:.2f}')
   ```

**Verification Steps**:
- • [ ] Control behaviors transfer successfully to real hardware
- • [ ] Sensor data distributions match between sim and reality
- • [ ] Robot dynamics behave similarly in both environments
- • [ ] Minimal parameter retuning required for real deployment

<h4 className="fourth-heading">
Problem: Advanced simulation consumes excessive resources
</h4>
<div className="underline-class"></div>
**Symptoms**:
- • High CPU usage (80%+ on multi-core systems)
- • Memory usage grows over time (memory leaks)
- • GPU usage is excessive even with basic rendering
- • Simulation performance degrades over time

**Causes**:
- • Complex rendering with high-quality graphics
- • Memory leaks in simulation plugins
- • Inefficient data processing pipelines
- • Unoptimized physics calculations

**Solutions**:
1. Optimize rendering settings:
   ```bash
   # Launch with reduced rendering quality
   gz sim -s --render-engine ogre2  # Use less demanding renderer

   # Or disable rendering for headless operation
   gz sim -s --render-engine none
   ```

2. Implement efficient resource management:
   ```python
   # Resource manager for simulation
   class ResourceManager:
       def __init__(self):
           self.active_objects = {}
           self.max_objects = 100

       def cleanup_old_objects(self):
           # Remove objects that haven't been accessed recently
           current_time = time.time()
           objects_to_remove = []

           for obj_id, obj_data in self.active_objects.items():
               if current_time - obj_data['last_access'] > 300:  # 5 minutes
                   objects_to_remove.append(obj_id)

           for obj_id in objects_to_remove:
               self.remove_object(obj_id)

       def limit_resource_usage(self):
           # Limit number of active objects
           if len(self.active_objects) > self.max_objects:
               # Remove oldest objects
               oldest_obj = min(self.active_objects.items(),
                              key=lambda x: x[1]['creation_time'])
               self.remove_object(oldest_obj[0])
   ```

3. Optimize physics calculations:
   ```xml
   <!-- Optimized physics for resource efficiency -->
   <physics name="ode_efficient" type="ode">
     <max_step_size>0.002</max_step_size>      <!-- Larger step for efficiency -->
     <real_time_update_rate>500</real_time_update_rate>  <!-- Lower update rate -->
     <gravity>0 0 -9.8</gravity>
     <ode>
       <solver>
         <type>quick</type>
         <iters>20</iters>                     <!-- Fewer iterations -->
         <sor>1.3</sor>
       </solver>
       <quick_step>
         <num_iterations>20</num_iterations>    <!-- Fewer quick step iterations -->
       </quick_step>
     </ode>
   </physics>
   ```

4. Use parallel processing where possible:
   ```python
   # Parallel processing for independent simulations
   from concurrent.futures import ThreadPoolExecutor
   import multiprocessing as mp

   class ParallelSimulationManager:
       def __init__(self, num_processes=None):
           if num_processes is None:
               num_processes = mp.cpu_count() - 2  # Leave some cores free
           self.executor = ThreadPoolExecutor(max_workers=num_processes)

       def run_parallel_simulations(self, simulation_configs):
           futures = []
           for config in simulation_configs:
               future = self.executor.submit(self.run_single_simulation, config)
               futures.append(future)

           results = [future.result() for future in futures]
           return results
   ```

**Verification Steps**:
- • [ ] CPU usage remains below 80% during normal operation
- • [ ] Memory usage is stable over time (no leaks)
- • [ ] GPU usage is reasonable for the rendering quality
- • [ ] Performance remains consistent over extended simulation runs

</details>

<h2 className="second-heading">
Multi-Robot Simulation Systems
</h2>
<div className="underline-class"></div>

<h3 className="third-heading">
- Coordinated Multi-Robot Environments
</h3>
<div className="underline-class"></div>

Creating simulation environments with multiple interacting robots requires careful coordination of physics, communication, and control systems:

```xml
<!-- Multi-robot world with coordination -->
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="multi_robot_lab">
    <!-- Common elements -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Robot 1: Humanoid assistant -->
    <model name="humanoid_1">
      <pose>-2 0 0.8 0 0 0</pose>
      <!-- Robot definition with sensors and actuators -->
      <link name="base_link">
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.15</radius>
              <length>0.8</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.15</radius>
              <length>0.8</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0.2 0.6 0.8 1</ambient>
            <diffuse>0.2 0.6 0.8 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>50</mass>
          <inertia>
            <ixx>1.0</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1.0</iyy>
            <iyz>0</iyz>
            <izz>1.0</izz>
          </inertia>
        </inertial>
      </link>

      <!-- Sensors -->
      <sensor name="camera_1" type="camera">
        <pose>0.1 0 0.3 0 0 0</pose>
        <camera name="camera">
          <horizontal_fov>1.047</horizontal_fov>
          <image>
            <width>640</width>
            <height>480</height>
          </image>
        </camera>
        <plugin name="camera_controller_1" filename="libgazebo_ros_camera.so">
          <ros>
            <namespace>/humanoid_1</namespace>
            <remapping>~/image_raw:=/camera/image_raw</remapping>
          </ros>
        </plugin>
      </sensor>

      <!-- Additional joints and links for humanoid structure -->
      <link name="head_link">
        <pose>0 0 0.4 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <sphere><radius>0.1</radius></sphere>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <sphere><radius>0.1</radius></sphere>
          </geometry>
        </visual>
        <inertial>
          <mass>2</mass>
          <inertia>
            <ixx>0.004</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.004</iyy>
            <iyz>0</iyz>
            <izz>0.004</izz>
          </inertia>
        </inertial>
      </link>

      <joint name="neck_joint" type="revolute">
        <parent>base_link</parent>
        <child>head_link</child>
        <axis>
          <xyz>0 0 1</xyz>
          <limit>
            <lower>-0.5</lower>
            <upper>0.5</upper>
            <effort>10</effort>
            <velocity>1</velocity>
          </limit>
        </axis>
      </joint>
    </model>

    <!-- Robot 2: Humanoid assistant -->
    <model name="humanoid_2">
      <pose>2 0 0.8 0 0 0</pose>
      <!-- Similar structure to robot 1 but with different namespace -->
      <link name="base_link">
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.15</radius>
              <length>0.8</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.15</radius>
              <length>0.8</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0.8 0.2 0.6 1</ambient>
            <diffuse>0.8 0.2 0.6 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>50</mass>
          <inertia>
            <ixx>1.0</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1.0</iyy>
            <iyz>0</iyz>
            <izz>1.0</izz>
          </inertia>
        </inertial>
      </link>

      <sensor name="camera_2" type="camera">
        <pose>0.1 0 0.3 0 0 0</pose>
        <camera name="camera">
          <horizontal_fov>1.047</horizontal_fov>
          <image>
            <width>640</width>
            <height>480</height>
          </image>
        </camera>
        <plugin name="camera_controller_2" filename="libgazebo_ros_camera.so">
          <ros>
            <namespace>/humanoid_2</namespace>
            <remapping>~/image_raw:=/camera/image_raw</remapping>
          </ros>
        </plugin>
      </sensor>

      <!-- Additional humanoid structure -->
      <link name="head_link">
        <pose>0 0 0.4 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <sphere><radius>0.1</radius></sphere>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <sphere><radius>0.1</radius></sphere>
          </geometry>
        </visual>
        <inertial>
          <mass>2</mass>
          <inertia>
            <ixx>0.004</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.004</iyy>
            <iyz>0</iyz>
            <izz>0.004</izz>
          </inertia>
        </inertial>
      </link>

      <joint name="neck_joint" type="revolute">
        <parent>base_link</parent>
        <child>head_link</child>
        <axis>
          <xyz>0 0 1</xyz>
          <limit>
            <lower>-0.5</lower>
            <upper>0.5</upper>
            <effort>10</effort>
            <velocity>1</velocity>
          </limit>
        </axis>
      </joint>
    </model>

    <!-- Interaction objects -->
    <model name="collaboration_table">
      <pose>0 -2 0.5 0 0 0</pose>
      <link name="table_link">
        <collision name="collision">
          <geometry>
            <box><size>1.5 1.0 0.8</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>1.5 1.0 0.8</size></box>
          </geometry>
          <material>
            <ambient>0.7 0.5 0.3 1</ambient>
            <diffuse>0.7 0.5 0.3 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>20</mass>
          <inertia>
            <ixx>2</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>3</iyy>
            <iyz>0</iyz>
            <izz>4</izz>
          </inertia>
        </inertial>
      </link>
    </model>
  </world>
</sdf>
```

<h3 className="third-heading">
- Coordination Algorithms
</h3>
<div className="underline-class"></div>

Implementing coordination algorithms for multi-robot systems:

```python
# Multi-robot coordination node
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import String
import math
import numpy as np

class MultiRobotCoordinator(Node):
    def __init__(self):
        super().__init__('multi_robot_coordinator')

        # Robot states (position and status)
        self.robot_states = {
            'humanoid_1': {'x': 0.0, 'y': 0.0, 'theta': 0.0, 'status': 'idle'},
            'humanoid_2': {'x': 0.0, 'y': 0.0, 'theta': 0.0, 'status': 'idle'}
        }

        # Publishers for each robot
        self.cmd_publishers = {}
        for robot_name in self.robot_states:
            self.cmd_publishers[robot_name] = self.create_publisher(
                Twist,
                f'/{robot_name}/cmd_vel',
                10
            )

        # Subscribers for robot odometry
        for robot_name in self.robot_states:
            self.create_subscription(
                Pose,
                f'/{robot_name}/pose',
                lambda msg, name=robot_name: self.pose_callback(msg, name),
                10
            )

        # Task assignment publisher
        self.task_publisher = self.create_publisher(
            String,
            '/task_assignment',
            10
        )

        # Timer for coordination logic
        self.timer = self.create_timer(0.5, self.coordination_logic)

        self.get_logger().info('Multi-robot coordinator initialized')

    def pose_callback(self, msg, robot_name):
        # Update robot state from pose message
        self.robot_states[robot_name]['x'] = msg.position.x
        self.robot_states[robot_name]['y'] = msg.position.y
        # Extract orientation (simplified)
        self.robot_states[robot_name]['theta'] = 0.0  # Would extract from quaternion

    def coordination_logic(self):
        # Simple coordination: assign different tasks to robots
        # Calculate distances between robots
        pos1 = np.array([self.robot_states['humanoid_1']['x'], self.robot_states['humanoid_1']['y']])
        pos2 = np.array([self.robot_states['humanoid_2']['x'], self.robot_states['humanoid_2']['y']])

        distance = np.linalg.norm(pos1 - pos2)

        # If robots are too close, make them move apart
        if distance < 1.0:
            # Generate separation commands
            direction = pos2 - pos1
            direction = direction / np.linalg.norm(direction) if np.linalg.norm(direction) > 0 else np.array([1, 0])

            # Send separation commands
            cmd1 = Twist()
            cmd1.linear.x = -0.2 * direction[0]
            cmd1.linear.y = -0.2 * direction[1]
            self.cmd_publishers['humanoid_1'].publish(cmd1)

            cmd2 = Twist()
            cmd2.linear.x = 0.2 * direction[0]
            cmd2.linear.y = 0.2 * direction[1]
            self.cmd_publishers['humanoid_2'].publish(cmd2)

        # Task assignment based on proximity to objects
        self.assign_tasks()

    def assign_tasks(self):
        # Example: assign tasks based on proximity to objects
        object_positions = [
            np.array([2, 2]),   # Object 1
            np.array([-2, -2])  # Object 2
        ]

        for i, obj_pos in enumerate(object_positions):
            # Calculate distances from each robot to object
            dist1 = np.linalg.norm(obj_pos - np.array([self.robot_states['humanoid_1']['x'],
                                                      self.robot_states['humanoid_1']['y']]))
            dist2 = np.linalg.norm(obj_pos - np.array([self.robot_states['humanoid_2']['x'],
                                                      self.robot_states['humanoid_2']['y']]))

            # Assign to closer robot
            if dist1 < dist2:
                task_msg = String()
                task_msg.data = f'humanoid_1:task_{i}'
                self.task_publisher.publish(task_msg)
            else:
                task_msg = String()
                task_msg.data = f'humanoid_2:task_{i}'
                self.task_publisher.publish(task_msg)

def main(args=None):
    rclpy.init(args=args)
    coordinator = MultiRobotCoordinator()

    try:
        rclpy.spin(coordinator)
    except KeyboardInterrupt:
        pass
    finally:
        coordinator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

<h2 className="second-heading">
Advanced Physics Models
</h2>
<div className="underline-class"></div>

<h3 className="third-heading">
- Realistic Contact Dynamics
</h3>
<div className="underline-class"></div>

Implementing advanced contact models for humanoid robots:

```xml
<!-- Advanced physics configuration -->
<world name="advanced_physics_world">
  <!-- Physics engine with advanced parameters -->
  <physics name="ode_advanced" type="ode">
    <max_step_size>0.001</max_step_size>
    <real_time_factor>1.0</real_time_factor>
    <real_time_update_rate>1000</real_time_update_rate>
    <gravity>0 0 -9.8</gravity>

    <ode>
      <solver>
        <type>quick</type>
        <iters>100</iters>
        <sor>1.3</sor>
      </solver>
      <constraints>
        <cfm>1e-5</cfm>
        <erp>0.2</erp>
        <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
        <contact_surface_layer>0.001</contact_surface_layer>
      </constraints>
    </ode>
  </physics>

  <!-- Robot with advanced surface properties -->
  <model name="advanced_robot">
    <link name="foot_link">
      <collision name="collision">
        <geometry>
          <box><size>0.2 0.1 0.05</size></box>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>0.9</mu>
              <mu2>0.9</mu2>
              <fdir1>0 0 0</fdir1>
              <slip1>0.001</slip1>
              <slip2>0.001</slip2>
            </ode>
            <torsional>
              <coefficient>0.8</coefficient>
              <use_patch_radius>true</use_patch_radius>
              <surface_radius>0.02</surface_radius>
              <patch_radius>0.02</patch_radius>
            </torsional>
          </friction>
          <bounce>
            <restitution_coefficient>0.1</restitution_coefficient>
            <threshold>100000</threshold>
          </bounce>
          <contact>
            <ode>
              <soft_cfm>0.001</soft_cfm>
              <soft_erp>0.8</soft_erp>
              <kp>1e+6</kp>
              <kd>10</kd>
              <max_vel>100.0</max_vel>
              <min_depth>0.0001</min_depth>
            </ode>
          </contact>
        </surface>
      </collision>
    </link>
  </model>
</world>
```

<h3 className="third-heading">
- Flexible Body Dynamics
</h3>
<div className="underline-class"></div>

Modeling flexible components for more realistic simulation:

```xml
<!-- Flexible body model (conceptual - using multiple rigid bodies with springs) -->
<model name="flexible_robot">
  <!-- Flexible spine using multiple segments -->
  <link name="spine_base">
    <pose>0 0 1.0 0 0 0</pose>
    <collision name="collision">
      <geometry>
        <cylinder>
          <radius>0.05</radius>
          <length>0.2</length>
        </cylinder>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <cylinder>
          <radius>0.05</radius>
          <length>0.2</length>
        </cylinder>
      </geometry>
      <material>
        <ambient>0.6 0.6 0.6 1</ambient>
        <diffuse>0.6 0.6 0.6 1</diffuse>
      </material>
    </visual>
    <inertial>
      <mass>2</mass>
      <inertia>
        <ixx>0.02</ixx>
        <ixy>0</ixy>
        <ixz>0</ixz>
        <iyy>0.02</iyy>
        <iyz>0</iyz>
        <izz>0.005</izz>
      </inertia>
    </inertial>
  </link>

  <link name="spine_middle">
    <pose>0 0 1.2 0 0 0</pose>
    <collision name="collision">
      <geometry>
        <cylinder>
          <radius>0.05</radius>
          <length>0.2</length>
        </cylinder>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <cylinder>
          <radius>0.05</radius>
          <length>0.2</length>
        </cylinder>
      </geometry>
      <material>
        <ambient>0.6 0.6 0.6 1</ambient>
        <diffuse>0.6 0.6 0.6 1</diffuse>
      </material>
    </visual>
    <inertial>
      <mass>2</mass>
      <inertia>
        <ixx>0.02</ixx>
        <ixy>0</ixy>
        <ixz>0</ixz>
        <iyy>0.02</iyy>
        <iyz>0</iyz>
        <izz>0.005</izz>
      </inertia>
    </inertial>
  </link>

  <link name="spine_top">
    <pose>0 0 1.4 0 0 0</pose>
    <collision name="collision">
      <geometry>
        <cylinder>
          <radius>0.05</radius>
          <length>0.2</length>
        </cylinder>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <cylinder>
          <radius>0.05</radius>
          <length>0.2</length>
        </cylinder>
      </geometry>
      <material>
        <ambient>0.6 0.6 0.6 1</ambient>
        <diffuse>0.6 0.6 0.6 1</diffuse>
      </material>
    </visual>
    <inertial>
      <mass>2</mass>
      <inertia>
        <ixx>0.02</ixx>
        <ixy>0</ixy>
        <ixz>0</ixz>
        <iyy>0.02</iyy>
        <iyz>0</iyz>
        <izz>0.005</izz>
      </inertia>
    </inertial>
  </link>

  <!-- Joints with spring-like properties -->
  <joint name="spine_lower_joint" type="ball">
    <parent>spine_base</parent>
    <child>spine_middle</child>
    <axis>
      <xyz>1 0 0</xyz>
      <limit>
        <lower>-0.2</lower>
        <upper>0.2</upper>
        <effort>50</effort>
        <velocity>2</velocity>
      </limit>
    </axis>
  </joint>

  <joint name="spine_upper_joint" type="ball">
    <parent>spine_middle</parent>
    <child>spine_top</child>
    <axis>
      <xyz>1 0 0</xyz>
      <limit>
        <lower>-0.2</lower>
        <upper>0.2</upper>
        <effort>50</effort>
        <velocity>2</velocity>
      </limit>
    </axis>
  </joint>
</model>
```

<h2 className="second-heading">
Dynamic Environment Simulation
</h2>
<div className="underline-class"></div>

<h3 className="third-heading">
- Interactive Environments
</h3>
<div className="underline-class"></div>

Creating environments that respond to robot actions:

```xml
<!-- Dynamic environment with interactive elements -->
<world name="interactive_environment">
  <!-- Physics engine -->
  <physics name="ode" type="ode">
    <max_step_size>0.001</max_step_size>
    <real_time_factor>1</real_time_factor>
    <gravity>0 0 -9.8</gravity>
  </physics>

  <!-- Movable furniture -->
  <model name="movable_table">
    <pose>2 1 0.5 0 0 0</pose>
    <link name="table_base">
      <collision name="collision">
        <geometry>
          <box><size>1.0 0.6 0.8</size></box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box><size>1.0 0.6 0.8</size></box>
        </geometry>
        <material>
          <ambient>0.6 0.4 0.2 1</ambient>
          <diffuse>0.6 0.4 0.2 1</diffuse>
        </material>
      </visual>
      <inertial>
        <mass>20</mass>
        <inertia>
          <ixx>2</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>3</iyy>
          <iyz>0</iyz>
          <izz>4</izz>
        </inertia>
      </inertial>
    </link>

    <!-- Attachable object on table -->
    <model name="object_on_table">
      <pose>2 1 1.0 0 0 0</pose>
      <link name="object_link">
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.05</radius>
              <length>0.2</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.05</radius>
              <length>0.2</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0.9 0.1 0.1 1</ambient>
            <diffuse>0.9 0.1 0.1 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>0.5</mass>
          <inertia>
            <ixx>0.001</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.001</iyy>
            <iyz>0</iyz>
            <izz>0.0005</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <!-- Door that can be opened -->
    <model name="door">
      <pose>0 3 1 0 0 0</pose>
      <link name="door_frame">
        <static>true</static>
        <collision name="collision">
          <geometry>
            <box><size>0.1 2 2</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>0.1 2 2</size></box>
          </geometry>
          <material>
            <ambient>0.4 0.2 0.1 1</ambient>
            <diffuse>0.4 0.2 0.1 1</diffuse>
          </material>
        </visual>
      </link>

      <link name="door_panel">
        <collision name="collision">
          <geometry>
            <box><size>0.05 1.8 2</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>0.05 1.8 2</size></box>
          </geometry>
          <material>
            <ambient>0.5 0.3 0.2 1</ambient>
            <diffuse>0.5 0.3 0.2 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>10</mass>
          <inertia>
            <ixx>3</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.1</iyy>
            <iyz>0</iyz>
            <izz>3.1</izz>
          </inertia>
        </inertial>
      </link>

      <joint name="door_hinge" type="revolute">
        <parent>door_frame</parent>
        <child>door_panel</child>
        <axis>
          <xyz>0 1 0</xyz>
          <limit>
            <lower>-1.57</lower>
            <upper>0</upper>
            <effort>100</effort>
            <velocity>1</velocity>
          </limit>
        </axis>
      </joint>

      <!-- Joint controller for door -->
      <plugin name="door_controller" filename="libgazebo_ros_joint_pose_trajectory.so">
        <ros>
          <namespace>/environment</namespace>
        </ros>
        <command_topic>door_control</command_topic>
        <joint_name>door_hinge</joint_name>
      </plugin>
    </model>
  </model>
</world>
```

<h3 className="third-heading">
- Environmental Effects
</h3>
<div className="underline-class"></div>

Simulating environmental conditions that affect robot performance:

```xml
<!-- Environment with weather effects -->
<world name="weather_simulation">
  <!-- Atmosphere -->
  <atmosphere type="adiabatic">
    <temperature>288.15</temperature>
    <pressure>101325</pressure>
  </atmosphere>

  <!-- Wind -->
  <wind>
    <linear_velocity>0.5 0.2 0</linear_velocity>
  </wind>

  <!-- Lighting conditions -->
  <light name="main_light" type="directional">
    <pose>0 0 10 0 0 0</pose>
    <diffuse>0.8 0.8 0.7 1</diffuse>
    <specular>0.2 0.2 0.2 1</specular>
    <direction>-0.3 -0.3 -0.9</direction>
  </light>

  <!-- Rain effect (simulated with particles) -->
  <model name="rain_effect">
    <static>true</static>
    <link name="rain_link">
      <visual name="rain_visual">
        <particle_emitter name="rain_emitter">
          <type>box</type>
          <size>10 10 0.1</size>
          <pos>0 0 5</pos>
          <min_velocity>10</min_velocity>
          <max_velocity>15</max_velocity>
          <lifetime>1</lifetime>
          <min_quantity>100</min_quantity>
          <max_quantity>200</max_quantity>
          <color>
            <r>0.8</r>
            <g>0.8</g>
            <b>1.0</b>
            <a>0.7</a>
          </color>
          <size>0.005 0.005 0.05</size>
          <particle_size>0.005 0.005 0.05</particle_size>
          <material>Gazebo/BlueGlow</material>
        </particle_emitter>
      </visual>
    </link>
  </model>

  <!-- Slippery surface -->
  <model name="wet_surface">
    <static>true</static>
    <pose>0 0 0 0 0 0</pose>
    <link name="wet_link">
      <collision name="collision">
        <geometry>
          <box><size>5 5 0.1</size></box>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>0.2</mu>
              <mu2>0.2</mu2>
            </ode>
          </friction>
        </surface>
      </collision>
      <visual name="visual">
        <geometry>
          <box><size>5 5 0.1</size></box>
        </geometry>
        <material>
          <ambient>0.3 0.3 0.4 1</ambient>
          <diffuse>0.3 0.3 0.4 1</diffuse>
          <specular>0.8 0.8 0.8 1</specular>
        </material>
      </visual>
    </link>
  </model>
</world>
```

<h2 className="second-heading">
Performance Optimization Techniques
</h2>
<div className="underline-class"></div>

<h3 className="third-heading">
- Level of Detail (LOD) Systems
</h3>
<div className="underline-class"></div>

Implementing performance optimization through level of detail:

```python
# LOD controller for simulation optimization
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import PointStamped
import math

class LODController(Node):
    def __init__(self):
        super().__init__('lod_controller')

        # Robot position subscriber
        self.position_subscriber = self.create_subscription(
            PointStamped,
            '/robot_position',
            self.position_callback,
            10
        )

        # LOD level publisher
        self.lod_publisher = self.create_publisher(
            Float32,
            '/lod_level',
            10
        )

        # Objects to optimize
        self.objects = [
            {'name': 'detailed_building', 'position': (10, 10), 'distance_thresholds': [5, 10, 20]},
            {'name': 'detailed_tree', 'position': (15, 5), 'distance_thresholds': [3, 6, 12]},
            {'name': 'detailed_car', 'position': (20, 15), 'distance_thresholds': [2, 5, 10]}
        ]

        # Timer for periodic optimization
        self.timer = self.create_timer(1.0, self.optimize_lod)

        self.robot_position = (0, 0)
        self.get_logger().info('LOD controller initialized')

    def position_callback(self, msg):
        # Update robot position
        self.robot_position = (msg.point.x, msg.point.y)

    def optimize_lod(self):
        # Calculate distances to objects
        for obj in self.objects:
            obj_pos = obj['position']
            distance = math.sqrt(
                (self.robot_position[0] - obj_pos[0])**2 +
                (self.robot_position[1] - obj_pos[1])**2
            )

            # Determine LOD level based on distance
            if distance < obj['distance_thresholds'][0]:
                lod_level = 0  # High detail
            elif distance < obj['distance_thresholds'][1]:
                lod_level = 1  # Medium detail
            elif distance < obj['distance_thresholds'][2]:
                lod_level = 2  # Low detail
            else:
                lod_level = 3  # Very low detail or invisible

            # Publish LOD level for this object
            lod_msg = Float32()
            lod_msg.data = float(lod_level)

            # In a real implementation, you would send this to a service
            # that controls the rendering detail of the object
            self.get_logger().info(f'{obj["name"]} LOD level: {lod_level}, Distance: {distance:.2f}')

def main(args=None):
    rclpy.init(args=args)
    lod_controller = LODController()

    try:
        rclpy.spin(lod_controller)
    except KeyboardInterrupt:
        pass
    finally:
        lod_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

<h3 className="third-heading">
- Parallel Simulation
</h3>
<div className="underline-class"></div>

Running multiple simulation instances for scalability:

```python
# Parallel simulation manager
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
import subprocess
import threading
import time

class ParallelSimulationManager(Node):
    def __init__(self):
        super().__init__('parallel_sim_manager')

        # Publisher for simulation status
        self.status_publisher = self.create_publisher(
            String,
            '/simulation_status',
            10
        )

        # Subscriber for simulation commands
        self.command_subscriber = self.create_subscription(
            String,
            '/simulation_command',
            self.command_callback,
            10
        )

        # Track simulation instances
        self.simulation_instances = {}
        self.instance_count = 0

        # Timer for monitoring instances
        self.monitor_timer = self.create_timer(2.0, self.monitor_instances)

        self.get_logger().info('Parallel simulation manager initialized')

    def command_callback(self, msg):
        command = msg.data
        if command.startswith('start_sim_'):
            # Extract environment name
            env_name = command.split('_')[2]  # start_sim_env1 -> env1
            self.start_simulation_instance(env_name)
        elif command.startswith('stop_sim_'):
            env_name = command.split('_')[2]
            self.stop_simulation_instance(env_name)

    def start_simulation_instance(self, env_name):
        # Create a new simulation instance
        instance_id = f"sim_{env_name}_{self.instance_count}"
        self.instance_count += 1

        # Launch simulation in a separate thread/process
        # In practice, you would use system commands to launch Gazebo instances
        def launch_simulation():
            self.get_logger().info(f'Launching simulation instance: {instance_id}')
            # Example command (would need to be adapted to your system):
            # cmd = f"gazebo --verbose worlds/{env_name}.world"
            # process = subprocess.Popen(cmd, shell=True)
            # self.simulation_instances[instance_id] = process

        # Start simulation in background thread
        thread = threading.Thread(target=launch_simulation)
        thread.daemon = True
        thread.start()

        self.simulation_instances[instance_id] = {
            'status': 'starting',
            'thread': thread,
            'start_time': time.time()
        }

        status_msg = String()
        status_msg.data = f'Started simulation: {instance_id}'
        self.status_publisher.publish(status_msg)

    def stop_simulation_instance(self, env_name):
        # Find and stop simulation instances with the given name
        instances_to_stop = [
            key for key in self.simulation_instances.keys()
            if env_name in key
        ]

        for instance_id in instances_to_stop:
            if instance_id in self.simulation_instances:
                # In practice, you would terminate the process
                self.get_logger().info(f'Stopping simulation instance: {instance_id}')
                del self.simulation_instances[instance_id]

    def monitor_instances(self):
        # Monitor running instances
        running_count = len(self.simulation_instances)
        status_msg = String()
        status_msg.data = f'Running simulations: {running_count}'
        self.status_publisher.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    sim_manager = ParallelSimulationManager()

    try:
        rclpy.spin(sim_manager)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up simulation instances
        for instance_id in list(sim_manager.simulation_instances.keys()):
            sim_manager.stop_simulation_instance(instance_id.split('_')[1])  # Extract env name

        sim_manager.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

<h2 className="second-heading">
Simulation-to-Reality Transfer
</h2>
<div className="underline-class"></div>

<h3 className="third-heading">
- Domain Randomization
</h3>
<div className="underline-class"></div>

Implementing domain randomization to improve reality transfer:

```python
# Domain randomization node
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from gazebo_msgs.srv import SetPhysicsProperties
import random

class DomainRandomizer(Node):
    def __init__(self):
        super().__init__('domain_randomizer')

        # Service client for physics properties
        self.physics_client = self.create_client(
            SetPhysicsProperties,
            '/set_physics_properties'
        )

        # Wait for service
        while not self.physics_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for physics service...')

        # Timer for randomization
        self.randomization_timer = self.create_timer(10.0, self.randomize_environment)

        self.get_logger().info('Domain randomizer initialized')

    def randomize_environment(self):
        # Randomize physics parameters
        gravity_variation = random.uniform(-1.0, 1.0)
        time_step_variation = random.uniform(0.0005, 0.002)
        friction_base = random.uniform(0.4, 1.2)

        # Create physics properties request
        req = SetPhysicsProperties.Request()
        req.time_step = time_step_variation
        req.max_step_size = time_step_variation
        req.real_time_factor = random.uniform(0.8, 1.2)
        req.real_time_update_rate = random.uniform(900, 1100)

        # Adjust gravity
        req.gravity.x = random.uniform(-0.1, 0.1)
        req.gravity.y = random.uniform(-0.1, 0.1)
        req.gravity.z = -9.8 + gravity_variation

        # Set ODE parameters
        req.ode_config.sor = random.uniform(1.2, 1.5)
        req.ode_config.erp = random.uniform(0.1, 0.3)
        req.ode_config.cfm = random.uniform(1e-6, 1e-4)
        req.ode_config.contact_surface_layer = random.uniform(0.0005, 0.002)
        req.ode_config.contact_max_correcting_vel = random.uniform(50, 200)

        # Send request
        future = self.physics_client.call_async(req)
        future.add_done_callback(self.physics_randomization_callback)

    def physics_randomization_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Physics properties randomized successfully')
            else:
                self.get_logger().error(f'Failed to randomize physics: {response.status_message}')
        except Exception as e:
            self.get_logger().error(f'Error randomizing physics: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    randomizer = DomainRandomizer()

    try:
        rclpy.spin(randomizer)
    except KeyboardInterrupt:
        pass
    finally:
        randomizer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

<h3 className="third-heading">
- Sensor Noise Modeling
</h3>
<div className="underline-class"></div>

Implementing realistic sensor noise for better transfer:

```python
# Advanced sensor noise model
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu, Image
from cv_bridge import CvBridge
import numpy as np
import cv2

class SensorNoiseModel(Node):
    def __init__(self):
        super().__init__('sensor_noise_model')

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Subscribers for clean sensor data
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/clean_lidar',
            self.lidar_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/clean_imu',
            self.imu_callback,
            10
        )

        self.camera_sub = self.create_subscription(
            Image,
            '/clean_camera',
            self.camera_callback,
            10
        )

        # Publishers for noisy sensor data
        self.noisy_lidar_pub = self.create_publisher(
            LaserScan,
            '/noisy_lidar',
            10
        )

        self.noisy_imu_pub = self.create_publisher(
            Imu,
            '/noisy_imu',
            10
        )

        self.noisy_camera_pub = self.create_publisher(
            Image,
            '/noisy_camera',
            10
        )

        self.get_logger().info('Sensor noise model initialized')

    def lidar_callback(self, msg):
        # Add realistic noise to LIDAR data
        noisy_msg = LaserScan()
        noisy_msg.header = msg.header
        noisy_msg.angle_min = msg.angle_min
        noisy_msg.angle_max = msg.angle_max
        noisy_msg.angle_increment = msg.angle_increment
        noisy_msg.time_increment = msg.time_increment
        noisy_msg.scan_time = msg.scan_time
        noisy_msg.range_min = msg.range_min
        noisy_msg.range_max = msg.range_max

        # Add noise to ranges
        ranges = np.array(msg.ranges)
        # Add Gaussian noise with distance-dependent variance
        distance_dependent_noise = 0.01 + 0.005 * ranges  # More noise at longer distances
        noise = np.random.normal(0, distance_dependent_noise, size=ranges.shape)
        noisy_ranges = ranges + noise

        # Ensure valid range values
        noisy_ranges = np.clip(noisy_ranges, msg.range_min, msg.range_max)
        noisy_msg.ranges = noisy_ranges.tolist()

        # Add noise to intensities if available
        if msg.intensities:
            intensities = np.array(msg.intensities)
            intensity_noise = np.random.normal(0, 0.1 * intensities, size=intensities.shape)
            noisy_intensities = intensities + intensity_noise
            noisy_msg.intensities = np.clip(noisy_intensities, 0, None).tolist()
        else:
            noisy_msg.intensities = msg.intensities

        self.noisy_lidar_pub.publish(noisy_msg)

    def imu_callback(self, msg):
        # Add realistic noise to IMU data
        noisy_msg = Imu()
        noisy_msg.header = msg.header

        # Add noise to orientation (if provided)
        if msg.orientation.x != 0 or msg.orientation.y != 0 or msg.orientation.z != 0 or msg.orientation.w != 0:
            # Add small random rotation
            noise_roll = np.random.normal(0, 0.01)  # 0.01 rad = ~0.6 deg
            noise_pitch = np.random.normal(0, 0.01)
            noise_yaw = np.random.normal(0, 0.02)

            # Convert to quaternion noise (simplified)
            noisy_msg.orientation.x = msg.orientation.x + noise_roll * 0.1
            noisy_msg.orientation.y = msg.orientation.y + noise_pitch * 0.1
            noisy_msg.orientation.z = msg.orientation.z + noise_yaw * 0.1
            noisy_msg.orientation.w = msg.orientation.w  # Keep w component normalized

        # Add noise to angular velocity
        angular_noise_std = 0.001  # rad/s
        noisy_msg.angular_velocity.x = msg.angular_velocity.x + np.random.normal(0, angular_noise_std)
        noisy_msg.angular_velocity.y = msg.angular_velocity.y + np.random.normal(0, angular_noise_std)
        noisy_msg.angular_velocity.z = msg.angular_velocity.z + np.random.normal(0, angular_noise_std)

        # Add noise to linear acceleration
        accel_noise_std = 0.01  # m/s²
        noisy_msg.linear_acceleration.x = msg.linear_acceleration.x + np.random.normal(0, accel_noise_std)
        noisy_msg.linear_acceleration.y = msg.linear_acceleration.y + np.random.normal(0, accel_noise_std)
        noisy_msg.linear_acceleration.z = msg.linear_acceleration.z + np.random.normal(0, accel_noise_std)

        self.noisy_imu_pub.publish(noisy_msg)

    def camera_callback(self, msg):
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Add realistic camera noise
            # 1. Gaussian noise
            gaussian_noise = np.random.normal(0, 10, cv_image.shape).astype(np.float32)
            noisy_image = cv_image.astype(np.float32) + gaussian_noise

            # 2. Shot noise (photon noise)
            # Convert to photon counts (simplified)
            photon_counts = np.maximum(noisy_image / 255.0 * 1000, 1)  # Max 1000 photons per pixel
            shot_noise = np.random.poisson(photon_counts) / 1000 * 255
            noisy_image = shot_noise

            # 3. Quantization noise
            noisy_image = np.clip(noisy_image, 0, 255).astype(np.uint8)

            # Convert back to ROS image
            noisy_msg = self.bridge.cv2_to_imgmsg(noisy_image, encoding='bgr8')
            noisy_msg.header = msg.header

            self.noisy_camera_pub.publish(noisy_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing camera image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    noise_model = SensorNoiseModel()

    try:
        rclpy.spin(noise_model)
    except KeyboardInterrupt:
        pass
    finally:
        noise_model.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

<h2 className="second-heading">
Best Practices for Advanced Simulation
</h2>
<div className="underline-class"></div>

<h3 className="third-heading">
- Architecture Design
</h3>
<div className="underline-class"></div>

Designing scalable simulation architectures:

1. **Modular components**: Keep simulation elements modular and reusable
2. **Resource management**: Efficiently manage computational resources
3. **Data flow optimization**: Minimize unnecessary data transmission
4. **Parallel processing**: Utilize multi-core systems effectively

<h3 className="third-heading">
- Validation Strategies
</h3>
<div className="underline-class"></div>

Validating simulation accuracy:

1. **Cross-validation**: Compare simulation results with real-world data
2. **Sensitivity analysis**: Test how sensitive results are to parameter changes
3. **Statistical validation**: Use statistical methods to validate distributions
4. **Benchmarking**: Compare against known benchmarks and standards

<h2 className="second-heading">
Troubleshooting Advanced Simulation Issues
</h2>
<div className="underline-class"></div>

<h3 className="third-heading">
- Performance Problems
</h3>
<div className="underline-class"></div>

**Problem**: Simulation performance degrades with complex environments
**Solutions**:
- • Implement level-of-detail (LOD) systems
- • Use spatial partitioning for collision detection
- • Optimize physics parameters
- • Consider parallel simulation instances

**Problem**: Multi-robot coordination fails at scale
**Solutions**:
- • Implement hierarchical coordination
- • Use distributed computing approaches
- • Optimize communication protocols
- • Apply load balancing techniques

<h3 className="third-heading">
- Realism Issues
</h3>
<div className="underline-class"></div>

**Problem**: Simulation behavior doesn't match real-world behavior
**Solutions**:
- • Validate sensor models against real hardware
- • Fine-tune physics parameters
- • Implement domain randomization
- • Add realistic noise models

<h2 className="second-heading">
Summary
</h2>
<div className="underline-class"></div>

Advanced simulation techniques enable the creation of complex, realistic environments for testing humanoid robots. By implementing multi-robot coordination, advanced physics models, dynamic environments, and performance optimization techniques, developers can create simulation systems that closely mirror real-world challenges. The key to successful advanced simulation lies in balancing computational efficiency with physical accuracy while maintaining the ability to transfer learned behaviors to real robots.

import ReadingTime from '@site/src/components/ReadingTime';
import ViewToggle from '@site/src/components/ViewToggle';

<ReadingTime minutes={20} />
<ViewToggle />