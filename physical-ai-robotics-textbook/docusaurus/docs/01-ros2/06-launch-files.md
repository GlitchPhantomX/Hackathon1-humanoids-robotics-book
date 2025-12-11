---
sidebar_position: 7
title: 'Launch Files: Managing Complex Robot Systems'
description: 'Understanding how to create and use launch files to manage complex ROS 2 robot systems'
---

# Launch Files: Managing Complex Robot Systems

Launch files are essential for managing complex robotic systems by allowing you to start multiple nodes with specific configurations simultaneously. This chapter covers creating and using launch files for humanoid robotics applications.

## Learning Objectives

By the end of this chapter, you will be able to:
- Create launch files to start multiple nodes simultaneously
- Use parameters, arguments, and conditions in launch files
- Launch different robot configurations based on arguments
- Integrate launch files with robot description packages
- Debug and troubleshoot launch file issues

## Exercises

<details>
<summary>Exercise 1.6.1: Basic Launch File Creation (⭐, ~25 min)</summary>

### Exercise 1.6.1: Basic Launch File Creation
**Difficulty**: ⭐ (Beginner)
**Time Estimate**: 25 minutes
**Requirements**: ROS 2 environment, basic Python knowledge

#### Starter Code
Create a basic launch file named `simple_robot.launch.py` that launches:
- A talker node from `demo_nodes_cpp`
- A listener node from `demo_nodes_cpp`
- Set the output to screen for both nodes

#### Success Criteria
- [ ] Launch file creates a proper LaunchDescription
- [ ] Both nodes start successfully when launch file is executed
- [ ] Nodes communicate properly (messages pass between talker and listener)
- [ ] Output appears on screen as expected
- [ ] Launch file follows ROS 2 launch file conventions

#### Test Commands
```bash
# Create the launch file
mkdir -p ~/ros2_ws/src/my_launch_package/launch
# (then create the launch file with your code)

# Run the launch file
ros2 launch my_launch_package simple_robot.launch.py

# Verify nodes are running
ros2 node list
```

#### Expected Output
- Both talker and listener nodes should appear in the node list
- Messages should flow from talker to listener
- Console output should show both nodes communicating

#### Challenges
- Add a third node that processes the messages
- Modify the node names to be more descriptive

#### Hints
- Use the launch and launch_ros modules for creating actions
- Remember to return a LaunchDescription with your actions
- Use proper package and executable names for the nodes

</details>

<details>
<summary>Exercise 1.6.2: Launch File with Arguments (⭐⭐, ~40 min)</summary>

### Exercise 1.6.2: Launch File with Arguments
**Difficulty**: ⭐⭐ (Intermediate)
**Time Estimate**: 40 minutes
**Requirements**: Understanding of basic launch files, ROS 2 parameters

#### Starter Code
Create a launch file named `configurable_robot.launch.py` that includes:
- Launch arguments for `use_sim_time`, `robot_name`, and `launch_rviz`
- Conditional launching of RViz based on the `launch_rviz` argument
- Robot state publisher with parameters from launch arguments
- Node remapping capabilities

#### Success Criteria
- [ ] Launch arguments are properly declared with default values
- [ ] RViz launches conditionally based on argument value
- [ ] Robot state publisher uses arguments for configuration
- [ ] Launch file works with different argument combinations
- [ ] All nodes start correctly with proper configurations

#### Test Commands
```bash
# Run with default arguments
ros2 launch my_launch_package configurable_robot.launch.py

# Run with custom arguments
ros2 launch my_launch_package configurable_robot.launch.py robot_name:=my_robot use_sim_time:=true launch_rviz:=false

# Verify parameters are set correctly
ros2 param list /robot_state_publisher
```

#### Expected Output
- Launch file should work with default values
- Different argument combinations should produce expected behavior
- Parameter values should reflect the launch arguments

#### Challenges
- Add more complex conditional logic based on multiple arguments
- Include a parameter file that's loaded conditionally

#### Hints
- Use LaunchConfiguration to reference declared arguments
- Use IfCondition and UnlessCondition for conditional actions
- Remember to include all declared arguments in the LaunchDescription

</details>

<details>
<summary>Exercise 1.6.3: Advanced Launch System (⭐⭐⭐, ~60 min)</summary>

### Exercise 1.6.3: Advanced Launch System
**Difficulty**: ⭐⭐⭐ (Advanced)
**Time Estimate**: 60 minutes
**Requirements**: Complete understanding of launch files, robot systems integration

#### Starter Code
Create a comprehensive launch system for a humanoid robot that includes:
- Main launch file that includes other launch files
- Robot controller group with namespace
- Simulation launch file that starts Gazebo
- Parameter file integration
- Error handling and validation

#### Success Criteria
- [ ] Main launch file properly includes other components
- [ ] Robot controllers launch in appropriate namespace
- [ ] Simulation launch works with Gazebo
- [ ] Parameter files are correctly loaded and applied
- [ ] Launch system handles errors gracefully
- [ ] All components work together seamlessly

#### Test Commands
```bash
# Test main robot launch
ros2 launch my_robot_bringup humanoid_robot.launch.py

# Test simulation launch
ros2 launch my_robot_gazebo humanoid_gazebo.launch.py

# Verify all components are working
ros2 node list
ros2 param list
ros2 topic list
```

#### Expected Output
- Complete robot system should launch successfully
- All controllers should be in proper namespaces
- Simulation should start with robot properly spawned
- Parameters should be correctly configured

#### Challenges
- Add automatic controller loading after robot spawn
- Implement safety checks before launching critical nodes

#### Hints
- Break the system into smaller, composable launch files
- Use proper error handling and validation
- Test each component separately before integration

</details>

<details>
<summary>Exercise Summary</summary>

### Exercise Summary
This chapter covered creating and using launch files to manage complex ROS 2 robot systems. You learned to create basic launch files, use parameters and arguments, conditionally launch nodes, and build comprehensive launch systems for humanoid robots. The exercises provided hands-on experience with basic launch creation, argument handling, and advanced system integration.

</details>

## Troubleshooting

<details>
<summary>Troubleshooting: Launch File Issues</summary>

### Troubleshooting: Launch File Issues

#### Problem: Nodes don't start or crash immediately
**Symptoms**:
- Launch file runs but nodes don't appear in node list
- Error messages about missing executables or packages
- Launch process terminates with errors

**Causes**:
- Incorrect package or executable names
- Package not built or sourced properly
- Missing dependencies

**Solutions**:
1. Verify package and executable names are correct:
   ```python
   Node(
       package='correct_package_name',      # Check this is correct
       executable='correct_executable_name', # And this too
       name='node_name'
   )
   ```
2. Ensure the package is built and sourced:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select correct_package_name
   source install/setup.bash
   ```
3. Use `output='screen'` to see detailed error messages:
   ```python
   Node(
       package='package_name',
       executable='executable_name',
       output='screen'  # This will show detailed errors
   )
   ```

**Verification Steps**:
- [ ] Package name matches exactly what's in package.xml
- [ ] Executable name matches what's defined in setup.py/CMakeLists.txt
- [ ] Package is properly built and sourced

#### Problem: Parameters not being set correctly
**Symptoms**:
- Nodes start but don't behave as expected
- Parameter values are not applied
- Nodes fail due to missing required parameters

**Causes**:
- Incorrect parameter file paths
- Parameter names don't match node expectations
- Parameter files have YAML syntax errors

**Solutions**:
1. Verify parameter file paths are correct:
   ```python
   Node(
       package='package_name',
       executable='executable_name',
       parameters=[
           '/absolute/path/to/params.yaml',  # Or use get_package_share_directory
           {'param_name': 'param_value'}     # Direct parameter assignment
       ]
   )
   ```
2. Check parameter file syntax and content:
   ```yaml
   # Correct YAML format
   node_name:
     ros__parameters:
       param1: value1
       param2: value2
   ```
3. Verify parameters after launch:
   ```bash
   ros2 param list /node_name
   ros2 param get /node_name param_name
   ```

**Verification Steps**:
- [ ] Parameter file exists at specified path
- [ ] YAML syntax is valid
- [ ] Parameter names match node expectations

#### Problem: Launch file takes too long to start or hangs
**Symptoms**:
- Launch process doesn't complete
- Some nodes start but others wait indefinitely
- High CPU or memory usage during launch

**Causes**:
- Nodes waiting for services that never become available
- Circular dependencies between nodes
- Network or resource issues

**Solutions**:
1. Add timeouts to service calls in your nodes
2. Check for proper initialization order of nodes
3. Use dry-run to see what will be launched:
   ```bash
   ros2 launch --dry-run package_name launch_file.py
   ```
4. Add logging to identify which node is causing the delay:
   ```python
   Node(
       package='package_name',
       executable='executable_name',
       output='screen'
   )
   ```

**Verification Steps**:
- [ ] All required services are available before nodes try to use them
- [ ] No circular dependencies exist between nodes
- [ ] Launch process completes within reasonable time

#### Problem: Namespace and topic conflicts
**Symptoms**:
- Nodes can't communicate properly
- Topics appear with unexpected prefixes
- Multiple robots interfere with each other

**Causes**:
- Improper namespace handling
- Topic remapping not configured correctly
- Conflicting node names

**Solutions**:
1. Use proper namespace handling in launch files:
   ```python
   from launch.actions import GroupAction
   from launch_ros.actions import PushRosNamespace

   GroupAction(
       actions=[
           PushRosNamespace(LaunchConfiguration('namespace')),
           Node(
               package='package_name',
               executable='executable_name',
               name='node_name'
           )
       ]
   )
   ```
2. Use remapping when needed:
   ```python
   Node(
       package='package_name',
       executable='executable_name',
       remappings=[
           ('original_topic', 'new_topic'),
           ('original_service', 'new_service')
       ]
   )
   ```
3. Verify topic names after launch:
   ```bash
   ros2 topic list
   ros2 topic info /expected_topic_name
   ```

**Verification Steps**:
- [ ] Topics have expected names with proper prefixes
- [ ] Nodes in different namespaces don't conflict
- [ ] Communication occurs as expected between nodes

#### Problem: Launch arguments not working as expected
**Symptoms**:
- Default values are ignored
- Arguments don't affect launch behavior
- Conditional logic doesn't work properly

**Causes**:
- Arguments not properly declared
- LaunchConfiguration not used correctly
- Conditions not set up properly

**Solutions**:
1. Ensure arguments are properly declared and used:
   ```python
   from launch.actions import DeclareLaunchArgument
   from launch.substitutions import LaunchConfiguration

   # Declare the argument
   my_arg = DeclareLaunchArgument(
       'arg_name',
       default_value='default_value',
       description='Description of argument'
   )

   # Use the argument
   arg_value = LaunchConfiguration('arg_name')

   # Include in launch description
   return LaunchDescription([
       my_arg,
       Node(
           package='package_name',
           executable='executable_name',
           parameters=[{'param': arg_value}]
       )
   ])
   ```
2. Use conditions correctly:
   ```python
   from launch.conditions import IfCondition
   from launch.actions import IncludeLaunchDescription

   IncludeLaunchDescription(
       # ... launch description source ...
       condition=IfCondition(LaunchConfiguration('should_launch'))
   )
   ```

**Verification Steps**:
- [ ] Arguments are properly declared in the launch description
- [ ] LaunchConfiguration is used to reference arguments
- [ ] Conditions work with different argument values

</details>

## Introduction to Launch Files

Launch files in ROS 2 use Python instead of XML (as in ROS 1), providing more flexibility and programmability. They allow you to:
- Start multiple nodes with a single command
- Set parameters for nodes
- Configure node behavior through arguments
- Conditionally launch nodes based on conditions

### Basic Launch File Structure

```python
# launch/basic_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='talker_node'
        ),
        Node(
            package='demo_nodes_cpp',
            executable='listener',
            name='listener_node'
        )
    ])
```

To run this launch file:
```bash
ros2 launch my_package basic_launch.py
```

## Launch File Components

### Actions

Actions are the building blocks of launch files:

#### Node Action
The most common action for starting ROS 2 nodes:

```python
from launch_ros.actions import Node

Node(
    package='my_package',
    executable='my_node',  # or 'node' for newer ROS 2 versions
    name='my_node_name',
    parameters=[
        {'param1': 'value1'},
        {'param2': 123},
        '/path/to/params.yaml'
    ],
    remappings=[
        ('original_topic', 'new_topic'),
        ('original_service', 'new_service')
    ],
    arguments=['arg1', 'arg2'],
    output='screen'  # or 'log'
)
```

#### ExecuteProcess Action
For launching non-ROS processes:

```python
from launch.actions import ExecuteProcess

ExecuteProcess(
    cmd=['echo', 'Hello World'],
    output='screen'
)
```

#### SetEnvironmentVariable Action
To set environment variables:

```python
from launch.actions import SetEnvironmentVariable

SetEnvironmentVariable(
    name='MY_VAR',
    value='my_value'
)
```

## Using Arguments in Launch Files

Arguments allow you to customize launch behavior:

```python
# launch/robot_launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    robot_name = DeclareLaunchArgument(
        'robot_name',
        default_value='my_robot',
        description='Name of the robot'
    )

    # Use launch configurations
    use_sim_time_config = LaunchConfiguration('use_sim_time')
    robot_name_config = LaunchConfiguration('robot_name')

    return LaunchDescription([
        use_sim_time,
        robot_name,

        Node(
            package='my_robot_package',
            executable='robot_controller',
            name=[robot_name_config, '_controller'],
            parameters=[
                {'use_sim_time': use_sim_time_config},
                {'robot_name': robot_name_config}
            ]
        )
    ])
```

Run with arguments:
```bash
ros2 launch my_package robot_launch.py robot_name:=turtlebot use_sim_time:=true
```

## Advanced Launch Concepts

### Conditions

Launch nodes conditionally based on arguments:

```python
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

# Declare argument
launch_rviz = DeclareLaunchArgument(
    'launch_rviz',
    default_value='true',
    description='Launch RViz?'
)

# Use condition
Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    condition=IfCondition(LaunchConfiguration('launch_rviz'))
)
```

### Groups and Namespaces

Organize nodes into groups or namespaces:

```python
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace

# Group nodes under a namespace
GroupAction(
    condition=IfCondition(LaunchConfiguration('use_namespace')),
    actions=[
        PushRosNamespace(LaunchConfiguration('namespace')),
        Node(
            package='my_package',
            executable='node1',
            name='node1'
        ),
        Node(
            package='my_package',
            executable='node2',
            name='node2'
        )
    ]
)
```

### Including Other Launch Files

Include other launch files in your launch file:

```python
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

# Include another launch file
IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        get_package_share_directory('other_package'),
        '/launch/other_launch.py'
    ])
)
```

## Practical Example: Humanoid Robot Launch System

Let's create a comprehensive launch system for a humanoid robot:

```python
# launch/humanoid_robot.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )

    robot_name = DeclareLaunchArgument(
        'robot_name',
        default_value='humanoid',
        description='Name of the robot'
    )

    launch_rviz = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Launch RViz?'
    )

    use_namespace = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Use namespace for robot nodes'
    )

    # Get configurations
    use_sim_time_config = LaunchConfiguration('use_sim_time')
    robot_name_config = LaunchConfiguration('robot_name')
    launch_rviz_config = LaunchConfiguration('launch_rviz')
    use_namespace_config = LaunchConfiguration('use_namespace')

    # Robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'use_sim_time': use_sim_time_config},
            {'robot_description':
                open(get_package_share_directory('my_robot_description') +
                     '/urdf/humanoid.urdf').read()}
        ],
        condition=IfCondition(use_namespace_config)
    )

    # Joint state publisher (GUI)
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(use_namespace_config)
    )

    # Robot controller nodes
    controller_nodes = GroupAction(
        condition=IfCondition(use_namespace_config),
        actions=[
            PushRosNamespace(robot_name_config),
            Node(
                package='my_robot_controller',
                executable='head_controller',
                name='head_controller',
                parameters=[{'use_sim_time': use_sim_time_config}]
            ),
            Node(
                package='my_robot_controller',
                executable='arm_controller',
                name='arm_controller',
                parameters=[{'use_sim_time': use_sim_time_config}]
            ),
            Node(
                package='my_robot_controller',
                executable='leg_controller',
                name='leg_controller',
                parameters=[{'use_sim_time': use_sim_time_config}]
            ),
            Node(
                package='my_robot_localization',
                executable='ekf_localization',
                name='ekf_localization',
                parameters=[get_package_share_directory('my_robot_localization') +
                           '/config/ekf.yaml']
            )
        ]
    )

    # RViz node
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', get_package_share_directory('my_robot_description') +
                  '/rviz/humanoid.rviz'],
        condition=IfCondition(launch_rviz_config)
    )

    return LaunchDescription([
        use_sim_time,
        robot_name,
        launch_rviz,
        use_namespace,
        robot_state_publisher,
        joint_state_publisher_gui,
        controller_nodes,
        rviz
    ])
```

## Launch File for Simulation

A specialized launch file for simulation environments:

```python
# launch/humanoid_gazebo.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Launch arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    robot_name = DeclareLaunchArgument(
        'robot_name',
        default_value='humanoid',
        description='Name of the robot'
    )

    world = DeclareLaunchArgument(
        'world',
        default_value='empty',
        description='Choose one of: empty, small_room, maze'
    )

    # Get configurations
    use_sim_time_config = LaunchConfiguration('use_sim_time')
    robot_name_config = LaunchConfiguration('robot_name')
    world_config = LaunchConfiguration('world')

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('gazebo_ros'),
            '/launch/gazebo.launch.py'
        ]),
        launch_arguments={
            'world': [get_package_share_directory('my_robot_gazebo'),
                     '/worlds/', world_config, '.world'],
            'verbose': 'false'
        }.items()
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', robot_name_config,
            '-x', '0', '-y', '0', '-z', '1.0'
        ],
        output='screen'
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'use_sim_time': use_sim_time_config},
            {'robot_description':
                open(get_package_share_directory('my_robot_description') +
                     '/urdf/humanoid.urdf').read()}
        ]
    )

    # Joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time_config}]
    )

    # Create launch description
    ld = LaunchDescription([
        use_sim_time,
        robot_name,
        world,
        gazebo,
        robot_state_publisher,
        joint_state_publisher,
        spawn_entity
    ])

    return ld
```

## Parameter Files with Launch Files

Launch files can load parameters from YAML files:

```yaml
# config/humanoid_params.yaml
/**:
  ros__parameters:
    use_sim_time: false
    controller_frequency: 50.0
    odom_frame_id: "odom"
    base_frame_id: "base_link"
    publish_odom_tf: true

head_controller:
  ros__parameters:
    joint_names: ["neck_joint"]
    command_interfaces: ["position"]
    state_interfaces: ["position", "velocity"]
    constraints:
      stopped_velocity_tolerance: 0.01
      goal_time: 0.5
      neck_joint:
        trajectory: 0.05
        goal: 0.01

arm_controller:
  ros__parameters:
    joint_names: ["left_shoulder_joint", "left_elbow_joint", "right_shoulder_joint", "right_elbow_joint"]
    command_interfaces: ["position"]
    state_interfaces: ["position", "velocity"]
```

Using parameters in launch:

```python
Node(
    package='controller_manager',
    executable='ros2_control_node',
    parameters=[
        {'use_sim_time': use_sim_time_config},
        os.path.join(get_package_share_directory('my_robot_control'),
                    'config', 'humanoid_params.yaml')
    ]
)
```

## Best Practices

### Organization
- Use descriptive names for launch files
- Group related functionality in separate launch files
- Use includes to compose complex systems from simpler components

### Error Handling
- Validate launch arguments
- Use appropriate conditions to avoid conflicts
- Provide clear error messages when configurations are invalid

### Performance
- Only launch nodes that are needed for the specific use case
- Use namespaces appropriately to avoid topic conflicts
- Consider using launch file caching for faster startup

## Debugging Launch Files

### Common Issues and Solutions

**Problem**: Nodes don't start or crash immediately
**Solutions**:
- Check that package names and executable names are correct
- Verify all required dependencies are installed
- Use `output='screen'` to see node output

**Problem**: Parameters not being set correctly
**Solutions**:
- Verify parameter file paths are correct
- Check that parameter names match what the node expects
- Use `ros2 param list` to verify parameters are set

**Problem**: Launch file takes too long to start
**Solutions**:
- Check for nodes that wait indefinitely for services
- Use timeouts on service calls
- Consider launching nodes in groups to identify bottlenecks

### Debugging Commands

```bash
# List all running nodes
ros2 node list

# Check parameters of a specific node
ros2 param list /node_name

# Echo topics to verify data flow
ros2 topic echo /topic_name

# Check service availability
ros2 service list

# Monitor launch file execution
ros2 launch --dry-run my_package my_launch.py  # Shows what would be launched
```

import ReadingTime from '@site/src/components/ReadingTime';
import ViewToggle from '@site/src/components/ViewToggle';

<ReadingTime minutes={18} />
<ViewToggle />