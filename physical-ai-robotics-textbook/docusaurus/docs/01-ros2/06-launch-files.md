---
sidebar_position: 7
title: 'Launch Files: Managing Complex Robot Systems'
description: 'Understanding how to create and use launch files to manage complex ROS 2 robot systems'
---

import ReadingTime from '@site/src/components/ReadingTime';

<ReadingTime minutes={5} />

<h1 className="main-heading">Launch Files: Managing Complex Robot Systems</h1>
<div className="underline-class"></div>

Launch files manage complex robotic systems by starting multiple nodes with specific configurations simultaneously.

<div className="border-line"></div>

<h2 className="second-heading">Learning Objectives</h2>
<div className="underline-class"></div>

- • Create launch files for multiple nodes
- • Use parameters, arguments, conditions
- • Launch different robot configurations
- • Integrate with robot description packages
- • Debug launch file issues

<div className="border-line"></div>

<h2 className="second-heading">Exercises</h2>
<div className="underline-class"></div>

<details>
<summary>Exercise 1.6.1: Basic Launch File (⭐, ~25 min)</summary>

<h3 className="third-heading">Exercise 1.6.1: Basic Launch File</h3>
<div className="underline-class"></div>

**Difficulty**: ⭐ | **Time**: 25 min

<h4 className="fourth-heading">Starter Code</h4>
<div className="underline-class"></div>

- • Talker node
- • Listener node
- • Screen output

<h4 className="fourth-heading">Success Criteria</h4>
<div className="underline-class"></div>

- [ ] Launch file creates LaunchDescription
- [ ] Both nodes start successfully
- [ ] Nodes communicate properly

<h4 className="fourth-heading">Test Commands</h4>
<div className="underline-class"></div>
```bash
ros2 launch my_launch_package simple_robot.launch.py
ros2 node list
```

<h4 className="fourth-heading">Hints</h4>
<div className="underline-class"></div>

- • Use launch and launch_ros modules
- • Return LaunchDescription

</details>

<details>
<summary>Exercise 1.6.2: Launch with Arguments (⭐⭐, ~40 min)</summary>

<h3 className="third-heading">Exercise 1.6.2: Launch with Arguments</h3>
<div className="underline-class"></div>

**Difficulty**: ⭐⭐ | **Time**: 40 min

<h4 className="fourth-heading">Starter Code</h4>
<div className="underline-class"></div>

- • Launch arguments (use_sim_time, robot_name, launch_rviz)
- • Conditional RViz launching
- • Robot state publisher with parameters

<h4 className="fourth-heading">Test Commands</h4>
<div className="underline-class"></div>
```bash
ros2 launch my_launch_package configurable_robot.launch.py
ros2 launch my_launch_package configurable_robot.launch.py robot_name:=my_robot
```

<h4 className="fourth-heading">Hints</h4>
<div className="underline-class"></div>

- • Use LaunchConfiguration
- • Use IfCondition/UnlessCondition

</details>

<details>
<summary>Exercise 1.6.3: Advanced Launch System (⭐⭐⭐, ~60 min)</summary>

<h3 className="third-heading">Exercise 1.6.3: Advanced Launch System</h3>
<div className="underline-class"></div>

**Difficulty**: ⭐⭐⭐ | **Time**: 60 min

<h4 className="fourth-heading">Starter Code</h4>
<div className="underline-class"></div>

- • Main launch file with includes
- • Robot controller group with namespace
- • Simulation launch with Gazebo
- • Parameter file integration

<h4 className="fourth-heading">Test Commands</h4>
<div className="underline-class"></div>
```bash
ros2 launch my_robot_bringup humanoid_robot.launch.py
ros2 node list
```

<h4 className="fourth-heading">Hints</h4>
<div className="underline-class"></div>

- • Break into smaller launch files
- • Test components separately

</details>

<div className="border-line"></div>

<h2 className="second-heading">Troubleshooting</h2>
<div className="underline-class"></div>

<details>
<summary>Common Issues</summary>

<h3 className="third-heading">Troubleshooting</h3>
<div className="underline-class"></div>

<h4 className="fourth-heading">Nodes Don't Start</h4>
<div className="underline-class"></div>

**Solutions**:
```python
Node(package='correct_name', executable='correct_exec', output='screen')
```
```bash
colcon build --packages-select package_name
source install/setup.bash
```

<h4 className="fourth-heading">Parameters Not Set</h4>
<div className="underline-class"></div>

**Solutions**:
```python
Node(parameters=['/path/to/params.yaml', {'param': 'value'}])
```
```bash
ros2 param list /node_name
```

<h4 className="fourth-heading">Launch Hangs</h4>
<div className="underline-class"></div>

**Solutions**:
```bash
ros2 launch --dry-run package_name launch_file.py
```

<h4 className="fourth-heading">Namespace Conflicts</h4>
<div className="underline-class"></div>

**Solutions**:
```python
GroupAction(actions=[
    PushRosNamespace(namespace),
    Node(...)
])
```

<h4 className="fourth-heading">Arguments Not Working</h4>
<div className="underline-class"></div>

**Solutions**:
```python
arg = DeclareLaunchArgument('arg_name', default_value='value')
arg_value = LaunchConfiguration('arg_name')
```

</details>

<div className="border-line"></div>

<h2 className="second-heading">Basic Launch File</h2>
<div className="underline-class"></div>
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='demo_nodes_cpp', executable='talker'),
        Node(package='demo_nodes_cpp', executable='listener')
    ])
```
```bash
ros2 launch my_package basic_launch.py
```

<div className="border-line"></div>

<h2 className="second-heading">Launch Components</h2>
<div className="underline-class"></div>

<h3 className="third-heading">Node Action</h3>
<div className="underline-class"></div>
```python
Node(
    package='my_package',
    executable='my_node',
    parameters=[{'param': 'value'}, '/path/params.yaml'],
    remappings=[('old_topic', 'new_topic')],
    output='screen'
)
```

<h3 className="third-heading">ExecuteProcess</h3>
<div className="underline-class"></div>
```python
ExecuteProcess(cmd=['echo', 'Hello'], output='screen')
```

<h3 className="third-heading">Environment Variables</h3>
<div className="underline-class"></div>
```python
SetEnvironmentVariable(name='VAR', value='value')
```

<div className="border-line"></div>

<h2 className="second-heading">Using Arguments</h2>
<div className="underline-class"></div>
```python
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false')
use_sim_time_config = LaunchConfiguration('use_sim_time')

Node(
    package='my_package',
    executable='node',
    parameters=[{'use_sim_time': use_sim_time_config}]
)
```
```bash
ros2 launch my_package robot.launch.py use_sim_time:=true
```

<div className="border-line"></div>

<h2 className="second-heading">Advanced Concepts</h2>
<div className="underline-class"></div>

<h3 className="third-heading">Conditions</h3>
<div className="underline-class"></div>
```python
from launch.conditions import IfCondition

Node(
    package='rviz2',
    executable='rviz2',
    condition=IfCondition(LaunchConfiguration('launch_rviz'))
)
```

<h3 className="third-heading">Groups & Namespaces</h3>
<div className="underline-class"></div>
```python
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace

GroupAction(actions=[
    PushRosNamespace(LaunchConfiguration('namespace')),
    Node(...),
    Node(...)
])
```

<h3 className="third-heading">Include Launch Files</h3>
<div className="underline-class"></div>
```python
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        get_package_share_directory('package'),
        '/launch/other.launch.py'
    ])
)
```

<div className="border-line"></div>

<h2 className="second-heading">Humanoid Robot Example</h2>
<div className="underline-class"></div>
```python
def generate_launch_description():
    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false')
    launch_rviz = DeclareLaunchArgument('launch_rviz', default_value='true')
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        condition=IfCondition(LaunchConfiguration('launch_rviz'))
    )
    
    return LaunchDescription([use_sim_time, launch_rviz, robot_state_publisher, rviz])
```

<div className="border-line"></div>

<h2 className="second-heading">Simulation Launch</h2>
<div className="underline-class"></div>
```python
gazebo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        get_package_share_directory('gazebo_ros'),
        '/launch/gazebo.launch.py'
    ]),
    launch_arguments={'world': world_config}.items()
)

spawn_entity = Node(
    package='gazebo_ros',
    executable='spawn_entity.py',
    arguments=['-topic', 'robot_description', '-entity', 'robot']
)
```

<div className="border-line"></div>

<h2 className="second-heading">Parameter Files</h2>
<div className="underline-class"></div>
```yaml
/**:
  ros__parameters:
    use_sim_time: false
    controller_frequency: 50.0

controller:
  ros__parameters:
    joint_names: ["joint1", "joint2"]
```
```python
Node(
    package='controller',
    executable='node',
    parameters=[os.path.join(pkg_dir, 'config', 'params.yaml')]
)
```

<div className="border-line"></div>

<h2 className="second-heading">Best Practices</h2>
<div className="underline-class"></div>

- • Use descriptive names
- • Group related functionality
- • Use includes for complex systems
- • Validate arguments
- • Use appropriate conditions
- • Launch only needed nodes
- • Use namespaces properly

<h2 className="second-heading">Debugging</h2>
<div className="underline-class"></div>
```bash
ros2 node list
ros2 param list /node_name
ros2 topic echo /topic_name
ros2 launch --dry-run package launch_file.py
```

<h2 className="second-heading">Summary</h2>
<div className="underline-class"></div>

Launch files use Python for flexibility in managing complex robot systems with multiple nodes, parameters, arguments, and conditions.