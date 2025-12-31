---
sidebar_position: 7
title: 'लॉन्च फाइलें: जटिल रोबोट प्रणालियों का प्रबंधन'
description: 'जटिल आरओएस 2 रोबोट प्रणालियों का प्रबंधन करने के लिए लॉन्च फाइलें बनाने और उपयोग करने की समझ'
---

import ReadingTime from '@site/src/components/ReadingTime';

<ReadingTime minutes={5} />

<h1 className="main-heading">लॉन्च फाइलें: जटिल रोबोट प्रणालियों का प्रबंधन</h1>
<div className="underline-class"></div>

लॉन्च फाइलें जटिल रोबोटिक्स प्रणालियों का प्रबंधन करती हैं जो एक साथ विशिष्ट विन्यासों के साथ कई नोड्स शुरू करती हैं।

<div className="border-line"></div>

<h2 className="second-heading">सीखने के उद्देश्य</h2>
<div className="underline-class"></div>

- • कई नोड्स के लिए लॉन्च फाइलें बनाएं
- • पैरामीटर, तर्क, शर्तों का उपयोग करें
- • अलग-अलग रोबोट विन्यास लॉन्च करें
- • रोबोट विवरण पैकेजों के साथ एकीकरण
- • लॉन्च फाइल समस्याओं का डिबग करें

<div className="border-line"></div>

<h2 className="second-heading">अभ्यास</h2>
<div className="underline-class"></div>

<details>
<summary>अभ्यास 1.6.1: मूल लॉन्च फाइल (⭐, ~25 मिनट)</summary>

<h3 className="third-heading">अभ्यास 1.6.1: मूल लॉन्च फाइल</h3>
<div className="underline-class"></div>

**कठिनाई**: ⭐ | **समय**: 25 मिनट

<h4 className="fourth-heading">स्टार्टर कोड</h4>
<div className="underline-class"></div>

- • टॉकर नोड
- • लिसनर नोड
- • स्क्रीन आउटपुट

<h4 className="fourth-heading">सफलता मानदंड</h4>
<div className="underline-class"></div>

- [ ] लॉन्च फाइल लॉन्चडिस्क्रिप्शन बनाती है
- [ ] दोनों नोड्स सफलतापूर्वक शुरू होते हैं
- [ ] नोड्स ठीक से संचार करते हैं

<h4 className="fourth-heading">परीक्षण कमांड</h4>
<div className="underline-class"></div>
```bash
ros2 launch my_launch_package simple_robot.launch.py
ros2 node list
```

<h4 className="fourth-heading">संकेत</h4>
<div className="underline-class"></div>

- • लॉन्च और लॉन्च_रोस मॉडल का उपयोग करें
- • लॉन्चडिस्क्रिप्शन लौटाएं

</details>

<details>
<summary>अभ्यास 1.6.2: तर्कों के साथ लॉन्च (⭐⭐, ~40 मिनट)</summary>

<h3 className="third-heading">अभ्यास 1.6.2: तर्कों के साथ लॉन्च</h3>
<div className="underline-class"></div>

**कठिनाई**: ⭐⭐ | **समय**: 40 मिनट

<h4 className="fourth-heading">स्टार्टर कोड</h4>
<div className="underline-class"></div>

- • लॉन्च तर्क (use_sim_time, robot_name, launch_rviz)
- • सशर्त आरवीआईज़ लॉन्चिंग
- • पैरामीटर के साथ रोबोट स्टेट पब्लिशर

<h4 className="fourth-heading">परीक्षण कमांड</h4>
<div className="underline-class"></div>
```bash
ros2 launch my_launch_package configurable_robot.launch.py
ros2 launch my_launch_package configurable_robot.launch.py robot_name:=my_robot
```

<h4 className="fourth-heading">संकेत</h4>
<div className="underline-class"></div>

- • लॉन्चकॉन्फ़िगरेशन का उपयोग करें
- • अगर_कॉन्डिशन/अनलेस_कॉन्डिशन का उपयोग करें

</details>

<details>
<summary>अभ्यास 1.6.3: उन्नत लॉन्च सिस्टम (⭐⭐⭐, ~60 मिनट)</summary>

<h3 className="third-heading">अभ्यास 1.6.3: उन्नत लॉन्च सिस्टम</h3>
<div className="underline-class"></div>

**कठिनाई**: ⭐⭐⭐ | **समय**: 60 मिनट

<h4 className="fourth-heading">स्टार्टर कोड</h4>
<div className="underline-class"></div>

- • इनक्लूड्स के साथ मुख्य लॉन्च फाइल
- • नामस्थान के साथ रोबोट कंट्रोलर समूह
- • गेज़बो के साथ सिमुलेशन लॉन्च
- • पैरामीटर फाइल एकीकरण

<h4 className="fourth-heading">परीक्षण कमांड</h4>
<div className="underline-class"></div>
```bash
ros2 launch my_robot_bringup humanoid_robot.launch.py
ros2 node list
```

<h4 className="fourth-heading">संकेत</h4>
<div className="underline-class"></div>

- • छोटी लॉन्च फाइलों में तोड़ें
- • घटकों को अलग से परीक्षण करें

</details>

<div className="border-line"></div>

<h2 className="second-heading">समस्या निवारण</h2>
<div className="underline-class"></div>

<details>
<summary>सामान्य समस्याएं</summary>

<h3 className="third-heading">समस्या निवारण</h3>
<div className="underline-class"></div>

<h4 className="fourth-heading">नोड्स शुरू नहीं होते</h4>
<div className="underline-class"></div>

**समाधान**:
```python
Node(package='correct_name', executable='correct_exec', output='screen')
```
```bash
colcon build --packages-select package_name
source install/setup.bash
```

<h4 className="fourth-heading">पैरामीटर सेट नहीं होते</h4>
<div className="underline-class"></div>

**समाधान**:
```python
Node(parameters=['/path/to/params.yaml', {'param': 'value'}])
```
```bash
ros2 param list /node_name
```

<h4 className="fourth-heading">लॉन्च फंस जाता है</h4>
<div className="underline-class"></div>

**समाधान**:
```bash
ros2 launch --dry-run package_name launch_file.py
```

<h4 className="fourth-heading">नामस्थान संघर्ष</h4>
<div className="underline-class"></div>

**समाधान**:
```python
GroupAction(actions=[
    PushRosNamespace(namespace),
    Node(...)
])
```

<h4 className="fourth-heading">तर्क काम नहीं करते</h4>
<div className="underline-class"></div>

**समाधान**:
```python
arg = DeclareLaunchArgument('arg_name', default_value='value')
arg_value = LaunchConfiguration('arg_name')
```

</details>

<div className="border-line"></div>

<h2 className="second-heading">मूल लॉन्च फाइल</h2>
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

<h2 className="second-heading">लॉन्च घटक</h2>
<div className="underline-class"></div>

<h3 className="third-heading">नोड एक्शन</h3>
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

<h3 className="third-heading">एक्सीक्यूटप्रोसेस</h3>
<div className="underline-class"></div>
```python
ExecuteProcess(cmd=['echo', 'Hello'], output='screen')
```

<h3 className="third-heading">वातावरण चर</h3>
<div className="underline-class"></div>
```python
SetEnvironmentVariable(name='VAR', value='value')
```

<div className="border-line"></div>

<h2 className="second-heading">तर्कों का उपयोग</h2>
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

<h2 className="second-heading">उन्नत अवधारणाएं</h2>
<div className="underline-class"></div>

<h3 className="third-heading">शर्तें</h3>
<div className="underline-class"></div>
```python
from launch.conditions import IfCondition

Node(
    package='rviz2',
    executable='rviz2',
    condition=IfCondition(LaunchConfiguration('launch_rviz'))
)
```

<h3 className="third-heading">समूह और नामस्थान</h3>
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

<h3 className="third-heading">लॉन्च फाइलें शामिल करें</h3>
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

<h2 className="second-heading">मानवरूपी रोबोट उदाहरण</h2>
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

<h2 className="second-heading">सिमुलेशन लॉन्च</h2>
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

<h2 className="second-heading">पैरामीटर फाइलें</h2>
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

<h2 className="second-heading">सर्वोत्तम प्रथाएं</h2>
<div className="underline-class"></div>

- • वर्णनात्मक नामों का उपयोग करें
- • संबंधित कार्यक्षमता को समूहित करें
- • जटिल प्रणालियों के लिए इनक्लूड्स का उपयोग करें
- • तर्कों की पुष्टि करें
- • उपयुक्त शर्तों का उपयोग करें
- • केवल आवश्यक नोड्स लॉन्च करें
- • नामस्थानों का उचित उपयोग करें

<h2 className="second-heading">डिबगिंग</h2>
<div className="underline-class"></div>
```bash
ros2 node list
ros2 param list /node_name
ros2 topic echo /topic_name
ros2 launch --dry-run package launch_file.py
```

<h2 className="second-heading">सारांश</h2>
<div className="underline-class"></div>

लॉन्च फाइलें मूलभूत रूप से जटिल रोबोट प्रणालियों के प्रबंधन में लचीलापन के लिए पायथन का उपयोग करती हैं जिनमें कई नोड्स, पैरामीटर, तर्क और शर्तें होती हैं।