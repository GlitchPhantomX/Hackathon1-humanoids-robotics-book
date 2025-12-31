---
sidebar_position: 7
title: 'لاؤنچ فائلز: پیچیدہ روبوٹ سسٹم کا نظم کرنا'
description: 'ROS 2 روبوٹ سسٹم کا نظم کرنے کے لیے لاؤنچ فائلز بنانے اور استعمال کرنے کی سمجھ'
---

import ReadingTime from '@site/src/components/ReadingTime';


<ReadingTime minutes={5} />

<h1 className="main-heading">لاؤنچ فائلز: پیچیدہ روبوٹ سسٹم کا نظم کرنا</h1>
<div className="underline-class"></div>

لاؤنچ فائلز پیچیدہ روبوٹک سسٹم کا نظم کرتی ہیں جو متعدد نوڈس کو مخصوص کنفیگریشن کے ساتھ ایک وقت میں شروع کرتی ہیں۔

<div className="border-line"></div>

<h2 className="second-heading">سیکھنے کے اہداف</h2>
<div className="underline-class"></div>

- • متعدد نوڈس کے لیے لاؤنچ فائلز بنانا
- • پیرامیٹرز، آرگومنٹس، شرائط استعمال کرنا
- • مختلف روبوٹ کنفیگریشنز لاؤنچ کرنا
- • روبوٹ کی تفصیل کے پیکیجز کے ساتھ انضمام
- • لاؤنچ فائل کے مسائل کی ڈیبگنگ کرنا

<div className="border-line"></div>

<h2 className="second-heading">مشقیں</h2>
<div className="underline-class"></div>

<details>
<summary>مشق 1.6.1: بنیادی لاؤنچ فائل (⭐, ~25 منٹ)</summary>

<h3 className="third-heading">مشق 1.6.1: بنیادی لاؤنچ فائل</h3>
<div className="underline-class"></div>

**پیچیدگی**: ⭐ | **وقت**: 25 منٹ

<h4 className="fourth-heading">شروع کرنے والا کوڈ</h4>
<div className="underline-class"></div>

- • ٹالکر نوڈ
- • لسٹنر نوڈ
- • اسکرین آؤٹ پٹ

<h4 className="fourth-heading">کامیابی کے معیار</h4>
<div className="underline-class"></div>

- [ ] لاؤنچ فائل LaunchDescription بنتی ہے
- [ ] دونوں نوڈس کامیابی سے شروع ہوتے ہیں
- [ ] نوڈس مناسب طریقے سے بات چیت کرتے ہیں

<h4 className="fourth-heading">ٹیسٹ کمانڈز</h4>
<div className="underline-class"></div>
```bash
ros2 launch my_launch_package simple_robot.launch.py
ros2 node list
```

<h4 className="fourth-heading">اشارے</h4>
<div className="underline-class"></div>

- • launch اور launch_ros ماڈیولز استعمال کریں
- • LaunchDescription واپس کریں

</details>

<details>
<summary>مشق 1.6.2: آرگومنٹس کے ساتھ لاؤنچ (⭐⭐, ~40 منٹ)</summary>

<h3 className="third-heading">مشق 1.6.2: آرگومنٹس کے ساتھ لاؤنچ</h3>
<div className="underline-class"></div>

**پیچیدگی**: ⭐⭐ | **وقت**: 40 منٹ

<h4 className="fourth-heading">شروع کرنے والا کوڈ</h4>
<div className="underline-class"></div>

- • لاؤنچ آرگومنٹس (use_sim_time, robot_name, launch_rviz)
- • شرائطی RViz لاؤنچنگ
- • پیرامیٹرز کے ساتھ روبوٹ اسٹیٹ پبلشر

<h4 className="fourth-heading">ٹیسٹ کمانڈز</h4>
<div className="underline-class"></div>
```bash
ros2 launch my_launch_package configurable_robot.launch.py
ros2 launch my_launch_package configurable_robot.launch.py robot_name:=my_robot
```

<h4 className="fourth-heading">اشارے</h4>
<div className="underline-class"></div>

- • LaunchConfiguration استعمال کریں
- • IfCondition/UnlessCondition استعمال کریں

</details>

<details>
<summary>مشق 1.6.3: اعلیٰ لاؤنچ سسٹم (⭐⭐⭐, ~60 منٹ)</summary>

<h3 className="third-heading">مشق 1.6.3: اعلیٰ لاؤنچ سسٹم</h3>
<div className="underline-class"></div>

**پیچیدگی**: ⭐⭐⭐ | **وقت**: 60 منٹ

<h4 className="fourth-heading">شروع کرنے والا کوڈ</h4>
<div className="underline-class"></div>

- • انکلودز کے ساتھ میں لاؤنچ فائل
- • نیم سپیس کے ساتھ روبوٹ کنٹرولر گروپ
- • Gazebo کے ساتھ سیمولیشن لاؤنچ
- • پیرامیٹر فائل کا انضمام

<h4 className="fourth-heading">ٹیسٹ کمانڈز</h4>
<div className="underline-class"></div>
```bash
ros2 launch my_robot_bringup humanoid_robot.launch.py
ros2 node list
```

<h4 className="fourth-heading">اشارے</h4>
<div className="underline-class"></div>

- • چھوٹی لاؤنچ فائلز میں توڑیں
- • اجزاء کو الگ الگ ٹیسٹ کریں

</details>

<div className="border-line"></div>

<h2 className="second-heading">ٹربل شوٹنگ</h2>
<div className="underline-class"></div>

<details>
<summary>عام مسائل</summary>

<h3 className="third-heading">ٹربل شوٹنگ</h3>
<div className="underline-class"></div>

<h4 className="fourth-heading">نوڈس شروع نہیں ہوتے</h4>
<div className="underline-class"></div>

**حل**:
```python
Node(package='correct_name', executable='correct_exec', output='screen')
```
```bash
colcon build --packages-select package_name
source install/setup.bash
```

<h4 className="fourth-heading">پیرامیٹرز سیٹ نہیں ہیں</h4>
<div className="underline-class"></div>

**حل**:
```python
Node(parameters=['/path/to/params.yaml', {'param': 'value'}])
```
```bash
ros2 param list /node_name
```

<h4 className="fourth-heading">لاؤنچ ہینگ ہو جاتا ہے</h4>
<div className="underline-class"></div>

**حل**:
```bash
ros2 launch --dry-run package_name launch_file.py
```

<h4 className="fourth-heading">نیم سپیس کانFLICTS</h4>
<div className="underline-class"></div>

**حل**:
```python
GroupAction(actions=[
    PushRosNamespace(namespace),
    Node(...)
])
```

<h4 className="fourth-heading">آرگومنٹس کام نہیں کرتے</h4>
<div className="underline-class"></div>

**حل**:
```python
arg = DeclareLaunchArgument('arg_name', default_value='value')
arg_value = LaunchConfiguration('arg_name')
```

</details>

<div className="border-line"></div>

<h2 className="second-heading">بنیادی لاؤنچ فائل</h2>
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

<h2 className="second-heading">لاؤنچ اجزاء</h2>
<div className="underline-class"></div>

<h3 className="third-heading">نوڈ ایکشن</h3>
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

<h3 className="third-heading">ایکسیکیوٹ پروسیس</h3>
<div className="underline-class"></div>
```python
ExecuteProcess(cmd=['echo', 'Hello'], output='screen')
```

<h3 className="third-heading">ماحولیاتی متغیرات</h3>
<div className="underline-class"></div>
```python
SetEnvironmentVariable(name='VAR', value='value')
```

<div className="border-line"></div>

<h2 className="second-heading">آرگومنٹس استعمال کرنا</h2>
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

<h2 className="second-heading">اعلیٰ تصورات</h2>
<div className="underline-class"></div>

<h3 className="third-heading">شرائط</h3>
<div className="underline-class"></div>
```python
from launch.conditions import IfCondition

Node(
    package='rviz2',
    executable='rviz2',
    condition=IfCondition(LaunchConfiguration('launch_rviz'))
)
```

<h3 className="third-heading">گروپس اور نیم سپیسز</h3>
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

<h3 className="third-heading">لاؤنچ فائلز شامل کرنا</h3>
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

<h2 className="second-heading">ہیومنوائڈ روبوٹ مثال</h2>
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

<h2 className="second-heading">سیمولیشن لاؤنچ</h2>
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

<h2 className="second-heading">پیرامیٹر فائلز</h2>
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

<h2 className="second-heading">بہترین مشقیں</h2>
<div className="underline-class"></div>

- • تفصیلی نام استعمال کریں
- • متعلقہ فعل کو گروپ کریں
- • پیچیدہ سسٹم کے لیے انکلودز استعمال کریں
- • آرگومنٹس کی تصدیق کریں
- • مناسب شرائط استعمال کریں
- • صرف ضروری نوڈس لاؤنچ کریں
- • نیم سپیسز کو مناسب طریقے سے استعمال کریں

<h2 className="second-heading">ڈیبگنگ</h2>
<div className="underline-class"></div>
```bash
ros2 node list
ros2 param list /node_name
ros2 topic echo /topic_name
ros2 launch --dry-run package launch_file.py
```

<h2 className="second-heading">خلاصہ</h2>
<div className="underline-class"></div>

لاؤنچ فائلز متعدد نوڈس، پیرامیٹرز، آرگومنٹس، اور شرائط کے ساتھ پیچیدہ روبوٹ سسٹم کا نظم کرنے کے لیے لچک کے لیے Python استعمال کرتی ہیں۔