---
sidebar_position: 7
title: 'اعلیٰ سیمولیشن: پیچیدہ منظرنامے'
description: 'پیچیدہ روبوٹکس منظرناموں اور کارکردگی کی بہتری کے لیے اعلیٰ سیمولیشن کی تکنیکیں'
---

import ReadingTime from '@site/src/components/ReadingTime';
import ViewToggle from '@site/src/components/ViewToggle';


<ReadingTime minutes={20} />
<!-- <ViewToggle /> -->

<h1 className="main-heading">اعلیٰ سیمولیشن: پیچیدہ منظرنامے اور بہتری</h1>

<div className="underline-class"></div>

<h2 className="second-heading">سیکھنے کے اہداف</h2>

<div className="border-line"></div>

اس باب کے اختتام تک، آپ کے اہل ہوں گے:
- • متعدد روبوٹ سیمولیشن منظرنامے ڈیزائن کرنا
- • بڑے پیمانے کے ماحول کے لیے سیمولیشن کارکردگی کو بہتر بنانا
- • حقیقی تعاملات کے لیے اعلیٰ فزکس ماڈلز نافذ کرنا
- • روبوٹ ایکشنز کا جواب دینے والے متحرک ماحول بنانا
- • اعلیٰ سیمولیشن-سے-حقیقت منتقلی کی تکنیکیں لاگو کرنا

<div className="border-line"></div>

<h2 className="second-heading">مشقیں</h2>

<div className="border-line"></div>

<details>
<summary>مشق 2.6.1: متعدد روبوٹ کوآرڈینیشن (⭐⭐, ~40 منٹ)</summary>

<h3 className="third-heading">مشق 2.6.1: متعدد روبوٹ کوآرڈینیشن</h3>

<div className="border-line"></div>

**پیچیدگی**: ⭐⭐ | **وقت**: 40 منٹ

<h4 className="fourth-heading">کام</h4>

<div className="border-line"></div>

کوآرڈینیشن کے ساتھ متعدد روبوٹ سیمولیشن بنائیں

<h4 className="fourth-heading">ٹیسٹ کمانڈز</h4>

<div className="border-line"></div>

```bash
# متعدد روبوٹ سیمولیشن لانچ کریں
gz sim -r multi_robot_lab.sdf

# روبوٹس چیک کریں
ros2 topic list | grep humanoid
ros2 topic echo /humanoid_1/pose

# کوآرڈینیشن ٹیسٹ کریں
ros2 topic echo /coordination/status
```

<h4 className="fourth-heading">کامیابی کے معیار</h4>

<div className="border-line"></div>

- [ ] دونوں روبوٹس ایک ساتھ کام کرتے ہیں
- [ ] کوآرڈینیشن تصادم سے بچاتی ہے
- [ ] کام کامیابی سے مکمل ہوتے ہیں
- [ ] مواصلات کار کام کرتی ہے
- [ ] کارکردگی مستحکم ہے

</details>

<details>
<summary>مشق 2.6.2: اعلیٰ فزکس (⭐⭐, ~50 منٹ)</summary>

<h3 className="third-heading">مشق 2.6.2: اعلیٰ فزکس سیمولیشن</h3>

<div className="border-line"></div>

**پیچیدگی**: ⭐⭐ | **وقت**: 50 منٹ

<h4 className="fourth-heading">کام</h4>

<div className="border-line"></div>

حقیقی کنٹیکٹ ڈائنامکس کے ساتھ روبوٹ بنائیں

<h4 className="fourth-heading">ٹیسٹ کمانڈز</h4>

<div className="border-line"></div>

```bash
# فزکس کی توثیق کریں
gz sdf -k advanced_physics_robot.sdf

# سیمولیشن لانچ کریں
gz sim -r advanced_physics_world.sdf

# کنٹیکٹس مانیٹر کریں
gz topic -e /world/contact
ros2 topic echo /robot/joint_states
```

<h4 className="fourth-heading">کامیابی کے معیار</h4>

<div className="border-line"></div>

- [ ] روبوٹ مستحکم توازن برقرار رکھتا ہے
- [ ] کنٹیکٹ فورسز حقیقی ہیں
- [ ] جوائنٹ حرکات مناسب ہیں
- [ ] فزکس پیرامیٹرز حقیقی ہیں
- [ ] بنیادی حرکات مستحکم ہیں

</details>

<details>
<summary>مشق 2.6.3: متحرک ماحول (⭐⭐⭐, ~65 منٹ)</summary>

<h3 className="third-heading">مشق 2.6.3: متحرک انٹرایکٹو ماحول</h3>

<div className="border-line"></div>

**پیچیدگی**: ⭐⭐⭐ | **وقت**: 65 منٹ

<h4 className="fourth-heading">کام</h4>

<div className="border-line"></div>

روبوٹ ایکشنز کا جواب دینے والا ماحول بنائیں

<h4 className="fourth-heading">ٹیسٹ کمانڈز</h4>

<div className="border-line"></div>

```bash
# ماحول لانچ کریں
gz sim -r interactive_environment.sdf

# انٹرایکشنز ٹیسٹ کریں
ros2 topic pub /robot/gripper/command std_msgs/msg/Float64 "data: 0.5"

# کارکردگی مانیٹر کریں
gz stats
ros2 topic hz /camera/image_raw
```

<h4 className="fourth-heading">کامیابی کے معیار</h4>

<div className="border-line"></div>

- [ ] اشیاء انٹرایکشنز کا جواب دیتی ہیں
- [ ] ماحولیاتی اثرات فعال ہیں
- [ ] سینسر نوائز حقیقی ہے
- [ ] کارکردگی کو بہتر بنایا گیا ہے
- [ ] روبوٹ کامیابی سے انٹرایکٹ کرتا ہے

</details>

<div className="border-line"></div>

<h2 className="second-heading">ٹربل شوٹنگ</h2>

<div className="border-line"></div>

<details>
<summary>ٹربل شوٹنگ: اعلیٰ سیمولیشن کے مسائل</summary>

<h3 className="third-heading">ٹربل شوٹنگ: اعلیٰ سیمولیشن کے مسائل</h3>

<div className="border-line"></div>

<h4 className="fourth-heading">مسئلہ: متعدد روبوٹ کارکردگی کم ہو جاتی ہے</h4>

<div className="border-line"></div>

**علامات**:
- • کم ریل ٹائم فیکٹر (< 0.5)
- • زیادہ CPU/GPU استعمال
- • جرکی حرکات
- • میسج قطار میں اوور فلو

<div className="border-line"></div>

**حل**:
```xml
<!-- اپ ڈیٹ شرح کم کریں -->
<sensor name="camera" type="camera">
  <update_rate>15</update_rate>
</sensor>
```

```bash
# الگ ڈومینز استعمال کریں
export ROS_DOMAIN_ID=1
ros2 launch robot_group_1 bringup.launch.py
```

<div className="border-line"></div>

<h4 className="fourth-heading">مسئلہ: فزکس سیمولیشن غیر مستحکم ہے</h4>

<div className="border-line"></div>

**علامات**:
- • اشیاء سطحوں کے ذریعے گرتی ہیں
- • جوائنٹس بے ترتیب طور پر برتاؤ کرتے ہیں
- • سیمولیشن پھٹ جاتی ہے

<div className="border-line"></div>

**حل**:
```xml
<!-- مستحکم فزکس کنفیگ -->
<physics name="ode_stable" type="ode">
  <max_step_size>0.001</max_step_size>
  <ode>
    <solver>
      <iters>100</iters>
    </solver>
    <constraints>
      <cfm>1e-6</cfm>
      <erp>0.2</erp>
    </constraints>
  </ode>
</physics>
```

<div className="border-line"></div>

<h4 className="fourth-heading">مسئلہ: متحرک ماحول کی غیر مستحکمی</h4>

<div className="border-line"></div>

**علامات**:
- • متحرک اشیاء کے ساتھ غیر مستحکم
- • فزکس کی ناکامیاں
- • کارکردگی میں کمی

<div className="border-line"></div>

**حل**:
```xml
<!-- سادہ کولیژن -->
<collision name="collision">
  <geometry>
    <box><size>0.2 0.2 0.2</size></box>
  </geometry>
</collision>
```

<div className="border-line"></div>

<h4 className="fourth-heading">مسئلہ: سیمولیشن-سے-حقیقت منتقلی ناکام ہو جاتی ہے</h4>

<div className="border-line"></div>

**علامات**:
- • حقیقی روبوٹس پر برتاؤ ناکام ہوتے ہیں
- • وسیع پیرامیٹر ری ٹیوننگ کی ضرورت ہوتی ہے
- • سینسر ڈیٹا تقسیم میں فرق ہوتا ہے

<div className="border-line"></div>

**حل**:
```python
# حقیقی نوائز ماڈل
def add_lidar_noise(ranges):
    noise_std = 0.01 + 0.005 * ranges
    noise = np.random.normal(0, noise_std)
    return max(0.05, ranges + noise)

# ڈومین رینڈمائزیشن
class DomainRandomizer:
    def __init__(self):
        self.param_ranges = {
            'gravity': (-10.1, -9.5),
            'friction': (0.4, 1.2),
            'mass_multiplier': (0.9, 1.1)
        }
```

</details>

<div className="border-line"></div>

<h2 className="second-heading">متعدد روبوٹ سیمولیشن</h2>

<div className="border-line"></div>

<h3 className="third-heading">ہم آہنگ ماحول</h3>

<div className="border-line"></div>

متعدد روبوٹ ماحول بنانے کے لیے فزکس، مواصلات، اور کنٹرول کا ہم آہنگ ہونا ضروری ہے:

```xml
<!-- متعدد روبوٹ ورلڈ -->
<world name="multi_robot_lab">
  <model name="humanoid_1">
    <pose>-2 0 0.8 0 0 0</pose>
    <sensor name="camera_1" type="camera">
      <plugin filename="libgazebo_ros_camera.so">
        <namespace>/humanoid_1</namespace>
      </plugin>
    </sensor>
  </model>

  <model name="humanoid_2">
    <pose>2 0 0.8 0 0 0</pose>
    <sensor name="camera_2" type="camera">
      <plugin filename="libgazebo_ros_camera.so">
        <namespace>/humanoid_2</namespace>
      </plugin>
    </sensor>
  </model>
</world>
```

<div className="border-line"></div>

<h3 className="third-heading">ہم آہنگ الگورتھم</h3>

<div className="border-line"></div>

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np

class MultiRobotCoordinator(Node):
    def __init__(self):
        super().__init__('coordinator')
        self.robot_states = {
            'humanoid_1': {'x': 0, 'y': 0},
            'humanoid_2': {'x': 0, 'y': 0}
        }
        self.cmd_pubs = {}
        for name in self.robot_states:
            self.cmd_pubs[name] = self.create_publisher(
                Twist, f'/{name}/cmd_vel', 10)

    def coordination_logic(self):
        pos1 = np.array([self.robot_states['humanoid_1']['x'],
                        self.robot_states['humanoid_1']['y']])
        pos2 = np.array([self.robot_states['humanoid_2']['x'],
                        self.robot_states['humanoid_2']['y']])

        distance = np.linalg.norm(pos1 - pos2)
        if distance < 1.0:  # بہت قریب
            self.send_separation_commands()
```

<div className="border-line"></div>

<h2 className="second-heading">اعلیٰ فزکس ماڈلز</h2>

<div className="border-line"></div>

<h3 className="third-heading">حقیقی کنٹیکٹ ڈائنامکس</h3>

<div className="border-line"></div>

```xml
<!-- اعلیٰ فزکس کنفیگ -->
<physics name="ode_advanced" type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_update_rate>1000</real_time_update_rate>
  <ode>
    <solver>
      <type>quick</type>
      <iters>100</iters>
    </solver>
    <constraints>
      <cfm>1e-5</cfm>
      <erp>0.2</erp>
    </constraints>
  </ode>
</physics>

<!-- اعلیٰ سطحی خصوصیات -->
<surface>
  <friction>
    <ode>
      <mu>0.9</mu>
      <mu2>0.9</mu2>
    </ode>
    <torsional>
      <coefficient>0.8</coefficient>
    </torsional>
  </friction>
  <contact>
    <ode>
      <soft_cfm>0.001</soft_cfm>
      <kp>1e+6</kp>
    </ode>
  </contact>
</surface>
```

<div className="border-line"></div>

<h2 className="second-heading">متحرک ماحول</h2>

<div className="border-line"></div>

<h3 className="third-heading">انٹرایکٹو عناصر</h3>

<div className="border-line"></div>

```xml
<!-- متحرک ماحول -->
<model name="movable_table">
  <link name="table_base">
    <inertial>
      <mass>20</mass>
    </inertial>
  </link>
</model>

<model name="door">
  <joint name="door_hinge" type="revolute">
    <parent>door_frame</parent>
    <child>door_panel</child>
    <plugin filename="libgazebo_ros_joint_trajectory.so">
      <command_topic>door_control</command_topic>
    </plugin>
  </joint>
</model>
```

<div className="border-line"></div>

<h2 className="second-heading">کارکردگی کی بہتری</h2>

<div className="border-line"></div>

<h3 className="third-heading">تفصیل کی سطح (LOD)</h3>

<div className="border-line"></div>

```python
class LODController(Node):
    def __init__(self):
        super().__init__('lod_controller')
        self.objects = [
            {'name': 'building', 'thresholds': [5, 10, 20]}
        ]

    def optimize_lod(self):
        for obj in self.objects:
            distance = self.calculate_distance(obj)
            if distance < obj['thresholds'][0]:
                lod_level = 0  # زیادہ تفصیل
            elif distance < obj['thresholds'][1]:
                lod_level = 1  # درمیانہ
            else:
                lod_level = 2  # کم
```

<div className="border-line"></div>

<h2 className="second-heading">سیمولیشن-سے-حقیقت منتقلی</h2>

<div className="border-line"></div>

<h3 className="third-heading">ڈومین رینڈمائزیشن</h3>

<div className="border-line"></div>

```python
class DomainRandomizer(Node):
    def __init__(self):
        super().__init__('randomizer')
        self.param_ranges = {
            'gravity': (-10.1, -9.5),
            'friction': (0.4, 1.2),
            'mass_mult': (0.9, 1.1)
        }

    def randomize_environment(self):
        for param, (min_val, max_val) in self.param_ranges.items():
            value = random.uniform(min_val, max_val)
            self.apply_parameter(param, value)
```

<div className="border-line"></div>

<h3 className="third-heading">سینسر نوائز ماڈلنگ</h3>

<div className="border-line"></div>

```python
class SensorNoiseModel(Node):
    def lidar_callback(self, msg):
        ranges = np.array(msg.ranges)
        noise_std = 0.01 + 0.005 * ranges
        noise = np.random.normal(0, noise_std, ranges.shape)
        noisy_ranges = np.clip(ranges + noise,
                               msg.range_min, msg.range_max)
        msg.ranges = noisy_ranges.tolist()
        self.pub.publish(msg)

    def imu_callback(self, msg):
        # زاویہ ویلوسٹی نوائز شامل کریں
        noise_std = 0.001
        msg.angular_velocity.x += np.random.normal(0, noise_std)
        msg.angular_velocity.y += np.random.normal(0, noise_std)
        self.pub.publish(msg)
```

<div className="border-line"></div>

<h2 className="second-heading">بہترین مشقیں</h2>

<div className="border-line"></div>

<h3 className="third-heading">آرکیٹیکچر ڈیزائن</h3>

<div className="border-line"></div>

- • **ماڈیولر اجزاء**: دوبارہ استعمال کے قابل عناصر
- • **ریسورس مینجمنٹ**: کارآمد استعمال
- • **ڈیٹا فلو کی بہتری**: منتقلی کو کم کریں
- • **متوازی پروسیسنگ**: ملٹی-کور استعمال

<div className="border-line"></div>

<h3 className="third-heading">توثیق کی حکمت عملیں</h3>

<div className="border-line"></div>

- • **کراس توثیق**: حقیقی ڈیٹا کے ساتھ موازنہ کریں
- • **حساسیت کا تجزیہ**: پیرامیٹر اثرات ٹیسٹ کریں
- • **احصائی توثیق**: تقسیم کی توثیق کریں
- • **بینچ مارکنگ**: معیارات کے خلاف موازنہ کریں

<div className="border-line"></div>

<h2 className="second-heading">عام مسائل</h2>

<div className="border-line"></div>

**کارکردگی میں کمی**
- • LOD سسٹم نافذ کریں
- • سپیشل پارٹیشننگ استعمال کریں
- • فزکس پیرامیٹرز کو بہتر بنائیں
- • متوازی انسٹینس پر غور کریں

**متعدد روبوٹ کوآرڈینیشن ناکام ہو جاتی ہے**
- • سلسلہ وار کوآرڈینیشن
- • تقسیم شدہ کمپیوٹنگ
- • پروٹوکولز کو بہتر بنائیں
- • لوڈ بیلنگ

**سیمولیشن حقیقت سے مماثل نہیں ہے**
- • سینسر ماڈلز کی توثیق کریں
- • فزکس کو فائن ٹیون کریں
- • ڈومین رینڈمائزیشن
- • حقیقی نوائز ماڈلز

<div className="border-line"></div>

<h2 className="second-heading">خلاصہ</h2>

<div className="border-line"></div>

اعلیٰ سیمولیشن کی تکنیکیں ہیومنوائڈ روبوٹس کی ٹیسٹنگ کے لیے پیچیدہ، حقیقی ماحول کو فعال کرتی ہیں۔ متعدد روبوٹ کوآرڈینیشن، اعلیٰ فزکس ماڈلز، متحرک ماحول، اور کارکردگی کی بہتری سیمولیشن سسٹم بناتی ہیں جو حقیقی دنیا کے چیلنجز کو عکاس کرتے ہیں۔ کامیابی کے لیے محسابی کارکردگی اور جسمانی درستگی کے درمیان توازن، جبکہ سیمولیشن-سے-حقیقت منتقلی کی صلاحیت برقرار رکھنے کی ضرورت ہوتی ہے۔