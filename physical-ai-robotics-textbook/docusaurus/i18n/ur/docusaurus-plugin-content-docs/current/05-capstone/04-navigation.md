---
sidebar_position: 4
title: "نیویگیشن سسٹم یکجہتی"
description: "ہیومنوائڈ روبوٹ کے لیے خودکار نیویگیشن اور SLAM یکجہتی"
---

import ReadingTime from '@site/src/components/ReadingTime';
import ViewToggle from '@site/src/components/ViewToggle';


<ReadingTime minutes={30} />

<h1 className="main-heading">نیویگیشن سسٹم یکجہتی</h1>
<div className="underline-class"></div>

<div className="full-content">

<div className="border-line"></div>

<h2 className="second-heading">سیکھنے کے اہداف</h2>
<div className="underline-class"></div>

- ہیومنوائڈ روبوٹ کے لیے SLAM سسٹم ڈیزائن کرنا
- خودکار نیویگیشن الگورتھم نافذ کرنا
- رکاوٹوں سے بچاؤ اور محفوظ نیویگیشن کو لاگو کرنا
- چلنے والے ہیومنوائڈ کے لیے توازن اور اسٹیبیلٹی کنٹرول کرنا
- نیویگیشن کارکردگی کا جائزہ لینا

<div className="border-line"></div>

<h2 className="second-heading">تعارف</h2>
<div className="underline-class"></div>

نیویگیشن سسٹم یکجہتی ہیومنوائڈ روبوٹ کو ماحول میں جگہ لینے، نقشہ بنانے، اور مطلوبہ منزل تک محفوظ طریقے سے جانے کے قابل بناتی ہے۔ یہ SLAM، پاتھ پلاننگ، اور ہومنوائڈ کنٹرول کے تصورات کو جمع کرتی ہے۔

<div className="border-line"></div>

<h2 className="second-heading">SLAM ایلگورتھم</h2>
<div className="underline-class"></div>

<h3 className="third-heading"> میپ بنانا اور لوکلائزیشن</h3>
<div className="underline-class"></div>

```python
class SLAMSystem:
    def __init__(self):
        self.mapper = ORB_SLAM3()
        self.localizer = ParticleFilter()
        self.map = OccupancyGrid()

    def update_map_and_pose(self, sensor_data):
        # سینسر ڈیٹا کے ساتھ میپ اور پوز کو اپ ڈیٹ کریں
        new_features = self.extract_features(sensor_data)
        self.map.update(new_features)
        self.localizer.update_pose(new_features)
        return self.localizer.get_current_pose()
```

<div className="border-line"></div>

<h3 className="third-heading"> چلنے والے ہیومنوائڈ کے لیے خصوصیات</h3>
<div className="underline-class"></div>

- ہیومنوائڈ کے متحرک ماڈل کو SLAM میں ضم کرنا
- چلنے کے دوران توازن کو مستحکم کرنا
- IMU اور گیرو اسکوپ ڈیٹا کا استعمال کرنا

<div className="border-line"></div>

<h2 className="second-heading">پاتھ پلاننگ</h2>
<div className="underline-class"></div>

<h3 className="third-heading"> گلوبل پلاننگ</h3>
<div className="underline-class"></div>

```python
class GlobalPlanner:
    def __init__(self):
        self.path_finder = AStar()
        self.cost_map = Costmap2D()

    def plan_path(self, start_pose, goal_pose, map_data):
        # گلوبل پاتھ پلان کریں
        global_path = self.path_finder.find_path(start_pose, goal_pose, self.cost_map)
        return self.smooth_path(global_path)
```

<div className="border-line"></div>

<h3 className="third-heading"> لوکل پلاننگ</h3>
<div className="underline-class"></div>

```python
class LocalPlanner:
    def __init__(self):
        self.trajectory_generator = TrajectoryGenerator()
        self.collision_checker = CollisionChecker()

    def plan_local_trajectory(self, global_path, current_pose, sensor_data):
        # مقامی ٹریجکٹری پلان کریں
        local_trajectory = self.trajectory_generator.generate(
            global_path, current_pose, sensor_data
        )
        return self.collision_checker.check(local_trajectory)
```

<div className="border-line"></div>

<h2 className="second-heading">ہیومنوائڈ نیویگیشن کنٹرول</h2>
<div className="underline-class"></div>

<h3 className="third-heading"> چلنے کا کنٹرول</h3>
<div className="underline-class"></div>

```python
class WalkingController:
    def __init__(self):
        self.zmp_controller = ZMPController()
        self.com_balancer = CoMBalancer()
        self.footstep_planner = FootstepPlanner()

    def generate_walking_trajectory(self, target_velocity):
        # چلنے کا ٹریجکٹری جنریٹ کریں
        footsteps = self.footstep_planner.plan(target_velocity)
        zmp_trajectory = self.zmp_controller.plan(footsteps)
        com_trajectory = self.com_balancer.balance(zmp_trajectory)
        return com_trajectory, footsteps
```

<div className="border-line"></div>

<h3 className="third-heading"> توازن کنٹرول</h3>
<div className="underline-class"></div>

```python
class BalanceController:
    def __init__(self):
        self.pendulum_model = InvertedPendulumModel()
        self.ik_solver = InverseKinematicsSolver()

    def maintain_balance(self, current_state, target_state):
        # ہیومنوائڈ کا توازن برقرار رکھیں
        desired_com = self.pendulum_model.calculate_com(target_state)
        joint_commands = self.ik_solver.solve_balance(desired_com, current_state)
        return joint_commands
```

<div className="border-line"></div>

<h2 className="second-heading">رکاوٹوں سے بچاؤ</h2>
<div className="underline-class"></div>

<h3 className="third-heading"> سینسر فیوژن</h3>
<div className="underline-class"></div>

```python
class ObstacleAvoidance:
    def __init__(self):
        self.lidar_processor = LidarProcessor()
        self.camera_processor = CameraProcessor()
        self.fusion_engine = SensorFusion()

    def detect_and_avoid_obstacles(self, lidar_data, camera_data):
        lidar_obstacles = self.lidar_processor.process(lidar_data)
        vision_obstacles = self.camera_processor.process(camera_data)
        fused_obstacles = self.fusion_engine.fuse(lidar_obstacles, vision_obstacles)
        avoidance_trajectory = self.calculate_avoidance_path(fused_obstacles)
        return avoidance_trajectory
```

<div className="border-line"></div>

<h2 className="second-heading">ROS 2 نیویگیشن اسٹیک</h2>
<div className="underline-class"></div>

<h3 className="third-heading"> تخصیص شدہ نیویگیشن نوڈس</h3>
<div className="underline-class"></div>

```python
class HumanoidNavigator(Node):
    def __init__(self):
        super().__init__('humanoid_navigator')
        self.slam_node = SLAMNode()
        self.planner_node = PlannerNode()
        self.controller_node = ControllerNode()

    def navigate_to_pose(self, goal_pose):
        # ہیومنوائڈ کے لیے نیویگیشن کریں
        self.planner_node.plan_to_pose(goal_pose)
        self.controller_node.follow_path()
```

<div className="border-line"></div>

<h2 className="second-heading">کارکردگی کی بہتری</h2>
<div className="underline-class"></div>

<h3 className="third-heading"> ریل ٹائم کارکردگی</h3>
<div className="underline-class"></div>

- SLAM الگورتھم کو کم کرنے کے لیے ہیورسٹکس
- ہیومنوائڈ کنٹرول کے لیے ہارڈویئر ایکسلریشن
- سینسر ڈیٹا کے لیے ایڈاپٹو سیمپلنگ

<div className="border-line"></div>

<h2 className="second-heading">ٹیسٹنگ اور جائزہ</h2>
<div className="underline-class"></div>

<h3 className="third-heading"> نیویگیشن میٹرکس</h3>
<div className="underline-class"></div>

- پاتھ کی کارآمدی
- منزل تک پہنچنے کی کامیابی کی شرح
- توازن برقرار رکھنے کی کارکردگی
- رکاوٹوں سے بچاؤ کی مؤثرتا

<div className="border-line"></div>

<h2 className="second-heading">چیلنج اور حل</h2>
<div className="underline-class"></div>

- چلنے کے دوران ہیومنوائڈ کا توازن برقرار رکھنا
- SLAM کی ناکامی کے دوران نیویگیشن کو جاری رکھنا
- گھنے ماحول میں رکاوٹوں سے بچاؤ
- کم لائٹ کے حالات میں وژن بیسڈ SLAM

<div className="border-line"></div>

<h2 className="second-heading">خلاصہ</h2>
<div className="underline-class"></div>

نیویگیشن سسٹم یکجہتی ہیومنوائڈ روبوٹ کو محفوظ اور مؤثر طریقے سے ماحول میں نقل و حمل کرنے کے قابل بناتی ہے۔ ہیومنوائڈ کے مخصوص چیلنجوں کو حل کرنا ضروری ہے۔

<div className="border-line"></div>

<h2 className="second-heading">ورقے</h2>
<div className="underline-class"></div>

1. SLAM سسٹم نافذ کریں
2. ہیومنوائڈ چلنے کا کنٹرول ڈیزائن کریں
3. رکاوٹوں سے بچاؤ الگورتھم ٹیسٹ کریں
4. نیویگیشن کارکردگی کا جائزہ لیں
5. توازن کنٹرول سسٹم کی توثیق کریں

</div>