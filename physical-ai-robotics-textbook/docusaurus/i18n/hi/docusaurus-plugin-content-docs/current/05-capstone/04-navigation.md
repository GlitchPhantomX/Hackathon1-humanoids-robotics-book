---
sidebar_position: 4
title: "स्वायत्त नेविगेशन सिस्टम"
description: "ह्यूमनॉइड रोबोट के लिए स्वायत्त नेविगेशन लागू करना"
---

import ReadingTime from '@site/src/components/ReadingTime';

<ReadingTime minutes={35} />


<h1 className="main-heading">स्वायत्त नेविगेशन सिस्टम</h1>
<div className="underline-class"></div>

<div className="full-content">

<div className="border-line"></div>

<h2 className="second-heading">सीखने के उद्देश्य</h2>
<div className="underline-class"></div>

- व्यापक स्वायत्त नेविगेशन सिस्टम डिज़ाइन करना
- धारणा, मैपिंग और पाथ योजना को एकीकृत करना
- गतिशील बाधा पहचान कार्यान्वित करना
- मजबूत स्थानीयकरण और मैपिंग प्रणाली बनाना
- मानव वातावरण में सुरक्षित नेविगेशन सुनिश्चित करना

<div className="border-line"></div>

<h2 className="second-heading">परिचय</h2>
<div className="underline-class"></div>

स्वायत्त नेविगेशन ह्यूमनॉइड रोबोट को बाधाओं से बचते हुए जटिल वातावरण में सुरक्षित रूप से चलने में सक्षम बनाता है। पहिया वाले रोबोट के विपरीत, ह्यूमनॉइड नेविगेशन को जटिल काइनेमैटिक्स, संतुलन और मानव-डिज़ाइन किए गए स्थानों को संभालना पड़ता है। सिस्टम धारणा, योजना, नियंत्रण और सुरक्षा को विजन-भाषा-क्रिया पैराडिम का उपयोग करके एकीकृत करता है।

<div className="border-line"></div>

<h2 className="second-heading">मैपिंग और स्थानीयकरण</h2>
<div className="underline-class"></div>

<h3 className="third-heading">SLAM सिस्टम</h3>

```python
import numpy as np
import open3d as o3d

class SLAMSystem:
    def __init__(self, voxel_size=0.1, max_correspondence=0.2):
        self.voxel_size = voxel_size
        self.max_correspondence = max_correspondence
        self.global_map = o3d.geometry.PointCloud()
        self.keyframes = []
        self.poses = []

    def process_frame(self, point_cloud, camera_pose=None):
        if len(self.poses) == 0:
            self.poses = [camera_pose if camera_pose else np.eye(4)]
            self.global_map.points = o3d.utility.Vector3dVector(point_cloud)
            return {'success': True, 'pose': self.poses[0]}

        current_cloud = o3d.geometry.PointCloud()
        current_cloud.points = o3d.utility.Vector3dVector(point_cloud)
        current_cloud_down = current_cloud.voxel_down_sample(self.voxel_size)

        # ग्लोबल मैप के साथ पंजीकृत करें
        result = self.register_clouds(current_cloud_down, self.global_map)

        if result.fitness > 0.5:
            current_pose = result.transformation
            self.poses.append(current_pose)
            self.global_map += current_cloud_down.transform(current_pose)
            return {'success': True, 'pose': current_pose, 'fitness': result.fitness}

        return {'success': False}

    def register_clouds(self, source, target):
        source.estimate_normals()
        target.estimate_normals()
        return o3d.pipelines.registration.registration_icp(
            source, target, self.max_correspondence, np.eye(4)
        )
```

<div className="border-line"></div>

<h3 className="third-heading">अधिकार ग्रिड</h3>

```python
class OccupancyGridMap:
    def __init__(self, resolution=0.05, width=200, height=200):
        self.resolution = resolution
        self.width = width
        self.height = height
        self.grid = np.full((height, width), -1, dtype=np.int8)

    def world_to_grid(self, x, y):
        grid_x = int(x / self.resolution)
        grid_y = int(y / self.resolution)
        return max(0, min(self.width-1, grid_x)), max(0, min(self.height-1, grid_y))

    def update_with_scan(self, ranges, angles, robot_pose):
        robot_x, robot_y, robot_theta = robot_pose

        for range_val, angle in zip(ranges, angles):
            if 0.1 < range_val < 10.0:
                obs_x = robot_x + range_val * np.cos(robot_theta + angle)
                obs_y = robot_y + range_val * np.sin(robot_theta + angle)
                obs_gx, obs_gy = self.world_to_grid(obs_x, obs_y)
                self.grid[obs_gy, obs_gx] = 100  # अधिकृत
                self.ray_trace(robot_x, robot_y, obs_x, obs_y)

    def is_occupied(self, x, y):
        gx, gy = self.world_to_grid(x, y)
        return self.grid[gy, gx] > 50
```

<div className="border-line"></div>

<h3 className="third-heading">स्थानीयकरण</h3>

```python
class LocalizationSystem:
    def __init__(self, num_particles=1000):
        self.num_particles = num_particles
        self.particles = self.initialize_particles()

    def initialize_particles(self):
        particles = []
        for _ in range(self.num_particles):
            x = np.random.uniform(-10, 10)
            y = np.random.uniform(-10, 10)
            theta = np.random.uniform(-np.pi, np.pi)
            particles.append({'pose': np.array([x, y, theta]), 'weight': 1.0/self.num_particles})
        return particles

    def update_pose(self, odom_delta, sensor_data, map_data):
        # भविष्यवाणी
        self.predict_particles(odom_delta)

        # वजन अपडेट करें
        self.update_weights(sensor_data, map_data)

        # पुन: नमूना
        self.resample_particles()

        return {'pose': self.calculate_estimate(), 'uncertainty': self.calc_uncertainty()}

    def predict_particles(self, odom_delta):
        dx, dy, dtheta = odom_delta
        for p in self.particles:
            p['pose'][0] += dx + np.random.normal(0, 0.1)
            p['pose'][1] += dy + np.random.normal(0, 0.1)
            p['pose'][2] += dtheta + np.random.normal(0, 0.05)

    def calculate_estimate(self):
        x = sum(p['weight'] * p['pose'][0] for p in self.particles)
        y = sum(p['weight'] * p['pose'][1] for p in self.particles)
        theta = sum(p['weight'] * p['pose'][2] for p in self.particles)
        return np.array([x, y, theta])
```

<div className="border-line"></div>

<h2 className="second-heading">पाथ योजना</h2>
<div className="underline-class"></div>

<h3 className="third-heading">ग्लोबल प्लानर (A*)</h3>

```python
import heapq

class GlobalPathPlanner:
    def __init__(self, map_resolution=0.05, inflation_radius=0.3):
        self.map_resolution = map_resolution
        self.inflation_radius = inflation_radius
        self.occupancy_map = None

    def plan_path(self, start, goal):
        if not self.occupancy_map:
            return None

        start_grid = self.occupancy_map.world_to_grid(*start)
        goal_grid = self.occupancy_map.world_to_grid(*goal)

        path_grid = self.a_star(start_grid, goal_grid)
        if not path_grid:
            return None

        # वर्ल्ड निर्देशांक में बदलें
        return [self.occupancy_map.grid_to_world(x, y) for x, y in path_grid]

    def a_star(self, start, goal):
        def heuristic(a, b):
            return abs(a[0]-b[0]) + abs(a[1]-b[1])

        open_set = [(0, 0, start)]
        came_from = {}
        g_score = {start: 0}

        while open_set:
            current = heapq.heappop(open_set)[2]

            if current == goal:
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                return path[::-1]

            for dx, dy in [(-1,0), (1,0), (0,-1), (0,1), (-1,-1), (-1,1), (1,-1), (1,1)]:
                neighbor = (current[0]+dx, current[1]+dy)

                if not self.is_valid(neighbor):
                    continue

                cost = np.sqrt(dx**2 + dy**2)
                tentative_g = g_score[current] + cost

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score, g_score[neighbor], neighbor))

        return None
```

<div className="border-line"></div>

<h3 className="third-heading">लोकल प्लानर (DWA)</h3>

```python
class LocalPathPlanner:
    def __init__(self, horizon=2.0):
        self.horizon = horizon
        self.max_linear_vel = 0.5
        self.max_angular_vel = 0.5

    def plan_local(self, current_pose, global_path, sensor_data):
        path_segment = self.get_segment(current_pose, global_path)
        obstacles = self.extract_obstacles(sensor_data)

        if obstacles:
            vel_cmd = self.dwa_plan(current_pose, path_segment, obstacles)
        else:
            vel_cmd = self.pure_pursuit(current_pose, path_segment)

        return {'linear': vel_cmd[0], 'angular': vel_cmd[1]}

    def dwa_plan(self, pose, path, obstacles):
        best_score = -float('inf')
        best_vel = (0.0, 0.0)

        for v in np.arange(0, self.max_linear_vel, 0.1):
            for w in np.arange(-self.max_angular_vel, self.max_angular_vel, 0.1):
                future_pos = self.simulate(pose, v, w)
                score = self.calc_score(future_pos, path, obstacles)

                if score > best_score:
                    best_score = score
                    best_vel = (v, w)

        return best_vel

    def simulate(self, pose, v, w, dt=0.1, steps=10):
        x, y, theta = pose
        for _ in range(steps):
            x += v * np.cos(theta) * dt
            y += v * np.sin(theta) * dt
            theta += w * dt
        return (x, y, theta)
```

<div className="border-line"></div>

<h2 className="second-heading">Isaac एकीकरण</h2>
<div className="underline-class"></div>

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

class IsaacNavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')

        # पब्लिशर
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # सब्सक्राइबर
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)

        # नेविगेशन सिस्टम
        self.localization = LocalizationSystem()
        self.global_planner = GlobalPathPlanner()
        self.local_planner = LocalPathPlanner()

        self.current_pose = np.eye(4)
        self.global_path = []
        self.navigation_active = False

    def set_goal(self, goal_x, goal_y):
        current_pos = (self.current_pose[0,3], self.current_pose[1,3])
        self.global_path = self.global_planner.plan_path(current_pos, (goal_x, goal_y))

        if self.global_path:
            self.navigation_active = True
            return True
        return False

    def scan_cb(self, msg):
        if not self.navigation_active:
            return

        sensor_data = {'ranges': list(msg.ranges), 'angles': list(msg.angles)}
        local_plan = self.local_planner.plan_local(self.current_pose, self.global_path, sensor_data)

        cmd = Twist()
        cmd.linear.x = local_plan['linear']
        cmd.angular.z = local_plan['angular']
        self.cmd_vel_pub.publish(cmd)
```

<div className="border-line"></div>

<h2 className="second-heading">सुरक्षा सिस्टम</h2>
<div className="underline-class"></div>

```python
class NavigationSafetySystem:
    def __init__(self):
        self.thresholds = {
            'min_obstacle_distance': 0.5,
            'max_linear_vel': 0.5,
            'max_angular_vel': 0.5,
            'human_proximity': 1.0
        }
        self.emergency_stop = False

    def validate_command(self, cmd_vel, sensor_data):
        if self.emergency_stop:
            return {'safe': False, 'reason': 'आपातकाल सक्रिय'}

        # वेलोसिटी सीमा जांचें
        if abs(cmd_vel.linear.x) > self.thresholds['max_linear_vel']:
            return {'safe': False, 'reason': 'वेलोसिटी सीमा से अधिक'}

        # बाधा की निकटता जांचें
        if 'ranges' in sensor_data:
            min_range = min(sensor_data['ranges'])
            if min_range < self.thresholds['min_obstacle_distance']:
                return {'safe': False, 'reason': 'बाधा बहुत करीब है'}

        return {'safe': True, 'reason': 'सभी जांच पास'}

    def trigger_emergency(self):
        self.emergency_stop = True

    def clear_emergency(self):
        self.emergency_stop = False
```

<div className="border-line"></div>

<h2 className="second-heading">प्रदर्शन अनुकूलन</h2>
<div className="underline-class"></div>

```python
class RealTimeNavigationOptimizer:
    def __init__(self):
        self.timing_requirements = {
            'localization': 0.1,    # 10Hz
            'global_planning': 1.0,  # 1Hz
            'local_planning': 0.05   # 20Hz
        }
        self.monitor = PerformanceMonitor()

    def optimize_localization(self, sensor_data):
        start = time.time()

        # अनुकूलिक डाउनसैंपलिंग
        if self.monitor.get_avg('localization') > 0.05:
            sensor_data = self.downsample(sensor_data, 0.5)

        result = self.perform_localization(sensor_data)
        self.monitor.record('localization', time.time() - start)
        return result

    def optimize_planning(self, start, goal, map_data):
        # पदानुक्रमित मोटे-से-ठीक योजना
        coarse_path = self.plan_coarse(start, goal, map_data)
        refined_path = self.refine_path(coarse_path, map_data)
        return refined_path

class PerformanceMonitor:
    def __init__(self):
        self.times = {}

    def record(self, component, time):
        if component not in self.times:
            self.times[component] = []
        self.times[component].append(time)

    def get_avg(self, component):
        return np.mean(self.times[component]) if component in self.times else float('inf')
```

<div className="border-line"></div>

<h2 className="second-heading">सारांश</h2>
<div className="underline-class"></div>

**मुख्य घटक:**

1. **SLAM & मैपिंग**: रोबोट स्थिति ट्रैकिंग के साथ पर्यावरण मैप बनाएं और बनाए रखें
2. **स्थानीयकरण**: कण फ़िल्टर-आधारित सटीक स्थिति निर्धारण
3. **पाथ योजना**: ह्यूमनॉइड काइनेमैटिक्स के लिए पदानुक्रमित ग्लोबल (A*) और लोकल (DWA) योजना
4. **Isaac एकीकरण**: हार्डवेयर एक्सेलरेशन के साथ विशिष्ट नेविगेशन घटक
5. **सुरक्षा प्रणाली**: सुरक्षित मानव इंटरैक्शन सुनिश्चित करने के लिए व्यापक मान्यता
6. **प्रदर्शन अनुकूलन**: समय और सटीकता के लिए रीयल-टाइम तकनीकें

**सफलता कारक**: धारणा, योजना, नियंत्रण और सुरक्षा का एकीकरण ह्यूमनॉइड काइनेमैटिक्स, संतुलन और सुरक्षित मानव इंटरैक्शन पर ध्यान केंद्रित करके।

<div className="border-line"></div>

<h2 className="second-heading">अभ्यास</h2>
<div className="underline-class"></div>

1. RGB-D सेंसर डेटा का उपयोग करके SLAM लागू करें
2. ह्यूमनॉइड कदम सीमा के साथ पाथ प्लानर बनाएं
3. गतिशील बाधाओं के लिए लोकल प्लानर डिज़ाइन करें
4. सुरक्षा मान्यता प्रणाली बनाएं
5. रीयल-टाइम प्रदर्शन के लिए एल्गोरिदम अनुकूलित करें

<div className="border-line"></div>

<h2 className="second-heading">आगे की पढ़ाई</h2>
<div className="underline-class"></div>

- "Probabilistic Robotics" थ्रन, बर्गर्ड और फ़ॉक्स द्वारा
- "Springer Handbook of Robotics" सिसिलियानो और खातिब द्वारा
- "Planning Algorithms" लावैले द्वारा
- नेविगेशन पर NVIDIA Isaac दस्तावेज़ीकरण

</div>