---
sidebar_position: 4
title: "Autonomous Navigation System"
description: "Implementing autonomous navigation for humanoid robots in complex environments"
---

import ReadingTime from '@site/src/components/ReadingTime';
import ViewToggle from '@site/src/components/ViewToggle';

<ReadingTime minutes={156} />

<!-- <ViewToggle /> -->

<h1 className="main-heading">Autonomous Navigation System</h1>
<div className="underline-class"></div>

<div className="full-content">

<div className="border-line"></div>
---

<h2 className="second-heading">
 Learning Objectives
</h2>
<div className="underline-class"></div>

After completing this chapter, you will be able to:
- • Design and implement a comprehensive autonomous navigation system for humanoid robots
- • Integrate perception, mapping, and path planning for safe navigation
- • Implement dynamic obstacle avoidance and replanning capabilities
- • Create robust localization and mapping systems for unknown environments
- • Ensure safe and efficient navigation in human-populated environments

<div className="border-line"></div>
---

<h2 className="second-heading">
 Introduction to Autonomous Navigation
</h2>
<div className="underline-class"></div>

Autonomous navigation represents one of the most fundamental capabilities for humanoid robots, enabling them to move safely and efficiently through complex environments while avoiding obstacles and reaching desired destinations. Unlike wheeled robots that operate primarily in planar environments, humanoid robots must navigate with complex kinematics, balance considerations, and the need to operate in spaces designed for humans.

The navigation system for a humanoid robot must address multiple challenges simultaneously: creating and maintaining maps of the environment, localizing the robot within those maps, planning safe and efficient paths to goals, executing those paths while maintaining balance, and continuously adapting to dynamic changes in the environment. This requires tight integration between perception, planning, control, and safety systems.

This chapter explores the implementation of a comprehensive autonomous navigation system for humanoid robots, focusing on the Vision-Language-Action paradigm that connects environmental perception to goal-directed navigation. The system must be robust enough to handle real-world conditions while maintaining the safety and efficiency required for human-robot interaction.

<h2 className="second-heading">
Mapping and Localization
</h2>
<div className="underline-class"></div>

<h3 className="third-heading">
- Simultaneous Localization and Mapping (SLAM)
</h3>
<div className="underline-class"></div>

SLAM is fundamental to navigation in unknown environments, allowing the robot to build maps while simultaneously determining its position within those maps:

```python
# Example: SLAM implementation for humanoid robot
import numpy as np
import cv2
from typing import List, Dict, Any, Tuple, Optional
import threading
import time
from scipy.spatial.transform import Rotation as R
from scipy.spatial.distance import cdist
import open3d as o3d

class SLAMSystem:
    def __init__(self, voxel_size: float = 0.1, max_correspondence_distance: float = 0.2):
        self.voxel_size = voxel_size
        self.max_correspondence_distance = max_correspondence_distance

        # Map representation
        self.global_map = o3d.geometry.PointCloud()
        self.keyframes = []  # Store keyframe poses and point clouds
        self.poses = []      # Store robot poses over time

        # Feature extraction parameters
        self.feature_extractor = cv2.SIFT_create()

        # Registration parameters
        self.registration_method = o3d.pipelines.registration.registration_icp
        self.transformation_estimator = o3d.pipelines.registration.TransformationEstimationPointToPlane()

        # Threading for real-time processing
        self.slam_lock = threading.RLock()
        self.is_running = False

        # Motion model for pose prediction
        self.previous_pose = np.eye(4)
        self.velocity_estimate = np.zeros(6)  # [vx, vy, vz, wx, wy, wz]

    def initialize_map(self, initial_pose: np.ndarray = None):
        """Initialize the SLAM system with an initial pose"""
        if initial_pose is None:
            initial_pose = np.eye(4)

        self.poses = [initial_pose]
        self.previous_pose = initial_pose.copy()
        self.keyframes = [(initial_pose, o3d.geometry.PointCloud())]

    def process_frame(self, point_cloud: np.ndarray, camera_pose: np.ndarray = None) -> Dict[str, Any]:
        """Process a new sensor frame for SLAM"""
        with self.slam_lock:
            if len(self.poses) == 0:
                # Initialize with first frame
                initial_pose = camera_pose if camera_pose is not None else np.eye(4)
                self.initialize_map(initial_pose)
                self.global_map.points = o3d.utility.Vector3dVector(point_cloud)
                return {
                    'success': True,
                    'pose': initial_pose,
                    'new_keyframe': True,
                    'map_updated': True
                }

            # Convert point cloud to Open3D format
            current_cloud = o3d.geometry.PointCloud()
            current_cloud.points = o3d.utility.Vector3dVector(point_cloud)

            # Downsample for efficiency
            current_cloud_down = current_cloud.voxel_down_sample(voxel_size=self.voxel_size)

            # Estimate initial transformation using motion model
            predicted_pose = self.predict_pose()

            # Perform registration to find actual transformation
            result = self.register_point_clouds(
                current_cloud_down,
                self.global_map,
                predicted_pose
            )

            if result.fitness > 0.5:  # Good alignment threshold
                # Update pose
                current_pose = result.transformation
                self.poses.append(current_pose)

                # Update global map
                current_cloud_transformed = current_cloud_down.transform(current_pose)
                self.global_map += current_cloud_transformed

                # Check if this should be a new keyframe
                is_keyframe = self.should_add_keyframe(current_pose)

                if is_keyframe:
                    self.keyframes.append((current_pose, current_cloud_transformed))

                # Update motion model
                self.update_motion_model(current_pose)

                return {
                    'success': True,
                    'pose': current_pose,
                    'new_keyframe': is_keyframe,
                    'map_updated': True,
                    'fitness': result.fitness,
                    'inlier_rmse': result.inlier_rmse
                }
            else:
                # Registration failed, use predicted pose
                self.poses.append(predicted_pose)
                return {
                    'success': False,
                    'pose': predicted_pose,
                    'new_keyframe': False,
                    'map_updated': False,
                    'fitness': result.fitness if hasattr(result, 'fitness') else 0.0
                }

    def register_point_clouds(self, source: o3d.geometry.PointCloud,
                            target: o3d.geometry.PointCloud,
                            initial_guess: np.ndarray) -> o3d.pipelines.registration.RegistrationResult:
        """Register two point clouds using ICP"""
        # Estimate normals for point-to-plane registration
        source.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30)
        )
        target.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30)
        )

        # Perform ICP registration
        result = o3d.pipelines.registration.registration_icp(
            source, target,
            max_correspondence_distance=self.max_correspondence_distance,
            init=initial_guess,
            estimation_method=self.transformation_estimator,
            criteria=o3d.pipelines.registration.ICPConvergenceCriteria(
                relative_fitness=1e-6,
                relative_rmse=1e-6,
                max_iteration=100
            )
        )

        return result

    def predict_pose(self) -> np.ndarray:
        """Predict current pose based on motion model"""
        if len(self.poses) < 2:
            return self.previous_pose.copy()

        # Simple constant velocity model
        dt = 0.1  # Assume 10Hz update rate
        translation = self.velocity_estimate[:3] * dt
        rotation_vec = self.velocity_estimate[3:] * dt

        # Create transformation matrix
        rotation = R.from_rotvec(rotation_vec).as_matrix()
        translation_matrix = np.eye(4)
        translation_matrix[:3, :3] = rotation
        translation_matrix[:3, 3] = translation

        predicted_pose = self.previous_pose @ translation_matrix
        return predicted_pose

    def update_motion_model(self, current_pose: np.ndarray):
        """Update motion model based on pose change"""
        if len(self.poses) < 2:
            return

        # Calculate pose difference
        prev_pose = self.poses[-2]
        pose_diff = np.linalg.inv(prev_pose) @ current_pose

        # Extract translation and rotation
        translation = pose_diff[:3, 3]
        rotation_matrix = pose_diff[:3, :3]
        rotation_vec = R.from_matrix(rotation_matrix).as_rotvec()

        # Calculate velocities (assuming constant time step)
        dt = 0.1  # 10Hz
        self.velocity_estimate[:3] = translation / dt
        self.velocity_estimate[3:] = rotation_vec / dt

        self.previous_pose = current_pose.copy()

    def should_add_keyframe(self, current_pose: np.ndarray) -> bool:
        """Determine if current frame should be added as a keyframe"""
        if not self.keyframes:
            return True

        # Check distance from last keyframe
        last_keyframe_pose = self.keyframes[-1][0]
        position_diff = np.linalg.norm(current_pose[:3, 3] - last_keyframe_pose[:3, 3])

        # Add keyframe if moved significantly
        return position_diff > 0.5  # 50cm threshold

    def get_global_map(self) -> o3d.geometry.PointCloud:
        """Get the current global map"""
        with self.slam_lock:
            return self.global_map.voxel_down_sample(voxel_size=self.voxel_size)

    def get_current_pose(self) -> np.ndarray:
        """Get the current estimated pose"""
        with self.slam_lock:
            if self.poses:
                return self.poses[-1].copy()
            else:
                return np.eye(4)

    def save_map(self, filename: str) -> bool:
        """Save the current map to file"""
        with self.slam_lock:
            try:
                o3d.io.write_point_cloud(filename, self.global_map)
                return True
            except Exception as e:
                print(f"Error saving map: {e}")
                return False

    def load_map(self, filename: str) -> bool:
        """Load a map from file"""
        with self.slam_lock:
            try:
                self.global_map = o3d.io.read_point_cloud(filename)
                return True
            except Exception as e:
                print(f"Error loading map: {e}")
                return False

class OccupancyGridMap:
    """2D occupancy grid for navigation planning"""
    def __init__(self, resolution: float = 0.05, width: int = 200, height: int = 200):
        self.resolution = resolution
        self.width = width
        self.height = height
        self.origin_x = 0.0
        self.origin_y = 0.0

        # Initialize grid (0 = free, 100 = occupied, -1 = unknown)
        self.grid = np.full((height, width), -1, dtype=np.int8)

        # Probabilities for updating grid
        self.prob_free = 0.3
        self.prob_occ = 0.7
        self.prob_thresh = 0.5

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world coordinates to grid coordinates"""
        grid_x = int((x - self.origin_x) / self.resolution)
        grid_y = int((y - self.origin_y) / self.resolution)

        # Check bounds
        grid_x = max(0, min(self.width - 1, grid_x))
        grid_y = max(0, min(self.height - 1, grid_y))

        return grid_x, grid_y

    def grid_to_world(self, grid_x: int, grid_y: int) -> Tuple[float, float]:
        """Convert grid coordinates to world coordinates"""
        x = grid_x * self.resolution + self.origin_x
        y = grid_y * self.resolution + self.origin_y
        return x, y

    def update_with_laser_scan(self, laser_ranges: List[float],
                             laser_angles: List[float],
                             robot_pose: np.ndarray):
        """Update occupancy grid with laser scan data"""
        robot_x, robot_y, robot_theta = self.pose_to_2d(robot_pose)

        for i, (range_val, angle) in enumerate(zip(laser_ranges, laser_angles)):
            if range_val < 0.1 or range_val > 10.0:  # Invalid range
                continue

            # Calculate obstacle position in world coordinates
            obs_angle = robot_theta + angle
            obs_x = robot_x + range_val * np.cos(obs_angle)
            obs_y = robot_y + range_val * np.sin(obs_angle)

            # Calculate grid coordinates
            obs_grid_x, obs_grid_y = self.world_to_grid(obs_x, obs_y)

            # Update obstacle cell
            self.update_cell(obs_grid_x, obs_grid_y, self.prob_occ)

            # Update free space along the ray
            self.ray_trace(robot_x, robot_y, obs_x, obs_y)

    def update_cell(self, grid_x: int, grid_y: int, prob_occ: float):
        """Update a single grid cell with new occupancy probability"""
        if 0 <= grid_x < self.width and 0 <= grid_y < self.height:
            # Convert probability to log-odds
            p = prob_occ
            l = np.log(p / (1 - p))

            # Update log-odds
            current_l = self.prob_to_logodds(self.grid[grid_y, grid_x])
            new_l = current_l + l - self.prob_to_logodds(0.5)  # Subtract prior

            # Convert back to probability and store as occupancy value (0-100)
            new_p = self.logodds_to_prob(new_l)
            self.grid[grid_y, grid_x] = int(new_p * 100)

    def ray_trace(self, start_x: float, start_y: float, end_x: float, end_y: float):
        """Ray trace to update free space between robot and obstacle"""
        start_grid_x, start_grid_y = self.world_to_grid(start_x, start_y)
        end_grid_x, end_grid_y = self.world_to_grid(end_x, end_y)

        # Bresenham's line algorithm for ray tracing
        dx = abs(end_grid_x - start_grid_x)
        dy = abs(end_grid_y - start_grid_y)
        x_step = 1 if end_grid_x > start_grid_x else -1
        y_step = 1 if end_grid_y > start_grid_y else -1

        error = dx - dy
        x, y = start_grid_x, start_grid_y

        while x != end_grid_x or y != end_grid_y:
            if 0 <= x < self.width and 0 <= y < self.height:
                if not (x == end_grid_x and y == end_grid_y):  # Don't update obstacle cell
                    self.update_cell(x, y, self.prob_free)

            # Move to next cell
            error2 = 2 * error
            if error2 > -dy:
                error -= dy
                x += x_step
            if error2 < dx:
                error += dx
                y += y_step

    def pose_to_2d(self, pose: np.ndarray) -> Tuple[float, float, float]:
        """Extract 2D pose (x, y, theta) from 4x4 transformation matrix"""
        x = pose[0, 3]
        y = pose[1, 3]

        # Extract rotation angle from rotation matrix
        theta = np.arctan2(pose[1, 0], pose[0, 0])

        return x, y, theta

    def prob_to_logodds(self, p: float) -> float:
        """Convert probability to log-odds"""
        p = max(0.01, min(0.99, p))  # Clamp to avoid log(0)
        return np.log(p / (1 - p))

    def logodds_to_prob(self, l: float) -> float:
        """Convert log-odds to probability"""
        p = 1 - 1 / (1 + np.exp(l))
        return p

    def is_occupied(self, x: float, y: float) -> bool:
        """Check if a position is occupied"""
        grid_x, grid_y = self.world_to_grid(x, y)
        if 0 <= grid_x < self.width and 0 <= grid_y < self.height:
            return self.grid[grid_y, grid_x] > 50  # Threshold for occupancy
        return True  # Out of bounds is considered occupied

    def is_free(self, x: float, y: float) -> bool:
        """Check if a position is free"""
        grid_x, grid_y = self.world_to_grid(x, y)
        if 0 <= grid_x < self.width and 0 <= grid_y < self.height:
            return self.grid[grid_y, grid_x] < 20  # Threshold for free space
        return False  # Out of bounds is not free

    def get_traversable_area(self) -> np.ndarray:
        """Get coordinates of traversable areas in the map"""
        free_mask = self.grid < 50
        y_coords, x_coords = np.where(free_mask)

        # Convert to world coordinates
        world_coords = []
        for y, x in zip(y_coords, x_coords):
            world_x, world_y = self.grid_to_world(x, y)
            world_coords.append([world_x, world_y])

        return np.array(world_coords)
```

<h3 className="third-heading">
- Localization System
</h3>
<div className="underline-class"></div>

Accurate localization is crucial for safe navigation, especially in dynamic environments:

```python
# Example: Advanced localization system
import particle_filter
from scipy.stats import norm
import random

class LocalizationSystem:
    def __init__(self, map_resolution: float = 0.05, num_particles: int = 1000):
        self.map_resolution = map_resolution
        self.num_particles = num_particles

        # Particle filter for Monte Carlo localization
        self.particle_filter = self.initialize_particle_filter()

        # Motion model parameters
        self.motion_model = {
            'alpha1': 0.1,  # Rotation noise from rotation
            'alpha2': 0.1,  # Rotation noise from translation
            'alpha3': 0.1,  # Translation noise from translation
            'alpha4': 0.1   # Translation noise from rotation
        }

        # Sensor model parameters
        self.sensor_model = {
            'sigma_hit': 0.2,    # Standard deviation for hit
            'lambda_short': 0.1, # Exponential decay rate for short readings
            'z_hit': 0.8,        # Probability of correct measurement
            'z_short': 0.1,      # Probability of short measurement
            'z_max': 0.05,       # Probability of max range
            'z_rand': 0.05       # Probability of random measurement
        }

    def initialize_particle_filter(self):
        """Initialize particle filter for localization"""
        # Create initial particles uniformly distributed
        particles = []
        for _ in range(self.num_particles):
            # Initialize with random pose (in practice, use odometry estimate)
            x = random.uniform(-10, 10)  # meters
            y = random.uniform(-10, 10)  # meters
            theta = random.uniform(-np.pi, np.pi)  # radians

            weight = 1.0 / self.num_particles
            particle = {
                'pose': np.array([x, y, theta]),
                'weight': weight
            }
            particles.append(particle)

        return particles

    def update_pose_estimate(self, odometry_delta: np.ndarray,
                           sensor_data: Dict[str, Any],
                           map_data: OccupancyGridMap) -> Dict[str, Any]:
        """Update pose estimate using particle filter"""
        # Prediction step: update particles based on odometry
        self.predict_particles(odometry_delta)

        # Update step: weight particles based on sensor data
        self.update_particle_weights(sensor_data, map_data)

        # Resample particles
        self.resample_particles()

        # Calculate final estimate
        pose_estimate = self.calculate_pose_estimate()

        return {
            'pose': pose_estimate,
            'uncertainty': self.calculate_uncertainty(),
            'particles': self.particle_filter
        }

    def predict_particles(self, odometry_delta: np.ndarray):
        """Predict particle poses based on odometry"""
        dx, dy, dtheta = odometry_delta

        for particle in self.particle_filter:
            # Add noise based on motion model
            noise_dx = np.random.normal(0, np.sqrt(
                self.motion_model['alpha1'] * dtheta**2 +
                self.motion_model['alpha3'] * dx**2
            ))

            noise_dy = np.random.normal(0, np.sqrt(
                self.motion_model['alpha2'] * dtheta**2 +
                self.motion_model['alpha4'] * dy**2
            ))

            noise_dtheta = np.random.normal(0, np.sqrt(
                self.motion_model['alpha1'] * dtheta**2
            ))

            # Update particle pose
            particle['pose'][0] += dx + noise_dx
            particle['pose'][1] += dy + noise_dy
            particle['pose'][2] += dtheta + noise_dtheta

    def update_particle_weights(self, sensor_data: Dict[str, Any],
                              map_data: OccupancyGridMap):
        """Update particle weights based on sensor measurements"""
        for particle in self.particle_filter:
            weight = self.calculate_sensor_likelihood(particle['pose'], sensor_data, map_data)
            particle['weight'] *= weight

    def calculate_sensor_likelihood(self, pose: np.ndarray,
                                  sensor_data: Dict[str, Any],
                                  map_data: OccupancyGridMap) -> float:
        """Calculate likelihood of sensor data given particle pose"""
        x, y, theta = pose
        ranges = sensor_data.get('ranges', [])
        angles = sensor_data.get('angles', [])

        total_likelihood = 1.0

        for range_val, angle in zip(ranges, angles):
            if range_val < 0.1 or range_val > 10.0:  # Invalid range
                continue

            # Calculate expected range to obstacle
            expected_range = self.ray_cast(x, y, theta + angle, map_data)

            # Calculate likelihood using sensor model
            likelihood = self.beam_range_finder_model(range_val, expected_range)
            total_likelihood *= likelihood

        return total_likelihood

    def ray_cast(self, x: float, y: float, angle: float,
                 map_data: OccupancyGridMap) -> float:
        """Ray cast to find expected range to obstacle"""
        # Simple ray casting implementation
        step_size = 0.05  # 5cm steps
        max_range = 10.0  # 10m max

        for dist in np.arange(0, max_range, step_size):
            test_x = x + dist * np.cos(angle)
            test_y = y + dist * np.sin(angle)

            if map_data.is_occupied(test_x, test_y):
                return dist

        return max_range  # No obstacle found

    def beam_range_finder_model(self, z: float, z_expected: float) -> float:
        """Beam range finder sensor model"""
        # Hit model: Gaussian around expected range
        p_hit = norm.pdf(z, z_expected, self.sensor_model['sigma_hit'])

        # Short model: Exponential decay for short readings
        if z < z_expected:
            p_short = self.sensor_model['lambda_short'] * np.exp(-self.sensor_model['lambda_short'] * z)
        else:
            p_short = 0

        # Max range model: spike at max range
        p_max = 1.0 if abs(z - 10.0) < 0.1 else 0.0  # Assuming max range is 10m

        # Random model: uniform distribution
        p_rand = 1.0 / 10.0  # Uniform over 0-10m range

        # Combine models
        p = (self.sensor_model['z_hit'] * p_hit +
             self.sensor_model['z_short'] * p_short +
             self.sensor_model['z_max'] * p_max +
             self.sensor_model['z_rand'] * p_rand)

        return p

    def resample_particles(self):
        """Resample particles based on their weights"""
        # Normalize weights
        weights = np.array([p['weight'] for p in self.particle_filter])
        weights_sum = np.sum(weights)

        if weights_sum == 0:
            # Reset particles if all weights are zero
            self.initialize_particle_filter()
            return

        weights /= weights_sum

        # Resample using systematic resampling
        new_particles = []
        step = 1.0 / self.num_particles
        start = np.random.uniform(0, step)

        cumulative_weight = 0.0
        current_particle_idx = 0

        for i in range(self.num_particles):
            threshold = start + i * step

            while cumulative_weight < threshold and current_particle_idx < len(self.particle_filter):
                cumulative_weight += weights[current_particle_idx]
                current_particle_idx += 1

            if current_particle_idx > 0:
                # Copy particle
                original_particle = self.particle_filter[current_particle_idx - 1]
                new_particle = {
                    'pose': original_particle['pose'].copy(),
                    'weight': 1.0 / self.num_particles
                }
                new_particles.append(new_particle)

        # Fill remaining particles if needed
        while len(new_particles) < self.num_particles:
            original_particle = random.choice(self.particle_filter)
            new_particle = {
                'pose': original_particle['pose'].copy(),
                'weight': 1.0 / self.num_particles
            }
            new_particles.append(new_particle)

        self.particle_filter = new_particles

    def calculate_pose_estimate(self) -> np.ndarray:
        """Calculate final pose estimate from particles"""
        # Calculate weighted mean of particles
        x_sum = y_sum = theta_sum = total_weight = 0

        for particle in self.particle_filter:
            weight = particle['weight']
            x, y, theta = particle['pose']

            x_sum += weight * x
            y_sum += weight * y
            theta_sum += weight * theta
            total_weight += weight

        if total_weight > 0:
            mean_x = x_sum / total_weight
            mean_y = y_sum / total_weight
            mean_theta = theta_sum / total_weight
        else:
            mean_x = mean_y = mean_theta = 0

        return np.array([mean_x, mean_y, mean_theta])

    def calculate_uncertainty(self) -> float:
        """Calculate uncertainty of pose estimate"""
        if not self.particle_filter:
            return float('inf')

        # Calculate variance of particles
        mean_pose = self.calculate_pose_estimate()
        variance_sum = 0

        for particle in self.particle_filter:
            diff = particle['pose'] - mean_pose
            # Handle angle wrap-around for theta
            diff[2] = (diff[2] + np.pi) % (2 * np.pi) - np.pi
            variance_sum += particle['weight'] * np.dot(diff, diff)

        return np.sqrt(variance_sum)
```

<h2 className="second-heading">
Path Planning and Navigation
</h2>
<div className="underline-class"></div>

<h3 className="third-heading">
- Global Path Planning
</h3>
<div className="underline-class"></div>

The global planner creates high-level paths from start to goal while considering the environment:

```python
# Example: Global path planning system
import heapq
from typing import Tuple, List, Dict, Optional

class GlobalPathPlanner:
    def __init__(self, map_resolution: float = 0.05, inflation_radius: float = 0.3):
        self.map_resolution = map_resolution
        self.inflation_radius = inflation_radius
        self.occupancy_map = None

    def set_map(self, occupancy_map: OccupancyGridMap):
        """Set the occupancy map for planning"""
        self.occupancy_map = occupancy_map
        # Inflate obstacles in the map
        self.inflated_map = self.inflate_obstacles(occupancy_map)

    def inflate_obstacles(self, original_map: OccupancyGridMap) -> OccupancyGridMap:
        """Inflate obstacles to account for robot size"""
        inflated = OccupancyGridMap(
            resolution=original_map.resolution,
            width=original_map.width,
            height=original_map.height
        )

        # Copy original grid
        inflated.grid = original_map.grid.copy()

        # Calculate inflation radius in grid cells
        inflation_cells = int(self.inflation_radius / original_map.resolution)

        # For each occupied cell, inflate its neighbors
        occupied_coords = np.where(original_map.grid > 50)
        for y, x in zip(occupied_coords[0], occupied_coords[1]):
            for dy in range(-inflation_cells, inflation_cells + 1):
                for dx in range(-inflation_cells, inflation_cells + 1):
                    new_x, new_y = x + dx, y + dy
                    if (0 <= new_x < original_map.width and
                        0 <= new_y < original_map.height):
                        dist = np.sqrt(dx**2 + dy**2) * original_map.resolution
                        if dist <= self.inflation_radius:
                            inflated.grid[new_y, new_x] = 100  # Mark as occupied

        return inflated

    def plan_path(self, start: Tuple[float, float],
                 goal: Tuple[float, float]) -> Optional[List[Tuple[float, float]]]:
        """Plan path using A* algorithm"""
        if not self.occupancy_map:
            return None

        # Convert world coordinates to grid coordinates
        start_grid = self.occupancy_map.world_to_grid(start[0], start[1])
        goal_grid = self.occupancy_map.world_to_grid(goal[0], goal[1])

        # Check if start and goal are valid
        if (self.inflated_map.is_occupied(*start) or
            self.inflated_map.is_occupied(*goal)):
            return None

        # Run A* pathfinding
        path_grid = self.a_star(start_grid, goal_grid)

        if path_grid is None:
            return None

        # Convert grid path back to world coordinates
        world_path = []
        for grid_x, grid_y in path_grid:
            world_x, world_y = self.occupancy_map.grid_to_world(grid_x, grid_y)
            world_path.append((world_x, world_y))

        return world_path

    def a_star(self, start: Tuple[int, int],
              goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
        """A* pathfinding algorithm"""
        # Heuristic function (Manhattan distance)
        def heuristic(a, b):
            return abs(a[0] - b[0]) + abs(a[1] - b[1])

        # Priority queue: (f_score, g_score, position)
        open_set = [(0, 0, start)]
        came_from = {}
        g_score = {start: 0}
        f_score = {start: heuristic(start, goal)}

        # Set of positions in open set
        open_set_hash = {start}

        while open_set:
            current = heapq.heappop(open_set)[2]
            open_set_hash.remove(current)

            if current == goal:
                # Reconstruct path
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path

            # Check 8-connected neighbors
            for dx in [-1, 0, 1]:
                for dy in [-1, 0, 1]:
                    if dx == 0 and dy == 0:
                        continue  # Skip current position

                    neighbor = (current[0] + dx, current[1] + dy)

                    # Check bounds
                    if (neighbor[0] < 0 or neighbor[0] >= self.inflated_map.width or
                        neighbor[1] < 0 or neighbor[1] >= self.inflated_map.height):
                        continue

                    # Check if neighbor is occupied
                    if self.inflated_map.grid[neighbor[1], neighbor[0]] > 50:
                        continue

                    # Calculate tentative g_score
                    movement_cost = np.sqrt(dx**2 + dy**2)  # Euclidean distance
                    tentative_g_score = g_score[current] + movement_cost

                    if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                        # This path to neighbor is better
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g_score
                        f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)

                        if neighbor not in open_set_hash:
                            heapq.heappush(open_set, (f_score[neighbor], g_score[neighbor], neighbor))
                            open_set_hash.add(neighbor)

        return None  # No path found

    def smooth_path(self, path: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """Smooth the planned path using path optimization"""
        if len(path) < 3:
            return path

        # Simple smoothing by removing unnecessary waypoints
        smoothed_path = [path[0]]

        i = 0
        while i < len(path) - 1:
            # Try to connect current point to future points directly
            j = len(path) - 1
            while j > i + 1:
                # Check if we can go directly from path[i] to path[j]
                if self.is_line_clear(path[i], path[j]):
                    smoothed_path.append(path[j])
                    i = j
                    break
                j -= 1
            else:
                # If no direct connection found, add next point
                i += 1
                if i < len(path):
                    smoothed_path.append(path[i])

        return smoothed_path

    def is_line_clear(self, start: Tuple[float, float],
                     end: Tuple[float, float]) -> bool:
        """Check if line between two points is clear of obstacles"""
        # Bresenham-like line algorithm to check for obstacles
        x0, y0 = start
        x1, y1 = end

        grid_start = self.occupancy_map.world_to_grid(x0, y0)
        grid_end = self.occupancy_map.world_to_grid(x1, y1)

        # Use Bresenham's algorithm to check line
        dx = abs(grid_end[0] - grid_start[0])
        dy = abs(grid_end[1] - grid_start[1])
        x_step = 1 if grid_end[0] > grid_start[0] else -1
        y_step = 1 if grid_end[1] > grid_start[1] else -1

        error = dx - dy
        x, y = grid_start[0], grid_start[1]

        while x != grid_end[0] or y != grid_end[1]:
            if 0 <= x < self.inflated_map.width and 0 <= y < self.inflated_map.height:
                if self.inflated_map.grid[y, x] > 50:  # Occupied
                    return False

            # Move to next cell
            error2 = 2 * error
            if error2 > -dy:
                error -= dy
                x += x_step
            if error2 < dx:
                error += dx
                y += y_step

        return True

class HumanoidPathPlanner(GlobalPathPlanner):
    """Path planner optimized for humanoid robot kinematics"""
    def __init__(self, map_resolution: float = 0.05, inflation_radius: float = 0.5):
        super().__init__(map_resolution, inflation_radius)

        # Humanoid-specific parameters
        self.step_size = 0.3  # Maximum step size for humanoid
        self.turn_radius = 0.2  # Minimum turning radius
        self.step_height = 0.1  # Maximum step height (for stairs/obstacles)

    def plan_path(self, start: Tuple[float, float],
                 goal: Tuple[float, float]) -> Optional[List[Tuple[float, float]]]:
        """Plan path considering humanoid kinematics"""
        # Use base class planning
        raw_path = super().plan_path(start, goal)

        if raw_path is None:
            return None

        # Adapt path for humanoid kinematics
        humanoid_path = self.adapt_path_for_humanoid(raw_path)

        return humanoid_path

    def adapt_path_for_humanoid(self, path: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """Adapt path for humanoid robot capabilities"""
        if len(path) < 2:
            return path

        # Ensure path points are reachable with humanoid step size
        adapted_path = [path[0]]

        for i in range(1, len(path)):
            current_point = path[i]
            prev_point = adapted_path[-1]

            # Calculate distance to next point
            dist = np.sqrt((current_point[0] - prev_point[0])**2 +
                          (current_point[1] - prev_point[1])**2)

            if dist > self.step_size:
                # Need to add intermediate points
                num_steps = int(np.ceil(dist / self.step_size))
                for j in range(1, num_steps):
                    ratio = j / num_steps
                    intermediate_x = prev_point[0] + ratio * (current_point[0] - prev_point[0])
                    intermediate_y = prev_point[1] + ratio * (current_point[1] - prev_point[1])
                    adapted_path.append((intermediate_x, intermediate_y))

            adapted_path.append(current_point)

        return adapted_path
```

<h3 className="third-heading">
- Local Path Planning and Obstacle Avoidance
</h3>
<div className="underline-class"></div>

The local planner handles dynamic obstacle avoidance and path following:

```python
# Example: Local path planning and obstacle avoidance
class LocalPathPlanner:
    def __init__(self, horizon: float = 2.0, resolution: float = 0.1):
        self.horizon = horizon  # Planning horizon in meters
        self.resolution = resolution  # Grid resolution in meters
        self.lookahead_distance = 0.5  # Distance to look ahead for path following

        # Velocity constraints for humanoid robot
        self.max_linear_vel = 0.5  # m/s
        self.max_angular_vel = 0.5  # rad/s
        self.min_linear_vel = 0.1   # m/s (minimum to maintain balance)

    def plan_local_path(self, current_pose: np.ndarray,
                       global_path: List[Tuple[float, float]],
                       sensor_data: Dict[str, Any]) -> Dict[str, Any]:
        """Plan local trajectory considering current obstacles"""
        # Find current position on global path
        current_pos = (current_pose[0, 3], current_pose[1, 3])
        current_theta = np.arctan2(current_pose[1, 0], current_pose[0, 0])

        # Get path segment within horizon
        path_segment = self.get_path_segment(current_pos, global_path)

        # Check for obstacles in sensor data
        obstacles = self.extract_obstacles_from_sensor(sensor_data)

        if obstacles:
            # Plan around obstacles using Dynamic Window Approach (DWA)
            velocity_cmd = self.dwa_plan(current_pos, current_theta, path_segment, obstacles)
        else:
            # Follow path using pure pursuit
            velocity_cmd = self.pure_pursuit_plan(current_pos, current_theta, path_segment)

        return {
            'linear_velocity': velocity_cmd[0],
            'angular_velocity': velocity_cmd[1],
            'path_segment': path_segment,
            'obstacles': obstacles
        }

    def get_path_segment(self, current_pos: Tuple[float, float],
                        global_path: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """Get path segment within planning horizon"""
        if not global_path:
            return []

        # Find closest point on path
        closest_idx = self.find_closest_point(current_pos, global_path)

        # Get points within horizon
        segment = []
        for i in range(closest_idx, len(global_path)):
            point = global_path[i]
            dist = np.sqrt((point[0] - current_pos[0])**2 + (point[1] - current_pos[1])**2)

            if dist > self.horizon:
                break

            segment.append(point)

        return segment if segment else [global_path[-1]]  # At least return goal

    def find_closest_point(self, pos: Tuple[float, float],
                          path: List[Tuple[float, float]]) -> int:
        """Find index of closest point on path to current position"""
        min_dist = float('inf')
        closest_idx = 0

        for i, point in enumerate(path):
            dist = np.sqrt((point[0] - pos[0])**2 + (point[1] - pos[1])**2)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i

        return closest_idx

    def extract_obstacles_from_sensor(self, sensor_data: Dict[str, Any]) -> List[Tuple[float, float]]:
        """Extract obstacle positions from sensor data"""
        obstacles = []

        if 'ranges' in sensor_data and 'angles' in sensor_data:
            ranges = sensor_data['ranges']
            angles = sensor_data['angles']

            for range_val, angle in zip(ranges, angles):
                if 0.1 < range_val < 3.0:  # Valid range for obstacles
                    # Convert polar to Cartesian (robot frame)
                    x = range_val * np.cos(angle)
                    y = range_val * np.sin(angle)
                    obstacles.append((x, y))

        return obstacles

    def dwa_plan(self, current_pos: Tuple[float, float],
                current_theta: float,
                path_segment: List[Tuple[float, float]],
                obstacles: List[Tuple[float, float]]) -> Tuple[float, float]:
        """Dynamic Window Approach for local planning"""
        # Define velocity space
        v_min = max(-self.max_linear_vel, self.min_linear_vel)
        v_max = self.max_linear_vel
        w_min = -self.max_angular_vel
        w_max = self.max_angular_vel

        # Define sampling resolution
        dv = 0.1
        dw = 0.1

        best_score = float('-inf')
        best_vel = (0.0, 0.0)

        # Sample velocity space
        for v in np.arange(v_min, v_max + dv, dv):
            for w in np.arange(w_min, w_max + dw, dw):
                # Simulate trajectory
                future_pos = self.simulate_trajectory(current_pos, current_theta, v, w)

                # Calculate scores
                heading_score = self.calculate_heading_score(future_pos, current_theta, path_segment)
                dist_score = self.calculate_obstacle_score(future_pos, obstacles)
                vel_score = abs(v)  # Prefer higher velocities

                # Weighted combination
                score = 0.3 * heading_score + 0.5 * dist_score + 0.2 * vel_score

                if score > best_score:
                    best_score = score
                    best_vel = (v, w)

        return best_vel

    def simulate_trajectory(self, pos: Tuple[float, float], theta: float,
                          v: float, w: float, dt: float = 0.1, steps: int = 10) -> Tuple[float, float]:
        """Simulate robot trajectory with given velocities"""
        x, y = pos

        for _ in range(steps):
            x += v * np.cos(theta) * dt
            y += v * np.sin(theta) * dt
            theta += w * dt

        return (x, y)

    def calculate_heading_score(self, pos: Tuple[float, float],
                              theta: float,
                              path_segment: List[Tuple[float, float]]) -> float:
        """Calculate score based on alignment with path"""
        if not path_segment:
            return 0.0

        # Find next path point
        next_point = path_segment[0]

        # Calculate desired heading
        desired_theta = np.arctan2(next_point[1] - pos[1], next_point[0] - pos[0])

        # Calculate heading difference
        heading_diff = abs((desired_theta - theta + np.pi) % (2 * np.pi) - np.pi)

        # Score: 1.0 for perfect alignment, 0.0 for 180-degree difference
        return max(0.0, 1.0 - heading_diff / np.pi)

    def calculate_obstacle_score(self, pos: Tuple[float, float],
                               obstacles: List[Tuple[float, float]]) -> float:
        """Calculate score based on obstacle proximity"""
        if not obstacles:
            return 1.0  # No obstacles is best

        min_dist = float('inf')
        for obs_x, obs_y in obstacles:
            dist = np.sqrt((obs_x - pos[0])**2 + (obs_y - pos[1])**2)
            min_dist = min(min_dist, dist)

        # Score: 1.0 for far obstacles, 0.0 for very close obstacles
        return min(1.0, min_dist / 1.0)  # Normalize by 1m threshold

    def pure_pursuit_plan(self, current_pos: Tuple[float, float],
                         current_theta: float,
                         path_segment: List[Tuple[float, float]]) -> Tuple[float, float]:
        """Pure pursuit path following"""
        if not path_segment:
            return (0.0, 0.0)

        # Find look-ahead point
        look_ahead_point = self.find_look_ahead_point(current_pos, path_segment)

        if look_ahead_point is None:
            return (0.0, 0.0)

        # Calculate distance to look-ahead point
        dx = look_ahead_point[0] - current_pos[0]
        dy = look_ahead_point[1] - current_pos[1]
        dist_to_goal = np.sqrt(dx**2 + dy**2)

        # Calculate angle to goal
        angle_to_goal = np.arctan2(dy, dx)
        angle_diff = (angle_to_goal - current_theta + np.pi) % (2 * np.pi) - np.pi

        # Calculate angular velocity
        k = 1.0  # Proportional gain
        angular_vel = k * angle_diff

        # Limit angular velocity
        angular_vel = max(-self.max_angular_vel, min(self.max_angular_vel, angular_vel))

        # Calculate linear velocity (reduce when turning)
        linear_vel = self.max_linear_vel * max(0.1, 1.0 - abs(angular_vel) / self.max_angular_vel)

        return (linear_vel, angular_vel)

    def find_look_ahead_point(self, current_pos: Tuple[float, float],
                            path_segment: List[Tuple[float, float]]) -> Optional[Tuple[float, float]]:
        """Find look-ahead point on path"""
        look_ahead_dist = self.lookahead_distance

        for i in range(len(path_segment) - 1):
            p1 = path_segment[i]
            p2 = path_segment[i + 1]

            # Check if look-ahead distance point is on this segment
            segment_vec = (p2[0] - p1[0], p2[1] - p1[1])
            segment_len = np.sqrt(segment_vec[0]**2 + segment_vec[1]**2)

            if segment_len == 0:
                continue

            # Normalize segment vector
            segment_unit = (segment_vec[0] / segment_len, segment_vec[1] / segment_len)

            # Project current position onto segment
            to_p1 = (current_pos[0] - p1[0], current_pos[1] - p1[1])
            projection = to_p1[0] * segment_unit[0] + to_p1[1] * segment_unit[1]

            # Check if projection is within segment
            if 0 <= projection <= segment_len:
                closest_on_segment = (
                    p1[0] + projection * segment_unit[0],
                    p1[1] + projection * segment_unit[1]
                )

                # Calculate distance from current position to closest point
                dist_to_segment = np.sqrt(
                    (current_pos[0] - closest_on_segment[0])**2 +
                    (current_pos[1] - closest_on_segment[1])**2
                )

                # If we're close enough to the path, look ahead
                if dist_to_segment <= look_ahead_dist:
                    # Look ahead along the path
                    look_ahead_param = min(projection + look_ahead_dist, segment_len)
                    look_ahead_point = (
                        p1[0] + look_ahead_param * segment_unit[0],
                        p1[1] + look_ahead_param * segment_unit[1]
                    )
                    return look_ahead_point

        # If no point found on current segments, return last point
        return path_segment[-1] if path_segment else None
```

<h2 className="second-heading">
Isaac Integration for Navigation
</h2>
<div className="underline-class"></div>

<h3 className="third-heading">
- Isaac Navigation Components
</h3>
<div className="underline-class"></div>

The Isaac ecosystem provides specialized components for navigation that leverage NVIDIA's hardware acceleration:

```python
# Example: Isaac integration for navigation
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
import tf2_ros
from tf2_ros import TransformException
import numpy as np
import json

class IsaacNavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/move_base_simple/goal', 10)
        self.path_pub = self.create_publisher(OccupancyGrid, '/global_costmap/costmap', 10)

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10
        )

        # Initialize navigation components
        self.localization_system = LocalizationSystem()
        self.global_planner = HumanoidPathPlanner()
        self.local_planner = LocalPathPlanner()
        self.slam_system = SLAMSystem()

        # Navigation state
        self.current_pose = np.eye(4)
        self.current_goal = None
        self.global_path = []
        self.navigation_active = False

        # TF buffer for coordinate transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info('Isaac Navigation node initialized')

    def odom_callback(self, msg: Odometry):
        """Handle odometry messages for pose estimation"""
        try:
            # Extract pose from odometry message
            pose = np.eye(4)
            pose[0, 3] = msg.pose.pose.position.x
            pose[1, 3] = msg.pose.pose.position.y
            pose[2, 3] = msg.pose.pose.position.z

            # Convert quaternion to rotation matrix
            quat = msg.pose.pose.orientation
            rotation = R.from_quat([quat.x, quat.y, quat.z, quat.w])
            pose[:3, :3] = rotation.as_matrix()

            self.current_pose = pose

            # Update SLAM system if available
            if self.slam_active:
                # This would require point cloud data from sensors
                pass

        except Exception as e:
            self.get_logger().error(f'Error processing odometry: {e}')

    def scan_callback(self, msg: LaserScan):
        """Handle laser scan messages for obstacle detection and localization"""
        try:
            # Convert laser scan to ranges and angles
            ranges = list(msg.ranges)
            angles = [msg.angle_min + i * msg.angle_increment
                     for i in range(len(ranges))]

            # Update localization system
            sensor_data = {
                'ranges': ranges,
                'angles': angles,
                'intensities': list(msg.intensities) if msg.intensities else []
            }

            # Update pose estimate
            if hasattr(self, 'occupancy_map'):
                localization_result = self.localization_system.update_pose_estimate(
                    self.get_odometry_delta(),
                    sensor_data,
                    self.occupancy_map
                )

                # Update current pose with refined estimate
                est_pose = localization_result['pose']
                self.current_pose[0, 3] = est_pose[0]
                self.current_pose[1, 3] = est_pose[1]

                # Extract 2D pose for navigation
                pose_2d = np.eye(4)
                pose_2d[0, 3] = est_pose[0]
                pose_2d[1, 3] = est_pose[1]
                pose_2d[2, 2] = np.cos(est_pose[2])
                pose_2d[2, 0] = -np.sin(est_pose[2])
                pose_2d[0, 2] = np.sin(est_pose[2])
                pose_2d[1, 2] = -np.cos(est_pose[2])

                self.current_pose = pose_2d

            # If navigation is active, update local planning
            if self.navigation_active and self.current_goal:
                self.execute_navigation_step(sensor_data)

        except Exception as e:
            self.get_logger().error(f'Error processing scan: {e}')

    def imu_callback(self, msg: Imu):
        """Handle IMU messages for balance and orientation"""
        # Use IMU data for humanoid balance control
        # This would interface with the robot's balance controller
        pass

    def set_navigation_goal(self, goal_x: float, goal_y: float, goal_theta: float = 0.0):
        """Set navigation goal and start planning"""
        goal_pose = PoseStamped()
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position.x = goal_x
        goal_pose.pose.position.y = goal_y
        goal_pose.pose.position.z = 0.0

        # Convert angle to quaternion
        from scipy.spatial.transform import Rotation as R
        quat = R.from_euler('z', goal_theta).as_quat()
        goal_pose.pose.orientation.x = quat[0]
        goal_pose.pose.orientation.y = quat[1]
        goal_pose.pose.orientation.z = quat[2]
        goal_pose.pose.orientation.w = quat[3]

        # Store goal internally
        self.current_goal = (goal_x, goal_y)

        # Plan global path
        current_pos = (self.current_pose[0, 3], self.current_pose[1, 3])
        self.global_path = self.global_planner.plan_path(current_pos, self.current_goal)

        if self.global_path:
            self.navigation_active = True
            self.get_logger().info(f'Navigation started to goal: ({goal_x}, {goal_y})')
            return True
        else:
            self.get_logger().error('Could not plan path to goal')
            return False

    def execute_navigation_step(self, sensor_data: Dict[str, Any]):
        """Execute one step of navigation"""
        if not self.navigation_active or not self.global_path:
            return

        # Get local plan based on global path and sensor data
        local_plan = self.local_planner.plan_local_path(
            self.current_pose,
            self.global_path,
            sensor_data
        )

        # Send velocity command
        cmd_msg = Twist()
        cmd_msg.linear.x = local_plan['linear_velocity']
        cmd_msg.angular.z = local_plan['angular_velocity']

        self.cmd_vel_pub.publish(cmd_msg)

        # Check if goal reached
        current_pos = (self.current_pose[0, 3], self.current_pose[1, 3])
        goal_pos = self.current_goal
        dist_to_goal = np.sqrt((current_pos[0] - goal_pos[0])**2 +
                              (current_pos[1] - goal_pos[1])**2)

        if dist_to_goal < 0.3:  # 30cm tolerance
            self.navigation_active = False
            self.get_logger().info('Goal reached successfully')
            self.publish_navigation_result('success')

    def get_odometry_delta(self) -> np.ndarray:
        """Get odometry delta for localization prediction"""
        # This would track the change since last update
        # For now, return a small default delta
        return np.array([0.01, 0.01, 0.01])  # dx, dy, dtheta

    def publish_navigation_result(self, result: str):
        """Publish navigation result"""
        result_msg = String()
        result_msg.data = json.dumps({
            'result': result,
            'goal': self.current_goal,
            'final_pose': [self.current_pose[0, 3], self.current_pose[1, 3]],
            'timestamp': time.time()
        })

        # Publish to navigation result topic
        # This would be a custom message in practice
        pass

class IsaacSLAMNode(Node):
    """Isaac node for SLAM functionality"""
    def __init__(self):
        super().__init__('slam_node')

        # Publishers
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        self.odom_pub = self.create_publisher(Occupy, '/slam_odom', 10)

        # Subscribers
        self.pointcloud_sub = self.create_subscription(
            PointCloud2, '/camera/depth/points', self.pointcloud_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10
        )

        # Initialize SLAM system
        self.slam_system = SLAMSystem()
        self.occupancy_grid = OccupancyGridMap()

        self.get_logger().info('Isaac SLAM node initialized')

    def pointcloud_callback(self, msg: PointCloud2):
        """Process point cloud data for SLAM"""
        try:
            # Convert PointCloud2 to numpy array
            # This requires point_cloud2 library
            import sensor_msgs.point_cloud2 as pc2
            points = []
            for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                points.append([point[0], point[1], point[2]])

            points_array = np.array(points)

            # Process frame with SLAM system
            slam_result = self.slam_system.process_frame(points_array)

            if slam_result['map_updated']:
                # Update occupancy grid based on SLAM results
                self.update_occupancy_grid(slam_result['pose'])

                # Publish updated map
                self.publish_map()

        except Exception as e:
            self.get_logger().error(f'Error processing point cloud: {e}')

    def update_occupancy_grid(self, pose: np.ndarray):
        """Update occupancy grid based on SLAM results"""
        # This would integrate SLAM map into occupancy grid
        # For this example, we'll use a simplified approach
        pass

    def publish_map(self):
        """Publish the current map"""
        # Convert internal map representation to OccupancyGrid message
        # This would involve more complex conversion in practice
        pass

    def imu_callback(self, msg: Imu):
        """Handle IMU data for pose estimation"""
        # Use IMU for better pose estimation in SLAM
        pass
```

<h3 className="third-heading">
- Navigation Safety and Validation
</h3>
<div className="underline-class"></div>

Safety is paramount in navigation systems, especially for humanoid robots operating near humans:

```python
# Example: Navigation safety and validation system
class NavigationSafetySystem:
    def __init__(self):
        self.safety_thresholds = {
            'min_obstacle_distance': 0.5,  # meters
            'max_linear_velocity': 0.5,    # m/s
            'max_angular_velocity': 0.5,   # rad/s
            'max_acceleration': 1.0,       # m/s^2
            'max_angular_acceleration': 1.0 # rad/s^2
        }

        self.emergency_stop = False
        self.safety_lock = threading.RLock()
        self.safety_log = []

        # Human detection and tracking
        self.human_tracking_enabled = True
        self.human_proximity_threshold = 1.0  # meters

    def validate_navigation_command(self, cmd_vel: Twist,
                                  sensor_data: Dict[str, Any]) -> Dict[str, Any]:
        """Validate navigation command for safety"""
        with self.safety_lock:
            if self.emergency_stop:
                return {
                    'safe': False,
                    'reason': 'Emergency stop active',
                    'recommended_action': 'Stop immediately'
                }

            # Check velocity limits
            vel_check = self.check_velocity_limits(cmd_vel)
            if not vel_check['safe']:
                return vel_check

            # Check obstacle proximity
            obs_check = self.check_obstacle_proximity(sensor_data)
            if not obs_check['safe']:
                return obs_check

            # Check human proximity if enabled
            if self.human_tracking_enabled:
                human_check = self.check_human_proximity(sensor_data)
                if not human_check['safe']:
                    return human_check

            return {
                'safe': True,
                'reason': 'All safety checks passed',
                'recommended_action': 'Proceed with navigation'
            }

    def check_velocity_limits(self, cmd_vel: Twist) -> Dict[str, Any]:
        """Check if velocity commands are within safe limits"""
        if abs(cmd_vel.linear.x) > self.safety_thresholds['max_linear_velocity']:
            return {
                'safe': False,
                'reason': f'Linear velocity {abs(cmd_vel.linear.x):.2f} exceeds limit {self.safety_thresholds["max_linear_velocity"]:.2f}',
                'recommended_action': f'Reduce linear velocity to <= {self.safety_thresholds["max_linear_velocity"]:.2f}'
            }

        if abs(cmd_vel.angular.z) > self.safety_thresholds['max_angular_velocity']:
            return {
                'safe': False,
                'reason': f'Angular velocity {abs(cmd_vel.angular.z):.2f} exceeds limit {self.safety_thresholds["max_angular_velocity"]:.2f}',
                'recommended_action': f'Reduce angular velocity to <= {self.safety_thresholds["max_angular_velocity"]:.2f}'
            }

        return {'safe': True}

    def check_obstacle_proximity(self, sensor_data: Dict[str, Any]) -> Dict[str, Any]:
        """Check for obstacles in sensor data"""
        if 'ranges' not in sensor_data:
            return {'safe': True}

        ranges = sensor_data['ranges']
        min_range = min(ranges) if ranges else float('inf')

        if min_range < self.safety_thresholds['min_obstacle_distance']:
            return {
                'safe': False,
                'reason': f'Obstacle detected at {min_range:.2f}m, minimum safe distance is {self.safety_thresholds["min_obstacle_distance"]:.2f}m',
                'recommended_action': 'Stop or replan path to avoid obstacle'
            }

        return {'safe': True}

    def check_human_proximity(self, sensor_data: Dict[str, Any]) -> Dict[str, Any]:
        """Check for humans in proximity using sensor data"""
        # This would use specialized human detection
        # For this example, we'll use a simplified approach
        if 'ranges' not in sensor_data:
            return {'safe': True}

        ranges = sensor_data['ranges']
        angles = sensor_data.get('angles', [0] * len(ranges))

        # Look for human-like signatures in scan data
        # This is a simplified detection based on range patterns
        for i, (r, a) in enumerate(zip(ranges, angles)):
            if r < self.human_proximity_threshold:
                # Check neighboring points for human-like width
                left_idx = max(0, i - 2)
                right_idx = min(len(ranges) - 1, i + 2)

                left_range = ranges[left_idx]
                right_range = ranges[right_idx]

                # If neighboring points are significantly farther, might be a person
                if (left_range > r * 1.5 and right_range > r * 1.5 and
                    abs(left_range - right_range) < 0.3):  # Human-like width
                    return {
                        'safe': False,
                        'reason': f'Human detected at {r:.2f}m, minimum safe distance is {self.human_proximity_threshold:.2f}m',
                        'recommended_action': 'Stop and wait for human to move away or request permission'
                    }

        return {'safe': True}

    def trigger_emergency_stop(self):
        """Trigger emergency stop for safety"""
        with self.safety_lock:
            self.emergency_stop = True
            self.safety_log.append({
                'timestamp': time.time(),
                'event': 'EMERGENCY_STOP',
                'reason': 'Safety system triggered'
            })

    def clear_emergency_stop(self):
        """Clear emergency stop condition"""
        with self.safety_lock:
            self.emergency_stop = False
            self.safety_log.append({
                'timestamp': time.time(),
                'event': 'EMERGENCY_STOP_CLEARED',
                'reason': 'Manual override'
            })

    def get_safety_status(self) -> Dict[str, Any]:
        """Get current safety status"""
        return {
            'emergency_stop': self.emergency_stop,
            'safety_thresholds': self.safety_thresholds,
            'recent_logs': self.safety_log[-10:],  # Last 10 events
            'human_detection_enabled': self.human_tracking_enabled
        }

class NavigationValidator:
    """System for validating navigation performance and safety"""
    def __init__(self):
        self.metrics = {
            'path_efficiency': [],
            'navigation_success': [],
            'safety_violations': [],
            'human_interactions': [],
            'collision_avoidance': []
        }

    def validate_navigation_execution(self, planned_path: List[Tuple[float, float]],
                                    executed_path: List[Tuple[float, float]],
                                    safety_logs: List[Dict[str, Any]]) -> Dict[str, Any]:
        """Validate navigation execution against plan"""
        # Calculate path efficiency (actual path length vs optimal)
        optimal_length = self.calculate_path_length(planned_path)
        executed_length = self.calculate_path_length(executed_path)

        path_efficiency = optimal_length / executed_length if executed_length > 0 else 0

        # Check for successful completion
        if executed_path and planned_path:
            start_error = self.calculate_position_error(
                planned_path[0], executed_path[0]
            )
            goal_error = self.calculate_position_error(
                planned_path[-1], executed_path[-1]
            )

            success = goal_error < 0.5  # 50cm tolerance
        else:
            start_error = goal_error = float('inf')
            success = False

        # Count safety violations
        safety_violations = len([log for log in safety_logs
                               if log['event'] == 'EMERGENCY_STOP'])

        result = {
            'path_efficiency': path_efficiency,
            'success': success,
            'start_error': start_error,
            'goal_error': goal_error,
            'safety_violations': safety_violations,
            'executed_length': executed_length,
            'optimal_length': optimal_length
        }

        # Store metrics
        self.metrics['path_efficiency'].append(path_efficiency)
        self.metrics['navigation_success'].append(success)
        self.metrics['safety_violations'].append(safety_violations)

        return result

    def calculate_path_length(self, path: List[Tuple[float, float]]) -> float:
        """Calculate total length of a path"""
        if len(path) < 2:
            return 0.0

        total_length = 0.0
        for i in range(1, len(path)):
            dx = path[i][0] - path[i-1][0]
            dy = path[i][1] - path[i-1][1]
            total_length += np.sqrt(dx*dx + dy*dy)

        return total_length

    def calculate_position_error(self, pos1: Tuple[float, float],
                               pos2: Tuple[float, float]) -> float:
        """Calculate Euclidean distance between two positions"""
        dx = pos1[0] - pos2[0]
        dy = pos1[1] - pos2[1]
        return np.sqrt(dx*dx + dy*dy)

    def get_validation_summary(self) -> Dict[str, Any]:
        """Get summary of validation metrics"""
        return {
            'avg_path_efficiency': np.mean(self.metrics['path_efficiency']) if self.metrics['path_efficiency'] else 0,
            'success_rate': np.mean(self.metrics['navigation_success']) if self.metrics['navigation_success'] else 0,
            'avg_safety_violations': np.mean(self.metrics['safety_violations']) if self.metrics['safety_violations'] else 0,
            'total_navigations': len(self.metrics['navigation_success'])
        }
```

<h2 className="second-heading">
Performance Optimization and Real-Time Considerations
</h2>
<div className="underline-class"></div>

<h3 className="third-heading">
- Real-Time Navigation Performance
</h3>
<div className="underline-class"></div>

Navigation systems must meet real-time constraints while maintaining accuracy and safety:

```python
# Example: Real-time navigation performance optimization
class RealTimeNavigationOptimizer:
    def __init__(self):
        self.timing_requirements = {
            'localization_update': 0.1,      # 10Hz
            'global_planning': 1.0,         # 1Hz
            'local_planning': 0.05,         # 20Hz
            'control_update': 0.01,         # 100Hz
            'safety_check': 0.02           # 50Hz
        }

        self.performance_monitor = PerformanceMonitor()
        self.adaptive_resolution = True
        self.multi_threading_enabled = True

    def optimize_localization_performance(self, sensor_data: Dict[str, Any]) -> Dict[str, Any]:
        """Optimize localization for real-time performance"""
        start_time = time.time()

        # Adaptive resolution based on computational load
        if self.performance_monitor.get_avg_time('localization') > 0.05:  # 50ms budget
            # Reduce resolution for faster processing
            sensor_data = self.downsample_sensor_data(sensor_data, factor=0.5)

        # Perform localization
        result = self.perform_localization_optimized(sensor_data)

        execution_time = time.time() - start_time
        self.performance_monitor.record_time('localization', execution_time)

        return result

    def downsample_sensor_data(self, sensor_data: Dict[str, Any], factor: float) -> Dict[str, Any]:
        """Downsample sensor data for faster processing"""
        if 'ranges' in sensor_data and 'angles' in sensor_data:
            ranges = sensor_data['ranges']
            angles = sensor_data['angles']

            # Keep every nth point based on factor
            step = max(1, int(1.0 / factor))
            downsampled_ranges = ranges[::step]
            downsampled_angles = angles[::step]

            sensor_data['ranges'] = downsampled_ranges
            sensor_data['angles'] = downsampled_angles

        return sensor_data

    def perform_localization_optimized(self, sensor_data: Dict[str, Any]) -> Dict[str, Any]:
        """Optimized localization using efficient algorithms"""
        # Use approximate nearest neighbor search for feature matching
        # Implement particle filter with reduced number of particles when possible
        # Use spatial indexing for faster map queries

        # Placeholder for optimized implementation
        return {
            'pose': np.array([0.0, 0.0, 0.0]),
            'uncertainty': 0.1
        }

    def optimize_path_planning_performance(self, start: Tuple[float, float],
                                         goal: Tuple[float, float],
                                         map_data: OccupancyGridMap) -> List[Tuple[float, float]]:
        """Optimize path planning for real-time performance"""
        start_time = time.time()

        # Use hierarchical planning: coarse-to-fine approach
        coarse_path = self.plan_coarse_path(start, goal, map_data)

        if coarse_path:
            # Refine path using local optimization
            refined_path = self.refine_path(coarse_path, map_data)
        else:
            refined_path = []

        execution_time = time.time() - start_time
        self.performance_monitor.record_time('path_planning', execution_time)

        return refined_path

    def plan_coarse_path(self, start: Tuple[float, float],
                        goal: Tuple[float, float],
                        map_data: OccupancyGridMap) -> List[Tuple[float, float]]:
        """Plan coarse path using lower resolution map"""
        # Create coarse map by downsampling
        coarse_map = self.create_coarse_map(map_data, factor=4)

        # Plan on coarse map
        coarse_start = self.downsample_point(start, 4)
        coarse_goal = self.downsample_point(goal, 4)

        coarse_planner = GlobalPathPlanner(
            map_resolution=map_data.resolution * 4,
            inflation_radius=0.5
        )
        coarse_planner.set_map(coarse_map)

        return coarse_planner.plan_path(coarse_start, coarse_goal)

    def create_coarse_map(self, fine_map: OccupancyGridMap, factor: int) -> OccupancyGridMap:
        """Create coarse map by downsampling fine map"""
        coarse_width = fine_map.width // factor
        coarse_height = fine_map.height // factor

        coarse_map = OccupancyGridMap(
            resolution=fine_map.resolution * factor,
            width=coarse_width,
            height=coarse_height
        )

        # Downsample using maximum pooling to preserve obstacles
        for y in range(coarse_height):
            for x in range(coarse_width):
                # Take maximum value in corresponding fine region
                fine_y_start = y * factor
                fine_x_start = x * factor

                max_val = -1
                for dy in range(factor):
                    for dx in range(factor):
                        fy = min(fine_y_start + dy, fine_map.height - 1)
                        fx = min(fine_x_start + dx, fine_map.width - 1)
                        max_val = max(max_val, fine_map.grid[fy, fx])

                coarse_map.grid[y, x] = max_val

        return coarse_map

    def downsample_point(self, point: Tuple[float, float], factor: int) -> Tuple[float, float]:
        """Downsample a world point for coarse map"""
        grid_x, grid_y = int(point[0] / (self.map_resolution * factor)), int(point[1] / (self.map_resolution * factor))
        world_x = grid_x * self.map_resolution * factor + self.map_resolution * factor / 2
        world_y = grid_y * self.map_resolution * factor + self.map_resolution * factor / 2
        return (world_x, world_y)

    def refine_path(self, coarse_path: List[Tuple[float, float]],
                   map_data: OccupancyGridMap) -> List[Tuple[float, float]]:
        """Refine coarse path to fine resolution"""
        if not coarse_path:
            return []

        refined_path = [coarse_path[0]]

        for i in range(1, len(coarse_path)):
            # Plan fine path between consecutive coarse points
            segment = self.plan_path_segment(coarse_path[i-1], coarse_path[i], map_data)
            if segment:
                # Add segment points (skip first to avoid duplication)
                refined_path.extend(segment[1:])
            else:
                # If segment planning fails, add the coarse point directly
                refined_path.append(coarse_path[i])

        return refined_path

    def plan_path_segment(self, start: Tuple[float, float],
                         end: Tuple[float, float],
                         map_data: OccupancyGridMap) -> List[Tuple[float, float]]:
        """Plan path between two points"""
        planner = GlobalPathPlanner(
            map_resolution=map_data.resolution,
            inflation_radius=0.3
        )
        planner.set_map(map_data)
        return planner.plan_path(start, end)

class PerformanceMonitor:
    """Monitor and optimize navigation system performance"""
    def __init__(self):
        self.component_times = {}
        self.frame_times = collections.deque(maxlen=100)
        self.cpu_usage = []
        self.memory_usage = []

    def record_time(self, component: str, execution_time: float):
        """Record execution time for a component"""
        if component not in self.component_times:
            self.component_times[component] = collections.deque(maxlen=50)

        self.component_times[component].append(execution_time)

    def get_avg_time(self, component: str) -> float:
        """Get average execution time for a component"""
        if component in self.component_times and self.component_times[component]:
            return np.mean(self.component_times[component])
        return float('inf')

    def is_component_overloaded(self, component: str, threshold: float = 0.8) -> bool:
        """Check if component is overloaded based on time budget"""
        avg_time = self.get_avg_time(component)
        budget = self.get_time_budget(component)
        return (avg_time / budget) > threshold if budget > 0 else False

    def get_time_budget(self, component: str) -> float:
        """Get expected time budget for component"""
        budgets = {
            'localization': 0.1,      # 100ms
            'path_planning': 1.0,     # 1000ms
            'local_planning': 0.05,   # 50ms
            'control': 0.01,          # 10ms
            'safety': 0.02           # 20ms
        }
        return budgets.get(component, 0.1)
```

<div className="border-line"></div>
---
<h2 className="second-heading">
 Summary
</h2>
<div className="underline-class"></div>

The autonomous navigation system for humanoid robots represents a sophisticated integration of perception, mapping, localization, and path planning technologies. The system must handle the unique challenges of humanoid locomotion, including balance constraints, step limitations, and the need to operate safely in human-populated environments.

Key components of the navigation system include:

1. • **SLAM and Mapping**: Simultaneous localization and mapping systems that build and maintain environmental maps while tracking the robot's position.

2. • **Localization**: Advanced localization systems using particle filters and sensor fusion to accurately determine the robot's position.

3. • **Path Planning**: Hierarchical planning systems that create global paths and adapt them for humanoid kinematics, with local planning for dynamic obstacle avoidance.

4. • **Isaac Integration**: Specialized components that leverage the Isaac ecosystem for enhanced navigation capabilities.

5. • **Safety Systems**: Comprehensive safety and validation systems that ensure safe operation around humans and obstacles.

6. • **Performance Optimization**: Real-time optimization techniques that ensure navigation systems meet timing constraints while maintaining accuracy.

The success of the navigation system depends on careful integration of these components, with particular attention to the unique requirements of humanoid robots, including their complex kinematics, balance requirements, and the need for safe human interaction.

<div className="border-line"></div>
---
<h2 className="second-heading">
 Exercises
</h2>
<div className="underline-class"></div>

1. • Implement a SLAM system using point cloud data from RGB-D sensors
2. • Create a path planner that considers humanoid step constraints
3. • Design a local planner that handles dynamic obstacle avoidance
4. • Build a safety validation system for navigation performance
5. • Optimize navigation algorithms for real-time performance on your robot platform

<div className="border-line"></div>
---
<h2 className="second-heading">
 Further Reading
</h2>
<div className="underline-class"></div>

- • "Probabilistic Robotics" by Thrun, Burgard, and Fox
- • "Springer Handbook of Robotics" by Siciliano and Khatib
- • "Robotics, Vision and Control" by Corke
- • "Planning Algorithms" by LaValle
- • NVIDIA Isaac documentation on navigation

</div>