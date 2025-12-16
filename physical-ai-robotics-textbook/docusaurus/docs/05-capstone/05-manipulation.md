---
sidebar_position: 5
title: "Robotic Manipulation System"
description: "Implementing dexterous manipulation capabilities for humanoid robots"
---

import ReadingTime from '@site/src/components/ReadingTime';
import ViewToggle from '@site/src/components/ViewToggle';

<ReadingTime minutes={205} />

<!-- <ViewToggle /> -->

<h1 className="main-heading">Robotic Manipulation System</h1>
<div className="underline-class"></div>

<div className="full-content">

<h2 className="second-heading">
Learning Objectives
</h2>
<div className="underline-class"></div>

After completing this chapter, you will be able to:
- • Design and implement a comprehensive manipulation system for humanoid robots
- • Integrate perception, planning, and control for dexterous manipulation
- • Implement grasp planning and execution for various object types
- • Create robust manipulation strategies for human-robot interaction
- • Ensure safe and efficient manipulation in complex environments

<h2 className="second-heading">
Introduction to Robotic Manipulation
</h2>
<div className="underline-class"></div>

Robotic manipulation represents one of the most challenging and essential capabilities for humanoid robots, enabling them to interact with the physical world in meaningful ways. Unlike simple pick-and-place operations, humanoid manipulation must handle complex kinematics, dexterous grasping, and the need to operate in environments designed for human use.

The manipulation system for a humanoid robot must address multiple interconnected challenges: perceiving objects in 3D space, planning feasible trajectories that avoid self-collisions and environmental obstacles, executing precise movements with multiple degrees of freedom, and adapting to uncertainties in object properties and environmental conditions. This requires tight integration between perception, planning, control, and safety systems.

This chapter explores the implementation of a comprehensive manipulation system for humanoid robots, focusing on the Vision-Language-Action paradigm that connects environmental perception to goal-directed manipulation. The system must be robust enough to handle real-world conditions while maintaining the safety and dexterity required for human-robot interaction.

<h2 className="second-heading">
Perception for Manipulation
</h2>
<div className="underline-class"></div>

<h3 className="third-heading">
- 3D Object Detection and Pose Estimation
</h3>
<div className="underline-class"></div>

Accurate perception of objects in 3D space is fundamental to successful manipulation:

```python
# Example: 3D object detection and pose estimation for manipulation
import numpy as np
import open3d as o3d
from typing import List, Dict, Any, Tuple, Optional
import cv2
from scipy.spatial.transform import Rotation as R
import threading
import time

class ObjectDetectionSystem:
    def __init__(self):
        # Initialize 3D processing pipeline
        self.voxel_size = 0.01  # 1cm resolution for fine detail
        self.table_height = 0.8  # Assumed table height for segmentation

        # Initialize feature extractors
        self.fpfh_radius = self.voxel_size * 2.0
        self.fpfh_max_nn = 100

        # Object models database
        self.object_models = {}
        self.load_object_models()

    def load_object_models(self):
        """Load pre-defined object models for recognition"""
        # In practice, this would load CAD models or point cloud templates
        # For this example, we'll create simple geometric models
        cup_model = o3d.geometry.TriangleMesh.create_cylinder(radius=0.03, height=0.08)
        cup_model.compute_vertex_normals()
        self.object_models['cup'] = cup_model

        box_model = o3d.geometry.TriangleMesh.create_box(width=0.05, height=0.05, depth=0.05)
        box_model.compute_vertex_normals()
        self.object_models['box'] = box_model

        bottle_model = o3d.geometry.TriangleMesh.create_cylinder(radius=0.02, height=0.12)
        bottle_model.compute_vertex_normals()
        self.object_models['bottle'] = bottle_model

    def process_scene(self, point_cloud: np.ndarray,
                     camera_pose: np.ndarray) -> List[Dict[str, Any]]:
        """Process scene to detect and estimate poses of objects"""
        # Convert to Open3D point cloud
        scene_cloud = o3d.geometry.PointCloud()
        scene_cloud.points = o3d.utility.Vector3dVector(point_cloud)

        # Transform to world coordinates if needed
        if camera_pose is not None:
            scene_cloud.transform(camera_pose)

        # Segment table and objects
        table_cloud, objects_cloud = self.segment_table_and_objects(scene_cloud)

        # Extract individual objects
        object_clusters = self.extract_object_clusters(objects_cloud)

        # For each cluster, estimate pose and identify object
        detected_objects = []
        for cluster in object_clusters:
            object_info = self.analyze_object_cluster(cluster)
            if object_info:
                detected_objects.append(object_info)

        return detected_objects

    def segment_table_and_objects(self, point_cloud: o3d.geometry.PointCloud) -> Tuple[o3d.geometry.PointCloud, o3d.geometry.PointCloud]:
        """Segment table surface from objects using RANSAC plane fitting"""
        # Downsample for efficiency
        cloud_down = point_cloud.voxel_down_sample(voxel_size=0.02)

        # Fit plane (table surface)
        plane_model, inliers = cloud_down.segment_plane(
            distance_threshold=0.01,
            ransac_n=3,
            num_iterations=1000
        )

        # Extract table and objects
        table_cloud = cloud_down.select_by_index(inliers)
        objects_cloud = cloud_down.select_by_index(inliers, invert=True)

        return table_cloud, objects_cloud

    def extract_object_clusters(self, objects_cloud: o3d.geometry.PointCloud) -> List[o3d.geometry.PointCloud]:
        """Extract individual object clusters using DBSCAN clustering"""
        # Downsample for clustering
        cloud_down = objects_cloud.voxel_down_sample(voxel_size=0.02)

        # Perform clustering
        labels = np.array(cloud_down.cluster_dbscan(
            eps=0.05,  # 5cm distance threshold
            min_points=10,  # Minimum points for a cluster
            print_progress=False
        ))

        # Extract individual clusters
        clusters = []
        unique_labels = set(labels)

        for label in unique_labels:
            if label == -1:  # Noise points
                continue

            # Get points for this cluster
            cluster_indices = np.where(labels == label)[0]
            cluster_cloud = cloud_down.select_by_index(cluster_indices)

            # Filter small clusters
            if len(cluster_cloud.points) > 50:  # Minimum size threshold
                clusters.append(cluster_cloud)

        return clusters

    def analyze_object_cluster(self, cluster_cloud: o3d.geometry.PointCloud) -> Optional[Dict[str, Any]]:
        """Analyze object cluster to estimate pose and identify object"""
        if len(cluster_cloud.points) < 50:
            return None

        # Estimate bounding box for basic dimensions
        aabb = cluster_cloud.get_axis_aligned_bounding_box()
        obb = cluster_cloud.get_oriented_bounding_box()

        # Calculate dimensions
        dimensions = obb.extent
        center = obb.center

        # Estimate orientation from principal axes
        points = np.asarray(cluster_cloud.points)
        covariance_matrix = np.cov(points.T)
        eigenvalues, eigenvectors = np.linalg.eigh(covariance_matrix)

        # Sort eigenvectors by eigenvalues (largest first)
        idx = np.argsort(eigenvalues)[::-1]
        eigenvectors = eigenvectors[:, idx]

        # Create rotation matrix
        rotation_matrix = eigenvectors
        if np.linalg.det(rotation_matrix) < 0:
            rotation_matrix[:, -1] *= -1  # Ensure right-handed coordinate system

        # Convert to quaternion
        rotation = R.from_matrix(rotation_matrix)
        quaternion = rotation.as_quat()

        # Identify object type based on dimensions
        object_type = self.identify_object_type(dimensions)

        # Calculate object properties
        volume = dimensions[0] * dimensions[1] * dimensions[2]
        surface_area = 2 * (dimensions[0]*dimensions[1] + dimensions[0]*dimensions[2] + dimensions[1]*dimensions[2])

        return {
            'type': object_type,
            'center': center,
            'dimensions': dimensions,
            'rotation': quaternion,
            'volume': volume,
            'surface_area': surface_area,
            'point_cloud': cluster_cloud,
            'bounding_box': obb
        }

    def identify_object_type(self, dimensions: np.ndarray) -> str:
        """Identify object type based on dimensions"""
        height, width, depth = sorted(dimensions, reverse=True)

        # Simple heuristics for object identification
        aspect_ratio = height / max(width, depth) if max(width, depth) > 0 else 1.0

        if aspect_ratio > 2.0:  # Tall object
            if width < 0.05 and depth < 0.05:
                return 'bottle'
            else:
                return 'cup'
        elif aspect_ratio < 1.2:  # Flat object
            return 'box'
        else:
            # Intermediate aspect ratio
            if height < 0.1 and width > 0.05 and depth > 0.05:
                return 'plate'
            else:
                return 'unknown'

    def register_object_model(self, object_name: str, model_cloud: o3d.geometry.PointCloud):
        """Register a new object model for recognition"""
        self.object_models[object_name] = model_cloud

class ManipulationPerceptionSystem:
    """Integrated perception system for manipulation tasks"""
    def __init__(self):
        self.object_detector = ObjectDetectionSystem()
        self.grasp_point_estimator = GraspPointEstimator()
        self.collision_detector = CollisionDetector()

    def perceive_manipulation_scene(self, point_cloud: np.ndarray,
                                  camera_pose: np.ndarray) -> Dict[str, Any]:
        """Perceive the complete manipulation scene"""
        # Detect objects
        objects = self.object_detector.process_scene(point_cloud, camera_pose)

        # Estimate grasp points for each object
        for obj in objects:
            grasp_points = self.grasp_point_estimator.estimate_grasp_points(
                obj['point_cloud'],
                obj['type']
            )
            obj['grasp_points'] = grasp_points

        # Detect potential collision obstacles
        collision_map = self.collision_detector.analyze_environment(point_cloud)

        return {
            'objects': objects,
            'collision_map': collision_map,
            'timestamp': time.time()
        }

class GraspPointEstimator:
    """System for estimating optimal grasp points on objects"""
    def __init__(self):
        # Predefined grasp strategies for different object types
        self.grasp_strategies = {
            'cup': self.estimate_cup_grasps,
            'bottle': self.estimate_bottle_grasps,
            'box': self.estimate_box_grasps,
            'plate': self.estimate_plate_grasps
        }

    def estimate_grasp_points(self, object_cloud: o3d.geometry.PointCloud,
                            object_type: str) -> List[Dict[str, Any]]:
        """Estimate potential grasp points for an object"""
        if object_type in self.grasp_strategies:
            return self.grasp_strategies[object_type](object_cloud)
        else:
            return self.estimate_generic_grasps(object_cloud)

    def estimate_cup_grasps(self, object_cloud: o3d.geometry.PointCloud) -> List[Dict[str, Any]]:
        """Estimate grasp points for cup objects"""
        grasps = []

        # Find handle (if present) - typically a protrusion
        points = np.asarray(object_cloud.points)
        center = np.mean(points, axis=0)

        # Calculate distances from center
        distances = np.linalg.norm(points - center, axis=1)
        avg_distance = np.mean(distances)

        # Look for handle region (points significantly farther than average)
        handle_indices = np.where(distances > avg_distance * 1.5)[0]

        if len(handle_indices) > 10:  # Handle detected
            handle_points = points[handle_indices]
            handle_center = np.mean(handle_points, axis=0)

            # Estimate handle orientation
            handle_pca = np.cov(handle_points.T)
            handle_eigenvals, handle_eigenvecs = np.linalg.eigh(handle_pca)
            handle_axis = handle_eigenvecs[:, -1]  # Principal axis

            # Generate handle grasp
            grasps.append({
                'position': handle_center,
                'orientation': self.calculate_grasp_orientation(handle_axis),
                'type': 'handle_grasp',
                'quality': 0.9
            })
        else:
            # No handle - side grasp
            # Find side points
            side_indices = np.where((distances > avg_distance * 0.8) & (distances < avg_distance * 1.2))[0]
            if len(side_indices) > 0:
                side_points = points[side_indices]
                side_center = np.mean(side_points, axis=0)

                # Estimate side orientation
                side_normal = self.estimate_surface_normal_at_point(
                    object_cloud, side_center
                )

                grasps.append({
                    'position': side_center,
                    'orientation': self.calculate_grasp_orientation(side_normal),
                    'type': 'side_grasp',
                    'quality': 0.7
                })

        return grasps

    def estimate_bottle_grasps(self, object_cloud: o3d.geometry.PointCloud) -> List[Dict[str, Any]]:
        """Estimate grasp points for bottle objects"""
        grasps = []

        # For bottles, look for neck and body regions
        points = np.asarray(object_cloud.points)
        z_sorted = points[np.argsort(points[:, 2])]  # Sort by height

        # Identify neck (top 20% of points)
        neck_start_idx = int(0.8 * len(z_sorted))
        neck_points = z_sorted[neck_start_idx:]

        if len(neck_points) > 5:
            neck_center = np.mean(neck_points, axis=0)

            # Calculate neck orientation (vertical)
            neck_normal = np.array([0, 0, 1])

            grasps.append({
                'position': neck_center,
                'orientation': self.calculate_grasp_orientation(neck_normal),
                'type': 'neck_grasp',
                'quality': 0.8
            })

        # Body grasp
        body_points = z_sorted[:int(0.6 * len(z_sorted))]
        if len(body_points) > 10:
            body_center = np.mean(body_points, axis=0)
            body_normal = self.estimate_surface_normal_at_point(object_cloud, body_center)

            grasps.append({
                'position': body_center,
                'orientation': self.calculate_grasp_orientation(body_normal),
                'type': 'body_grasp',
                'quality': 0.7
            })

        return grasps

    def estimate_box_grasps(self, object_cloud: o3d.geometry.PointCloud) -> List[Dict[str, Any]]:
        """Estimate grasp points for box objects"""
        grasps = []

        # Find corner and face centers
        points = np.asarray(object_cloud.points)
        bbox = object_cloud.get_oriented_bounding_box()

        # Calculate face centers
        face_centers = self.calculate_box_face_centers(bbox)

        for face_center in face_centers:
            surface_normal = self.estimate_surface_normal_at_point(object_cloud, face_center)
            grasps.append({
                'position': face_center,
                'orientation': self.calculate_grasp_orientation(surface_normal),
                'type': 'face_grasp',
                'quality': 0.6
            })

        # Add corner grasps
        corner_points = np.asarray(bbox.get_box_points())
        for corner in corner_points:
            grasps.append({
                'position': corner,
                'orientation': np.eye(3),  # Identity orientation for corner grasps
                'type': 'corner_grasp',
                'quality': 0.5
            })

        return grasps

    def calculate_box_face_centers(self, bbox: o3d.geometry.OrientedBoundingBox) -> List[np.ndarray]:
        """Calculate face centers of a bounding box"""
        box_points = np.asarray(bbox.get_box_points())

        # Define face indices (0-7 are the 8 corners of the box)
        faces = [
            [0, 1, 2, 3],  # Bottom face
            [4, 5, 6, 7],  # Top face
            [0, 1, 4, 5],  # Front face
            [2, 3, 6, 7],  # Back face
            [0, 3, 4, 7],  # Left face
            [1, 2, 5, 6]   # Right face
        ]

        face_centers = []
        for face in faces:
            face_points = box_points[face]
            center = np.mean(face_points, axis=0)
            face_centers.append(center)

        return face_centers

    def estimate_surface_normal_at_point(self, cloud: o3d.geometry.PointCloud,
                                       point: np.ndarray,
                                       radius: float = 0.02) -> np.ndarray:
        """Estimate surface normal at a given point"""
        # Find neighboring points
        points = np.asarray(cloud.points)
        distances = np.linalg.norm(points - point, axis=1)
        neighbor_indices = np.where(distances < radius)[0]

        if len(neighbor_indices) < 3:
            return np.array([0, 0, 1])  # Default normal

        neighbor_points = points[neighbor_indices]

        # Calculate covariance matrix
        covariance = np.cov(neighbor_points.T)

        # Get eigenvectors
        eigenvals, eigenvecs = np.linalg.eigh(covariance)

        # Normal is the eigenvector corresponding to smallest eigenvalue
        normal = eigenvecs[:, 0]

        # Ensure normal points outward (away from center)
        center_to_point = point - np.mean(points, axis=0)
        if np.dot(normal, center_to_point) < 0:
            normal = -normal

        return normal / np.linalg.norm(normal)

    def calculate_grasp_orientation(self, approach_vector: np.ndarray) -> np.ndarray:
        """Calculate grasp orientation matrix from approach vector"""
        # Normalize approach vector
        approach = approach_vector / np.linalg.norm(approach_vector)

        # Calculate orthogonal vectors for complete orientation
        if abs(approach[2]) < 0.9:  # Not nearly vertical
            up = np.array([0, 0, 1])
        else:  # Nearly vertical, use different up vector
            up = np.array([0, 1, 0])

        # Calculate orthogonal vectors
        y_axis = np.cross(up, approach)
        y_axis = y_axis / np.linalg.norm(y_axis)
        x_axis = np.cross(y_axis, approach)

        # Create orientation matrix
        orientation = np.eye(3)
        orientation[:, 0] = x_axis  # X axis (gripper approach)
        orientation[:, 1] = y_axis  # Y axis (gripper opening)
        orientation[:, 2] = approach  # Z axis (normal to surface)

        return orientation

    def estimate_generic_grasps(self, object_cloud: o3d.geometry.PointCloud) -> List[Dict[str, Any]]:
        """Estimate generic grasp points when object type is unknown"""
        grasps = []

        # Use geometric features to find potential grasp points
        points = np.asarray(object_cloud.points)

        # Calculate center of mass
        center = np.mean(points, axis=0)

        # Find points at various orientations around the object
        for angle in np.linspace(0, 2*np.pi, 8):  # 8 directions
            direction = np.array([np.cos(angle), np.sin(angle), 0])
            target_point = center + direction * 0.05  # 5cm offset

            # Find closest point on surface
            distances = np.linalg.norm(points - target_point, axis=1)
            closest_idx = np.argmin(distances)
            surface_point = points[closest_idx]

            normal = self.estimate_surface_normal_at_point(object_cloud, surface_point)

            grasps.append({
                'position': surface_point,
                'orientation': self.calculate_grasp_orientation(normal),
                'type': 'surface_grasp',
                'quality': 0.4
            })

        return grasps
```

<h3 className="third-heading">
- Collision Detection and Avoidance
</h3>
<div className="underline-class"></div>

Safe manipulation requires comprehensive collision detection:

```python
# Example: Collision detection system for manipulation
class CollisionDetector:
    def __init__(self):
        self.robot_model = self.load_robot_model()
        self.environment_map = o3d.geometry.PointCloud()
        self.collision_threshold = 0.02  # 2cm clearance

    def load_robot_model(self) -> Dict[str, Any]:
        """Load robot kinematic model for collision checking"""
        # Define robot links with their collision volumes
        robot_model = {
            'base': {
                'shape': 'box',
                'dimensions': [0.3, 0.3, 0.1],  # width, depth, height
                'offset': [0, 0, 0.05]  # offset from joint
            },
            'torso': {
                'shape': 'box',
                'dimensions': [0.2, 0.2, 0.6],
                'offset': [0, 0, 0.4]
            },
            'upper_arm_left': {
                'shape': 'cylinder',
                'dimensions': [0.05, 0.3],  # radius, length
                'offset': [-0.15, 0, 0.5]
            },
            'forearm_left': {
                'shape': 'cylinder',
                'dimensions': [0.04, 0.25],
                'offset': [-0.15, 0, 0.25]
            },
            'hand_left': {
                'shape': 'box',
                'dimensions': [0.1, 0.08, 0.06],
                'offset': [-0.15, 0, 0.1]
            },
            'upper_arm_right': {
                'shape': 'cylinder',
                'dimensions': [0.05, 0.3],
                'offset': [0.15, 0, 0.5]
            },
            'forearm_right': {
                'shape': 'cylinder',
                'dimensions': [0.04, 0.25],
                'offset': [0.15, 0, 0.25]
            },
            'hand_right': {
                'shape': 'box',
                'dimensions': [0.1, 0.08, 0.06],
                'offset': [0.15, 0, 0.1]
            }
        }
        return robot_model

    def analyze_environment(self, point_cloud: np.ndarray) -> Dict[str, Any]:
        """Analyze environment for collision obstacles"""
        # Convert to Open3D point cloud
        env_cloud = o3d.geometry.PointCloud()
        env_cloud.points = o3d.utility.Vector3dVector(point_cloud)

        # Downsample for efficiency
        env_cloud = env_cloud.voxel_down_sample(voxel_size=0.02)

        self.environment_map = env_cloud

        # Identify potential collision objects
        collision_objects = self.identify_collision_objects(env_cloud)

        return {
            'collision_objects': collision_objects,
            'map': env_cloud,
            'timestamp': time.time()
        }

    def identify_collision_objects(self, env_cloud: o3d.geometry.PointCloud) -> List[Dict[str, Any]]:
        """Identify objects that could cause collisions"""
        # Segment objects using clustering
        clusters = self.extract_object_clusters(env_cloud)

        collision_objects = []
        for cluster in clusters:
            # Calculate bounding box and check size
            bbox = cluster.get_axis_aligned_bounding_box()
            dimensions = bbox.extent

            # Only consider objects that are large enough to be obstacles
            if np.all(dimensions > [0.05, 0.05, 0.05]):  # 5cm minimum in each dimension
                collision_objects.append({
                    'point_cloud': cluster,
                    'bounding_box': bbox,
                    'dimensions': dimensions,
                    'center': bbox.center
                })

        return collision_objects

    def extract_object_clusters(self, cloud: o3d.geometry.PointCloud) -> List[o3d.geometry.PointCloud]:
        """Extract object clusters using DBSCAN"""
        labels = np.array(cloud.cluster_dbscan(
            eps=0.05,
            min_points=20,
            print_progress=False
        ))

        clusters = []
        unique_labels = set(labels)

        for label in unique_labels:
            if label == -1:  # Noise
                continue

            cluster_indices = np.where(labels == label)[0]
            cluster_cloud = cloud.select_by_index(cluster_indices)

            if len(cluster_cloud.points) > 50:  # Minimum size
                clusters.append(cluster_cloud)

        return clusters

    def check_trajectory_collision(self, trajectory: List[np.ndarray],
                                 joint_angles: List[np.ndarray]) -> Dict[str, Any]:
        """Check if a trajectory has collision risks"""
        collision_risks = []

        for i, (pose, joints) in enumerate(zip(trajectory, joint_angles)):
            # Transform robot model to current configuration
            robot_links = self.transform_robot_model(joints)

            # Check collision with environment
            collision_result = self.check_robot_environment_collision(
                robot_links, self.environment_map
            )

            if collision_result['collision']:
                collision_risks.append({
                    'step': i,
                    'pose': pose,
                    'joints': joints,
                    'collision_links': collision_result['collision_links'],
                    'collision_points': collision_result['collision_points']
                })

        return {
            'has_collision': len(collision_risks) > 0,
            'collision_risks': collision_risks,
            'safe_steps': len(trajectory) - len(collision_risks)
        }

    def transform_robot_model(self, joint_angles: np.ndarray) -> Dict[str, Any]:
        """Transform robot model based on joint angles"""
        # This would implement forward kinematics
        # For this example, return simplified link positions
        links = {}

        # Base is at origin
        links['base'] = {
            'position': np.array([0, 0, 0]),
            'orientation': np.eye(3)
        }

        # Torso (simplified)
        links['torso'] = {
            'position': np.array([0, 0, 0.5]),
            'orientation': np.eye(3)
        }

        # Arms with simplified kinematics
        # Left arm
        left_elbow_angle = joint_angles[0] if len(joint_angles) > 0 else 0
        left_wrist_angle = joint_angles[1] if len(joint_angles) > 1 else 0

        left_shoulder = np.array([-0.15, 0, 0.6])
        left_elbow = left_shoulder + np.array([
            0,
            0.3 * np.cos(left_elbow_angle),
            -0.3 * np.sin(left_elbow_angle)
        ])
        left_wrist = left_elbow + np.array([
            0,
            0.25 * np.cos(left_elbow_angle + left_wrist_angle),
            -0.25 * np.sin(left_elbow_angle + left_wrist_angle)
        ])

        links['upper_arm_left'] = {'position': left_shoulder, 'orientation': np.eye(3)}
        links['forearm_left'] = {'position': left_elbow, 'orientation': np.eye(3)}
        links['hand_left'] = {'position': left_wrist, 'orientation': np.eye(3)}

        # Right arm (similar calculation)
        right_elbow_angle = joint_angles[2] if len(joint_angles) > 2 else 0
        right_wrist_angle = joint_angles[3] if len(joint_angles) > 3 else 0

        right_shoulder = np.array([0.15, 0, 0.6])
        right_elbow = right_shoulder + np.array([
            0,
            0.3 * np.cos(right_elbow_angle),
            -0.3 * np.sin(right_elbow_angle)
        ])
        right_wrist = right_elbow + np.array([
            0,
            0.25 * np.cos(right_elbow_angle + right_wrist_angle),
            -0.25 * np.sin(right_elbow_angle + right_wrist_angle)
        ])

        links['upper_arm_right'] = {'position': right_shoulder, 'orientation': np.eye(3)}
        links['forearm_right'] = {'position': right_elbow, 'orientation': np.eye(3)}
        links['hand_right'] = {'position': right_wrist, 'orientation': np.eye(3)}

        return links

    def check_robot_environment_collision(self, robot_links: Dict[str, Any],
                                        env_cloud: o3d.geometry.PointCloud) -> Dict[str, Any]:
        """Check for collisions between robot links and environment"""
        collision_links = []
        collision_points = []

        for link_name, link_info in robot_links.items():
            link_pos = link_info['position']

            # Get link collision volume parameters
            link_params = self.robot_model.get(link_name, {})
            if not link_params:
                continue

            # Check distance to environment points
            env_points = np.asarray(env_cloud.points)
            distances = np.linalg.norm(env_points - link_pos, axis=1)

            # Check for collisions based on link size
            min_distance = np.min(distances) if len(distances) > 0 else float('inf')
            safety_margin = self.get_link_safety_margin(link_params)

            if min_distance < safety_margin:
                collision_links.append(link_name)
                collision_idx = np.argmin(distances)
                collision_points.append(env_points[collision_idx])

        return {
            'collision': len(collision_links) > 0,
            'collision_links': collision_links,
            'collision_points': collision_points
        }

    def get_link_safety_margin(self, link_params: Dict[str, Any]) -> float:
        """Get safety margin for a robot link based on its size"""
        if link_params['shape'] == 'box':
            dimensions = link_params['dimensions']
            max_dim = max(dimensions)
        elif link_params['shape'] == 'cylinder':
            radius, length = link_params['dimensions']
            max_dim = max(radius, length)
        else:
            max_dim = 0.1  # Default size

        return max_dim / 2 + self.collision_threshold
```

<h2 className="second-heading">
Grasp Planning and Execution
</h2>
<div className="underline-class"></div>

<h3 className="third-heading">
- Grasp Synthesis and Evaluation
</h3>
<div className="underline-class"></div>

Creating stable grasps requires understanding object properties and robot capabilities:

```python
# Example: Grasp planning and synthesis system
class GraspPlanner:
    def __init__(self):
        self.grasp_database = self.load_grasp_database()
        self.stability_evaluator = GraspStabilityEvaluator()
        self.force_closure_analyzer = ForceClosureAnalyzer()

    def load_grasp_database(self) -> Dict[str, List[Dict[str, Any]]]:
        """Load pre-computed grasp configurations for common objects"""
        # This would load grasp data from a database or file
        # For this example, create simple grasp templates
        grasp_database = {
            'cup': [
                {
                    'type': 'cylindrical',
                    'approach': [0, 0, 1],  # Approach from above
                    'grasp_width': 0.06,
                    'quality': 0.8,
                    'required_dofs': ['finger_1', 'finger_2']
                },
                {
                    'type': 'lateral',
                    'approach': [1, 0, 0],  # Approach from side
                    'grasp_width': 0.08,
                    'quality': 0.7,
                    'required_dofs': ['finger_1', 'finger_2']
                }
            ],
            'bottle': [
                {
                    'type': 'cylindrical',
                    'approach': [0, 0, 1],
                    'grasp_width': 0.04,
                    'quality': 0.85,
                    'required_dofs': ['finger_1', 'finger_2']
                }
            ],
            'box': [
                {
                    'type': 'parallel',
                    'approach': [0, 0, 1],
                    'grasp_width': 0.05,
                    'quality': 0.75,
                    'required_dofs': ['finger_1', 'finger_2', 'finger_3']
                }
            ]
        }
        return grasp_database

    def plan_grasps(self, object_info: Dict[str, Any],
                   robot_hand_config: Dict[str, Any]) -> List[Dict[str, Any]]:
        """Plan grasps for a given object"""
        object_type = object_info['type']
        object_pose = object_info['center']
        object_dims = object_info['dimensions']

        # Get pre-defined grasps for object type
        predefined_grasps = self.grasp_database.get(object_type, [])

        # Generate additional grasps based on object geometry
        geometric_grasps = self.generate_geometric_grasps(object_info)

        # Combine and evaluate all grasps
        all_grasps = predefined_grasps + geometric_grasps

        # Evaluate each grasp for the specific object instance
        evaluated_grasps = []
        for grasp in all_grasps:
            evaluation = self.evaluate_grasp(grasp, object_info, robot_hand_config)
            if evaluation['feasible']:
                evaluated_grasps.append({
                    'grasp': grasp,
                    'evaluation': evaluation,
                    'score': evaluation['stability'] * 0.6 + evaluation['quality'] * 0.4
                })

        # Sort by score
        evaluated_grasps.sort(key=lambda x: x['score'], reverse=True)

        return evaluated_grasps

    def generate_geometric_grasps(self, object_info: Dict[str, Any]) -> List[Dict[str, Any]]:
        """Generate grasps based on object geometric properties"""
        object_type = object_info['type']
        dimensions = object_info['dimensions']

        grasps = []

        if object_type == 'box':
            # Generate corner grasps
            for i in range(8):  # 8 corners of a box
                grasp = {
                    'type': 'corner',
                    'approach': self.calculate_corner_approach(i, dimensions),
                    'grasp_width': min(dimensions) * 0.8,
                    'quality': 0.6,
                    'required_dofs': ['finger_1', 'finger_2', 'finger_3']
                }
                grasps.append(grasp)

            # Generate face grasps
            for axis in range(3):
                grasp = {
                    'type': 'face',
                    'approach': self.calculate_face_approach(axis),
                    'grasp_width': dimensions[axis] * 0.9,
                    'quality': 0.7,
                    'required_dofs': ['finger_1', 'finger_2']
                }
                grasps.append(grasp)

        elif object_type in ['cup', 'bottle']:
            # Generate cylindrical grasps
            for angle in np.linspace(0, 2*np.pi, 8):
                approach = [np.cos(angle), np.sin(angle), 0]
                grasp = {
                    'type': 'cylindrical',
                    'approach': approach,
                    'grasp_width': dimensions[0] * 0.8,  # Use smallest dimension
                    'quality': 0.75,
                    'required_dofs': ['finger_1', 'finger_2']
                }
                grasps.append(grasp)

        return grasps

    def calculate_corner_approach(self, corner_idx: int, dimensions: np.ndarray) -> List[float]:
        """Calculate approach vector for corner grasp"""
        # Convert corner index to 3D binary representation
        x_dir = 1 if (corner_idx & 1) else -1
        y_dir = 1 if (corner_idx & 2) else -1
        z_dir = 1 if (corner_idx & 4) else -1

        return [x_dir, y_dir, z_dir]

    def calculate_face_approach(self, axis: int) -> List[float]:
        """Calculate approach vector for face grasp"""
        approach = [0, 0, 0]
        approach[axis] = 1
        return approach

    def evaluate_grasp(self, grasp: Dict[str, Any],
                      object_info: Dict[str, Any],
                      robot_hand_config: Dict[str, Any]) -> Dict[str, Any]:
        """Evaluate a grasp for feasibility and quality"""
        # Check geometric feasibility
        geometric_feasible = self.check_geometric_feasibility(
            grasp, object_info, robot_hand_config
        )

        # Check force closure
        force_closure = self.force_closure_analyzer.check_force_closure(
            grasp, object_info
        )

        # Evaluate stability
        stability = self.stability_evaluator.evaluate_stability(
            grasp, object_info
        )

        # Check robot constraints
        robot_feasible = self.check_robot_constraints(grasp, robot_hand_config)

        return {
            'feasible': geometric_feasible and force_closure and robot_feasible,
            'geometric_feasible': geometric_feasible,
            'force_closure': force_closure,
            'stability': stability,
            'robot_feasible': robot_feasible,
            'quality': grasp.get('quality', 0.5),
            'warnings': self.get_evaluation_warnings(grasp, object_info)
        }

    def check_geometric_feasibility(self, grasp: Dict[str, Any],
                                  object_info: Dict[str, Any],
                                  robot_hand_config: Dict[str, Any]) -> bool:
        """Check if grasp is geometrically feasible"""
        grasp_width = grasp.get('grasp_width', 0)
        object_dims = object_info['dimensions']

        # Check if grasp width is appropriate for object size
        min_object_dim = min(object_dims)
        max_object_dim = max(object_dims)

        if grasp_width > max_object_dim * 1.2:  # Too wide
            return False

        if grasp_width < min_object_dim * 0.5:  # Too narrow
            return False

        # Check if robot hand can achieve required grasp width
        max_hand_width = robot_hand_config.get('max_aperture', 0.1)
        if grasp_width > max_hand_width:
            return False

        return True

    def check_robot_constraints(self, grasp: Dict[str, Any],
                              robot_hand_config: Dict[str, Any]) -> bool:
        """Check if grasp satisfies robot hardware constraints"""
        required_dofs = grasp.get('required_dofs', [])
        available_dofs = robot_hand_config.get('dof_names', [])

        # Check if required DOFs are available
        for dof in required_dofs:
            if dof not in available_dofs:
                return False

        return True

    def get_evaluation_warnings(self, grasp: Dict[str, Any],
                              object_info: Dict[str, Any]) -> List[str]:
        """Get warnings for grasp evaluation"""
        warnings = []

        if object_info.get('type') == 'unknown':
            warnings.append("Object type unknown, using generic grasp")

        if grasp.get('quality', 0) < 0.5:
            warnings.append("Low quality grasp")

        return warnings

class GraspStabilityEvaluator:
    """System for evaluating grasp stability"""
    def __init__(self):
        self.friction_coefficient = 0.5
        self.stability_threshold = 0.6

    def evaluate_stability(self, grasp: Dict[str, Any],
                          object_info: Dict[str, Any]) -> float:
        """Evaluate the stability of a grasp"""
        # Consider object properties
        object_mass = object_info.get('mass', 0.5)  # Default 0.5kg
        object_com = object_info.get('center', [0, 0, 0])
        object_dims = object_info['dimensions']

        # Consider grasp configuration
        grasp_type = grasp.get('type', 'unknown')
        grasp_width = grasp.get('grasp_width', 0.05)

        # Calculate stability based on grasp type and object properties
        if grasp_type == 'cylindrical':
            # Cylindrical grasp stability
            stability = self.evaluate_cylindrical_stability(
                grasp_width, object_dims, object_mass
            )
        elif grasp_type == 'parallel':
            # Parallel grasp stability
            stability = self.evaluate_parallel_stability(
                grasp_width, object_dims, object_mass
            )
        elif grasp_type == 'corner':
            # Corner grasp stability
            stability = self.evaluate_corner_stability(
                object_dims, object_mass
            )
        else:
            # Default stability for unknown grasp types
            stability = 0.5

        # Adjust for object properties
        stability *= self.adjust_for_object_properties(object_info)

        return min(1.0, max(0.0, stability))

    def evaluate_cylindrical_stability(self, grasp_width: float,
                                     object_dims: np.ndarray,
                                     object_mass: float) -> float:
        """Evaluate stability of cylindrical grasp"""
        # For cylindrical grasp, stability depends on grasp width relative to object diameter
        object_diameter = max(object_dims[0], object_dims[1])  # Assuming circular cross-section
        grasp_efficiency = min(grasp_width / object_diameter, 1.0)

        # Consider object height-to-diameter ratio
        height_ratio = object_dims[2] / object_diameter
        height_factor = min(height_ratio * 0.3, 0.7)  # Taller objects are harder to grasp

        # Combine factors
        stability = 0.4 * grasp_efficiency + 0.3 * height_factor + 0.3

        return stability

    def evaluate_parallel_stability(self, grasp_width: float,
                                  object_dims: np.ndarray,
                                  object_mass: float) -> float:
        """Evaluate stability of parallel grasp"""
        # For parallel grasp, consider the contact area and object dimensions
        min_dim = min(object_dims)
        max_dim = max(object_dims)

        width_ratio = grasp_width / max_dim
        width_factor = min(width_ratio * 0.5, 0.5)

        # Consider object thickness
        thickness_factor = min(min_dim * 2.0, 0.5)

        stability = 0.4 * width_factor + 0.3 * thickness_factor + 0.3

        return stability

    def evaluate_corner_stability(self, object_dims: np.ndarray,
                                object_mass: float) -> float:
        """Evaluate stability of corner grasp"""
        # Corner grasps are generally less stable
        min_dim = min(object_dims)
        max_dim = max(object_dims)

        # Stability depends on how much of the object is supported
        support_ratio = min_dim / max_dim
        stability = 0.3 + 0.4 * support_ratio  # Base 0.3 for corner grasps

        return stability

    def adjust_for_object_properties(self, object_info: Dict[str, Any]) -> float:
        """Adjust stability based on object properties"""
        adjustment = 1.0

        # Adjust for object material (simplified)
        material = object_info.get('material', 'unknown')
        if material == 'metal':
            adjustment *= 1.1  # Metal objects provide better grip
        elif material == 'plastic':
            adjustment *= 0.9  # Plastic objects may be slippery
        elif material == 'glass':
            adjustment *= 0.8  # Glass requires careful handling

        # Adjust for surface texture
        texture = object_info.get('texture', 'smooth')
        if texture == 'rough':
            adjustment *= 1.1  # Rough surfaces provide better grip
        elif texture == 'smooth':
            adjustment *= 0.9  # Smooth surfaces are slippery
        elif texture == 'textured':
            adjustment *= 1.05

        return adjustment

class ForceClosureAnalyzer:
    """System for analyzing force closure of grasps"""
    def __init__(self):
        self.num_fingers = 2  # For 2-finger gripper
        self.friction_cone_angle = np.deg2rad(15)  # 15 degrees friction cone

    def check_force_closure(self, grasp: Dict[str, Any],
                           object_info: Dict[str, Any]) -> bool:
        """Check if grasp provides force closure"""
        # For a 2D approximation, force closure requires that contact forces
        # can resist any external wrench

        # Get contact points and normals
        contact_points, contact_normals = self.calculate_contact_points(grasp, object_info)

        if len(contact_points) < 2:
            return False

        # Check if contact normals point inward (toward object center)
        object_center = object_info.get('center', [0, 0, 0])

        for i, (point, normal) in enumerate(zip(contact_points, contact_normals)):
            # Vector from contact point to object center
            to_center = np.array(object_center) - np.array(point)
            # Normal should point toward object (dot product should be negative)
            if np.dot(normal, to_center) < 0:
                # Normal points outward, need to flip it
                contact_normals[i] = -normal

        # For 2D case with 2 contacts, force closure exists if normals are not parallel
        if len(contact_normals) >= 2:
            normal1 = contact_normals[0]
            normal2 = contact_normals[1]

            # Check if normals are parallel (dot product close to 1 or -1)
            dot_product = abs(np.dot(normal1 / np.linalg.norm(normal1),
                                   normal2 / np.linalg.norm(normal2)))

            # If dot product is close to 1, normals are parallel and no force closure
            if abs(dot_product) > 0.95:
                return False

        # For more complex analysis, we would check the grasp matrix
        # For this simplified version, assume force closure if contacts are not parallel
        return True

    def calculate_contact_points(self, grasp: Dict[str, Any],
                               object_info: Dict[str, Any]) -> Tuple[List[np.ndarray], List[np.ndarray]]:
        """Calculate contact points and normals for a grasp"""
        object_center = object_info.get('center', [0, 0, 0])
        object_dims = object_info['dimensions']
        object_type = object_info['type']

        contact_points = []
        contact_normals = []

        if object_type == 'box':
            # For box, contacts are typically on opposite faces
            approach = np.array(grasp.get('approach', [0, 0, 1]))
            approach = approach / np.linalg.norm(approach)

            # Find face normal closest to approach
            face_normals = [
                np.array([1, 0, 0]),
                np.array([-1, 0, 0]),
                np.array([0, 1, 0]),
                np.array([0, -1, 0]),
                np.array([0, 0, 1]),
                np.array([0, 0, -1])
            ]

            # Find two opposing faces
            best_normal_idx = np.argmax([abs(np.dot(approach, n)) for n in face_normals])
            primary_normal = face_normals[best_normal_idx]
            opposing_normal = -primary_normal

            # Calculate contact points on these faces
            primary_point = np.array(object_center) + primary_normal * (object_dims[best_normal_idx % 3] / 2)
            opposing_point = np.array(object_center) + opposing_normal * (object_dims[best_normal_idx % 3] / 2)

            contact_points = [primary_point, opposing_point]
            contact_normals = [primary_normal, opposing_normal]

        elif object_type in ['cup', 'bottle']:
            # For cylindrical objects, contacts are typically on opposite sides
            approach = np.array(grasp.get('approach', [1, 0, 0]))
            approach = approach / np.linalg.norm(approach)

            # Find perpendicular direction for contacts
            if abs(approach[2]) < 0.9:  # Not vertical approach
                perp_dir = np.array([-approach[1], approach[0], 0])  # Perpendicular in XY plane
            else:  # Vertical approach
                perp_dir = np.array([1, 0, 0])  # Default horizontal

            perp_dir = perp_dir / np.linalg.norm(perp_dir)

            # Calculate contact points on cylinder surface
            radius = min(object_dims[:2]) / 2  # Assume circular cross-section
            point1 = np.array(object_center) + perp_dir * radius
            point2 = np.array(object_center) - perp_dir * radius

            normal1 = perp_dir
            normal2 = -perp_dir

            contact_points = [point1, point2]
            contact_normals = [normal1, normal2]

        return contact_points, contact_normals
```

<h3 className="third-heading">
- Manipulation Trajectory Planning
</h3>
<div className="underline-class"></div>

Planning safe and efficient manipulation trajectories:

```python
# Example: Manipulation trajectory planning
class ManipulationTrajectoryPlanner:
    def __init__(self):
        self.collision_checker = CollisionDetector()
        self.ik_solver = InverseKinematicsSolver()
        self.smoothing_enabled = True

    def plan_manipulation_trajectory(self, start_pose: Dict[str, Any],
                                   goal_pose: Dict[str, Any],
                                   object_info: Dict[str, Any],
                                   robot_config: Dict[str, Any],
                                   environment_info: Dict[str, Any]) -> Dict[str, Any]:
        """Plan complete manipulation trajectory"""
        # Plan approach trajectory
        approach_trajectory = self.plan_approach_trajectory(
            start_pose, goal_pose, object_info, robot_config, environment_info
        )

        if not approach_trajectory['success']:
            return approach_trajectory

        # Plan grasp trajectory
        grasp_trajectory = self.plan_grasp_trajectory(
            goal_pose, object_info, robot_config
        )

        if not grasp_trajectory['success']:
            return grasp_trajectory

        # Plan lift trajectory
        lift_trajectory = self.plan_lift_trajectory(
            goal_pose, object_info, robot_config
        )

        # Combine trajectories
        complete_trajectory = self.combine_trajectories([
            approach_trajectory['trajectory'],
            grasp_trajectory['trajectory'],
            lift_trajectory['trajectory']
        ])

        # Smooth trajectory
        if self.smoothing_enabled:
            complete_trajectory = self.smoothen_trajectory(complete_trajectory)

        # Validate trajectory
        validation_result = self.validate_trajectory(
            complete_trajectory, robot_config, environment_info
        )

        return {
            'success': validation_result['valid'],
            'trajectory': complete_trajectory,
            'validation': validation_result,
            'execution_time': time.time() - approach_trajectory.get('start_time', time.time())
        }

    def plan_approach_trajectory(self, start_pose: Dict[str, Any],
                               goal_pose: Dict[str, Any],
                               object_info: Dict[str, Any],
                               robot_config: Dict[str, Any],
                               environment_info: Dict[str, Any]) -> Dict[str, Any]:
        """Plan approach trajectory to object"""
        start_time = time.time()

        # Calculate approach pose (above object)
        approach_height = 0.1  # 10cm above object
        approach_pose = goal_pose.copy()
        approach_pose['position'][2] += approach_height

        # Plan path from start to approach pose
        path = self.plan_cartesian_path(start_pose, approach_pose)

        # Add intermediate waypoints for safety
        safe_path = self.add_safe_waypoints(path, environment_info)

        # Convert to joint space trajectory
        joint_trajectory = self.cartesian_to_joint_trajectory(
            safe_path, robot_config
        )

        return {
            'success': len(joint_trajectory) > 0,
            'trajectory': joint_trajectory,
            'path': safe_path,
            'start_time': start_time
        }

    def plan_grasp_trajectory(self, goal_pose: Dict[str, Any],
                            object_info: Dict[str, Any],
                            robot_config: Dict[str, Any]) -> Dict[str, Any]:
        """Plan trajectory for grasping"""
        # Plan descent to object
        grasp_poses = []

        # Approach pose (above object)
        approach_pose = goal_pose.copy()
        approach_pose['position'][2] += 0.05  # 5cm above object

        # Grasp pose (at object)
        grasp_poses.append(approach_pose)
        grasp_poses.append(goal_pose)

        # Convert to joint space
        joint_trajectory = self.cartesian_to_joint_trajectory(
            grasp_poses, robot_config
        )

        return {
            'success': len(joint_trajectory) > 0,
            'trajectory': joint_trajectory,
            'grasp_poses': grasp_poses
        }

    def plan_lift_trajectory(self, grasp_pose: Dict[str, Any],
                           object_info: Dict[str, Any],
                           robot_config: Dict[str, Any]) -> Dict[str, Any]:
        """Plan trajectory for lifting object"""
        lift_height = 0.1  # Lift 10cm

        # Lift pose
        lift_pose = grasp_pose.copy()
        lift_pose['position'][2] += lift_height

        # Plan lift trajectory
        trajectory = [
            grasp_pose,  # Current grasp pose
            lift_pose    # Lifted pose
        ]

        joint_trajectory = self.cartesian_to_joint_trajectory(
            trajectory, robot_config
        )

        return {
            'success': len(joint_trajectory) > 0,
            'trajectory': joint_trajectory,
            'lift_pose': lift_pose
        }

    def plan_cartesian_path(self, start_pose: Dict[str, Any],
                          end_pose: Dict[str, Any],
                          resolution: float = 0.01) -> List[Dict[str, Any]]:
        """Plan straight-line Cartesian path between poses"""
        start_pos = np.array(start_pose['position'])
        end_pos = np.array(end_pose['position'])

        # Calculate path length
        path_vec = end_pos - start_pos
        path_length = np.linalg.norm(path_vec)

        if path_length < resolution:
            return [start_pose, end_pose]

        # Calculate number of waypoints
        num_waypoints = max(2, int(path_length / resolution) + 1)

        path = []
        for i in range(num_waypoints):
            ratio = i / (num_waypoints - 1)
            pos = start_pos + ratio * path_vec

            # Interpolate orientation
            if 'orientation' in start_pose and 'orientation' in end_pose:
                # Use spherical linear interpolation for orientation
                start_quat = start_pose['orientation']
                end_quat = end_pose['orientation']
                interp_quat = self.slerp_quaternions(start_quat, end_quat, ratio)
                orientation = interp_quat
            else:
                orientation = [0, 0, 0, 1]  # Identity quaternion

            path.append({
                'position': pos.tolist(),
                'orientation': orientation
            })

        return path

    def slerp_quaternions(self, q1: List[float], q2: List[float], t: float) -> List[float]:
        """Spherical linear interpolation between quaternions"""
        q1 = np.array(q1)
        q2 = np.array(q2)

        # Calculate dot product
        dot = np.dot(q1, q2)

        # If dot product is negative, negate one quaternion to take shorter path
        if dot < 0.0:
            q2 = -q2
            dot = -dot

        # Clamp dot product to be in valid range
        dot = np.clip(dot, -1.0, 1.0)

        # Calculate angle
        theta_0 = np.arccos(abs(dot))
        sin_theta_0 = np.sin(theta_0)

        # If sin_theta_0 is small, use linear interpolation
        if sin_theta_0 < 1e-6:
            return (1.0 - t) * q1 + t * q2

        # Calculate interpolation
        theta = theta_0 * t
        sin_theta = np.sin(theta)
        s0 = np.cos(theta) - dot * sin_theta / sin_theta_0
        s1 = sin_theta / sin_theta_0

        return (s0 * q1 + s1 * q2).tolist()

    def add_safe_waypoints(self, path: List[Dict[str, Any]],
                          environment_info: Dict[str, Any]) -> List[Dict[str, Any]]:
        """Add safe waypoints to avoid obstacles"""
        if not environment_info.get('collision_objects'):
            return path

        safe_path = [path[0]]  # Start with first point

        for i in range(1, len(path)):
            current_pose = path[i]
            prev_pose = safe_path[-1]

            # Check if direct path is safe
            if self.is_path_safe(prev_pose, current_pose, environment_info):
                safe_path.append(current_pose)
            else:
                # Find safe intermediate waypoint
                safe_waypoint = self.find_safe_waypoint(prev_pose, current_pose, environment_info)
                if safe_waypoint:
                    safe_path.append(safe_waypoint)
                    safe_path.append(current_pose)
                else:
                    # Could not find safe path, return to previous safe point
                    return safe_path

        return safe_path

    def is_path_safe(self, start_pose: Dict[str, Any],
                    end_pose: Dict[str, Any],
                    environment_info: Dict[str, Any]) -> bool:
        """Check if path between two poses is safe"""
        # Sample points along the path and check each for collisions
        path = self.plan_cartesian_path(start_pose, end_pose, resolution=0.02)

        for pose in path:
            if self.is_pose_safe(pose, environment_info):
                continue
            else:
                return False

        return True

    def is_pose_safe(self, pose: Dict[str, Any],
                    environment_info: Dict[str, Any]) -> bool:
        """Check if a single pose is safe"""
        # Convert pose to robot configuration
        # This would require inverse kinematics or forward kinematics
        # For this example, we'll use a simplified check

        pos = np.array(pose['position'])
        env_points = np.asarray(environment_info['map'].points)

        # Check minimum distance to obstacles
        if len(env_points) > 0:
            distances = np.linalg.norm(env_points - pos, axis=1)
            min_distance = np.min(distances) if len(distances) > 0 else float('inf')
            return min_distance > 0.1  # 10cm safety margin

        return True

    def find_safe_waypoint(self, start_pose: Dict[str, Any],
                          end_pose: Dict[str, Any],
                          environment_info: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """Find a safe intermediate waypoint"""
        start_pos = np.array(start_pose['position'])
        end_pos = np.array(end_pose['position'])

        # Try several intermediate heights
        for height_factor in [0.3, 0.5, 0.7]:
            mid_pos = start_pos + height_factor * (end_pos - start_pos)
            # Add some lateral offset to avoid obstacles
            mid_pos[2] += 0.2  # Lift up

            mid_pose = {
                'position': mid_pos.tolist(),
                'orientation': start_pose.get('orientation', [0, 0, 0, 1])
            }

            # Check if this waypoint is safe
            if self.is_pose_safe(mid_pose, environment_info):
                return mid_pose

        return None

    def cartesian_to_joint_trajectory(self, cartesian_path: List[Dict[str, Any]],
                                    robot_config: Dict[str, Any]) -> List[np.ndarray]:
        """Convert Cartesian path to joint space trajectory"""
        joint_trajectory = []

        for pose in cartesian_path:
            joint_angles = self.ik_solver.solve_inverse_kinematics(
                pose, robot_config
            )
            if joint_angles is not None:
                joint_trajectory.append(joint_angles)

        return joint_trajectory

    def combine_trajectories(self, trajectories: List[List[Any]]) -> List[Any]:
        """Combine multiple trajectories into one"""
        combined = []
        for traj in trajectories:
            combined.extend(traj)
        return combined

    def smoothen_trajectory(self, trajectory: List[np.ndarray]) -> List[np.ndarray]:
        """Apply smoothing to trajectory"""
        if len(trajectory) < 3:
            return trajectory

        smoothed = [trajectory[0]]  # Keep first point

        for i in range(1, len(trajectory) - 1):
            # Apply simple smoothing (average with neighbors)
            prev_pt = trajectory[i-1]
            curr_pt = trajectory[i]
            next_pt = trajectory[i+1]

            smoothed_pt = 0.25 * prev_pt + 0.5 * curr_pt + 0.25 * next_pt
            smoothed.append(smoothed_pt)

        smoothed.append(trajectory[-1])  # Keep last point
        return smoothed

    def validate_trajectory(self, trajectory: List[np.ndarray],
                          robot_config: Dict[str, Any],
                          environment_info: Dict[str, Any]) -> Dict[str, Any]:
        """Validate trajectory for collisions and joint limits"""
        collision_free = True
        within_limits = True
        collision_risks = []

        for i, joints in enumerate(trajectory):
            # Check joint limits
            if not self.check_joint_limits(joints, robot_config):
                within_limits = False

            # Check collisions (simplified)
            if self.check_collision_at_configuration(joints, environment_info):
                collision_free = False
                collision_risks.append(i)

        return {
            'valid': collision_free and within_limits,
            'collision_free': collision_free,
            'within_limits': within_limits,
            'collision_risks': collision_risks
        }

    def check_joint_limits(self, joints: np.ndarray,
                          robot_config: Dict[str, Any]) -> bool:
        """Check if joint configuration is within limits"""
        joint_limits = robot_config.get('joint_limits', {})
        if not joint_limits:
            return True

        for i, joint_angle in enumerate(joints):
            if i < len(joint_limits):
                limit = joint_limits[i]
                if joint_angle < limit['min'] or joint_angle > limit['max']:
                    return False

        return True

    def check_collision_at_configuration(self, joints: np.ndarray,
                                       environment_info: Dict[str, Any]) -> bool:
        """Check if robot configuration collides with environment"""
        # Transform robot model to configuration
        robot_links = self.collision_checker.transform_robot_model(joints)

        # Check collisions with environment
        collision_result = self.collision_checker.check_robot_environment_collision(
            robot_links, environment_info['map']
        )

        return collision_result['collision']

class InverseKinematicsSolver:
    """Solver for inverse kinematics problems"""
    def __init__(self):
        # For this example, use a simple Jacobian-based solver
        self.max_iterations = 100
        self.tolerance = 1e-4

    def solve_inverse_kinematics(self, target_pose: Dict[str, Any],
                               robot_config: Dict[str, Any]) -> Optional[np.ndarray]:
        """Solve inverse kinematics for target pose"""
        # Get current joint angles (starting configuration)
        current_joints = robot_config.get('current_joints', np.zeros(6))

        # Use iterative Jacobian method
        for iteration in range(self.max_iterations):
            # Calculate current end-effector pose
            current_pose = self.forward_kinematics(current_joints, robot_config)

            # Calculate error
            position_error = np.array(target_pose['position']) - np.array(current_pose['position'])
            orientation_error = self.calculate_orientation_error(
                target_pose.get('orientation', [0, 0, 0, 1]),
                current_pose.get('orientation', [0, 0, 0, 1])
            )

            # Check if within tolerance
            if np.linalg.norm(position_error) < self.tolerance and np.linalg.norm(orientation_error) < self.tolerance:
                return current_joints

            # Calculate Jacobian
            jacobian = self.calculate_jacobian(current_joints, robot_config)

            # Calculate joint updates
            pose_error = np.concatenate([position_error, orientation_error])
            joint_updates = np.linalg.pinv(jacobian) @ pose_error

            # Update joint angles
            current_joints += 0.1 * joint_updates  # Learning rate

        # If no solution found within iterations, return None
        return None

    def forward_kinematics(self, joints: np.ndarray,
                          robot_config: Dict[str, Any]) -> Dict[str, Any]:
        """Calculate forward kinematics"""
        # Simplified forward kinematics for example
        # In practice, this would implement the full kinematic chain
        x = joints[0] * 0.1  # Simplified mapping
        y = joints[1] * 0.1
        z = joints[2] * 0.1 + 1.0  # Base height

        return {
            'position': [x, y, z],
            'orientation': [0, 0, 0, 1]  # Identity quaternion
        }

    def calculate_jacobian(self, joints: np.ndarray,
                          robot_config: Dict[str, Any]) -> np.ndarray:
        """Calculate geometric Jacobian matrix"""
        # Simplified Jacobian calculation
        # In practice, this would implement the full Jacobian
        jacobian = np.zeros((6, len(joints)))  # 6 DOF (position + orientation) x joint DOF

        # Fill with simplified values
        for i in range(min(6, len(joints))):
            jacobian[i, i] = 1.0

        return jacobian

    def calculate_orientation_error(self, target_quat: List[float],
                                  current_quat: List[float]) -> np.ndarray:
        """Calculate orientation error from quaternions"""
        # Convert to rotation vectors for error calculation
        target_rot = R.from_quat(target_quat).as_rotvec()
        current_rot = R.from_quat(current_quat).as_rotvec()

        return target_rot - current_rot
```

<h2 className="second-heading">
Isaac Integration for Manipulation
</h2>
<div className="underline-class"></div>

<h3 className="third-heading">
- Isaac Manipulation Components
</h3>
<div className="underline-class"></div>

The Isaac ecosystem provides specialized components for manipulation that leverage NVIDIA's hardware acceleration:

```python
# Example: Isaac integration for manipulation
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, PointCloud2
from geometry_msgs.msg import PoseStamped, WrenchStamped
from std_msgs.msg import String
from control_msgs.msg import JointTrajectoryControllerState
import tf2_ros
from tf2_ros import TransformException
import numpy as np
import json

class IsaacManipulationNode(Node):
    def __init__(self):
        super().__init__('manipulation_node')

        # Publishers
        self.joint_command_pub = self.create_publisher(
            JointState, '/joint_commands', 10
        )
        self.gripper_command_pub = self.create_publisher(
            String, '/gripper_commands', 10
        )
        self.manipulation_goal_pub = self.create_publisher(
            PoseStamped, '/manipulation/goal', 10
        )

        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        self.pointcloud_sub = self.create_subscription(
            PointCloud2, '/camera/depth/points', self.pointcloud_callback, 10
        )
        self.gripper_state_sub = self.create_subscription(
            JointState, '/gripper_states', self.gripper_state_callback, 10
        )

        # Initialize manipulation components
        self.perception_system = ManipulationPerceptionSystem()
        self.grasp_planner = GraspPlanner()
        self.trajectory_planner = ManipulationTrajectoryPlanner()

        # Robot state
        self.current_joint_state = None
        self.current_gripper_state = None
        self.robot_config = self.initialize_robot_config()

        # TF buffer for coordinate transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info('Isaac Manipulation node initialized')

    def initialize_robot_config(self) -> Dict[str, Any]:
        """Initialize robot configuration"""
        return {
            'dof_names': ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'],
            'max_aperture': 0.1,  # 10cm max gripper opening
            'joint_limits': [
                {'min': -3.14, 'max': 3.14},  # Joint 1
                {'min': -1.57, 'max': 1.57},  # Joint 2
                {'min': -3.14, 'max': 3.14},  # Joint 3
                {'min': -2.0, 'max': 2.0},    # Joint 4
                {'min': -2.0, 'max': 2.0},    # Joint 5
                {'min': -3.14, 'max': 3.14}   # Joint 6
            ],
            'current_joints': np.zeros(6)
        }

    def joint_state_callback(self, msg: JointState):
        """Handle joint state updates"""
        self.current_joint_state = {
            'position': dict(zip(msg.name, msg.position)),
            'velocity': dict(zip(msg.name, msg.velocity)),
            'effort': dict(zip(msg.name, msg.effort)),
            'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        }

        # Update robot config with current joint positions
        joint_names = self.robot_config['dof_names']
        current_positions = []
        for name in joint_names:
            if name in self.current_joint_state['position']:
                current_positions.append(self.current_joint_state['position'][name])
            else:
                current_positions.append(0.0)

        self.robot_config['current_joints'] = np.array(current_positions)

    def pointcloud_callback(self, msg: PointCloud2):
        """Handle point cloud data for manipulation perception"""
        try:
            # Convert PointCloud2 to numpy array
            import sensor_msgs.point_cloud2 as pc2
            points = []
            for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                points.append([point[0], point[1], point[2]])

            point_cloud = np.array(points)

            # Get camera pose from TF
            try:
                transform = self.tf_buffer.lookup_transform(
                    'base_link', 'camera_link', rclpy.time.Time()
                )
                camera_pose = self.tf_transform_to_matrix(transform)
            except TransformException:
                camera_pose = np.eye(4)  # Default identity if transform not available

            # Process manipulation scene
            scene_info = self.perception_system.perceive_manipulation_scene(
                point_cloud, camera_pose
            )

            # Process detected objects
            for obj in scene_info['objects']:
                self.process_detected_object(obj)

        except Exception as e:
            self.get_logger().error(f'Error processing point cloud: {e}')

    def tf_transform_to_matrix(self, transform) -> np.ndarray:
        """Convert TF transform to 4x4 transformation matrix"""
        t = transform.transform.translation
        r = transform.transform.rotation

        # Convert quaternion to rotation matrix
        rotation = R.from_quat([r.x, r.y, r.z, r.w])
        matrix = np.eye(4)
        matrix[:3, :3] = rotation.as_matrix()
        matrix[0, 3] = t.x
        matrix[1, 3] = t.y
        matrix[2, 3] = t.z

        return matrix

    def gripper_state_callback(self, msg: JointState):
        """Handle gripper state updates"""
        self.current_gripper_state = {
            'position': dict(zip(msg.name, msg.position)),
            'effort': dict(zip(msg.name, msg.effort))
        }

    def process_detected_object(self, obj_info: Dict[str, Any]):
        """Process a detected object for potential manipulation"""
        self.get_logger().info(f'Detected {obj_info["type"]} at {obj_info["center"]}')

        # Plan grasp for the object
        grasp_candidates = self.grasp_planner.plan_grasps(
            obj_info, self.get_gripper_config()
        )

        if grasp_candidates:
            best_grasp = grasp_candidates[0]  # Take highest scoring grasp
            self.get_logger().info(f'Best grasp for {obj_info["type"]}: {best_grasp["score"]:.3f}')

            # Plan manipulation trajectory
            if self.current_joint_state:
                current_pose = self.get_current_end_effector_pose()
                goal_pose = self.get_grasp_pose(obj_info, best_grasp['grasp'])

                trajectory = self.trajectory_planner.plan_manipulation_trajectory(
                    current_pose, goal_pose, obj_info, self.robot_config, {}
                )

                if trajectory['success']:
                    self.execute_trajectory(trajectory['trajectory'])

    def get_gripper_config(self) -> Dict[str, Any]:
        """Get current gripper configuration"""
        return {
            'max_aperture': 0.1,
            'dof_names': ['gripper_left', 'gripper_right'],
            'current_aperture': self.current_gripper_state['position'].get('gripper_left', 0) +
                              self.current_gripper_state['position'].get('gripper_right', 0) if self.current_gripper_state else 0.05
        }

    def get_current_end_effector_pose(self) -> Dict[str, Any]:
        """Get current end-effector pose from forward kinematics"""
        # This would implement forward kinematics
        # For this example, return a placeholder
        return {
            'position': [0.5, 0.0, 1.0],  # Placeholder position
            'orientation': [0, 0, 0, 1]   # Identity quaternion
        }

    def get_grasp_pose(self, object_info: Dict[str, Any],
                      grasp_info: Dict[str, Any]) -> Dict[str, Any]:
        """Get grasp pose from object and grasp information"""
        # Calculate grasp pose based on object center and grasp approach
        object_center = object_info['center']
        approach_vector = np.array(grasp_info['approach'])
        approach_vector = approach_vector / np.linalg.norm(approach_vector)

        # Position grasp point slightly offset from object center
        grasp_position = object_center + approach_vector * 0.05  # 5cm offset

        # Calculate orientation based on approach vector
        z_axis = approach_vector
        if abs(z_axis[2]) < 0.9:
            y_axis = np.array([0, 0, 1])
        else:
            y_axis = np.array([0, 1, 0])

        x_axis = np.cross(y_axis, z_axis)
        x_axis = x_axis / np.linalg.norm(x_axis)
        y_axis = np.cross(z_axis, x_axis)
        y_axis = y_axis / np.linalg.norm(y_axis)

        # Convert to quaternion
        rotation_matrix = np.column_stack([x_axis, y_axis, z_axis])
        quaternion = R.from_matrix(rotation_matrix).as_quat()

        return {
            'position': grasp_position.tolist(),
            'orientation': quaternion.tolist()
        }

    def execute_trajectory(self, trajectory: List[np.ndarray]):
        """Execute the planned trajectory"""
        self.get_logger().info(f'Executing trajectory with {len(trajectory)} waypoints')

        for i, joints in enumerate(trajectory):
            # Send joint command
            joint_msg = JointState()
            joint_msg.name = self.robot_config['dof_names']
            joint_msg.position = joints.tolist()
            joint_msg.header.stamp = self.get_clock().now().to_msg()

            self.joint_command_pub.publish(joint_msg)

            # Wait briefly between waypoints
            time.sleep(0.1)

        self.get_logger().info('Trajectory execution completed')

class IsaacGripperControlNode(Node):
    """Isaac node for gripper control"""
    def __init__(self):
        super().__init__('gripper_control_node')

        # Publishers
        self.gripper_command_pub = self.create_publisher(
            JointState, '/gripper_commands', 10
        )

        # Subscribers
        self.gripper_state_sub = self.create_subscription(
            JointState, '/gripper_states', self.gripper_state_callback, 10
        )

        self.gripper_command_sub = self.create_subscription(
            String, '/gripper_commands', self.gripper_command_callback, 10
        )

        # Gripper state
        self.current_gripper_position = 0.0
        self.gripper_limits = {'min': 0.0, 'max': 0.1}  # 0 to 10cm

        self.get_logger().info('Isaac Gripper Control node initialized')

    def gripper_state_callback(self, msg: JointState):
        """Update gripper state"""
        if msg.name and msg.position:
            # Assume first joint is the gripper position
            self.current_gripper_position = msg.position[0]

    def gripper_command_callback(self, msg: String):
        """Handle gripper commands"""
        try:
            command_data = json.loads(msg.data)
            command_type = command_data.get('command')

            if command_type == 'grasp':
                self.execute_grasp(command_data)
            elif command_type == 'release':
                self.execute_release()
            elif command_type == 'position':
                target_position = command_data.get('position', 0.05)
                self.move_to_position(target_position)
        except Exception as e:
            self.get_logger().error(f'Error processing gripper command: {e}')

    def execute_grasp(self, command_data: Dict[str, Any]):
        """Execute grasp action"""
        object_info = command_data.get('object', {})
        object_size = object_info.get('dimensions', [0.05, 0.05, 0.05])
        required_aperture = min(object_size) * 1.2  # Slightly larger than object

        # Calculate appropriate grasp position
        target_position = max(self.gripper_limits['min'],
                            self.gripper_limits['max'] - required_aperture)

        self.move_to_position(target_position)

    def execute_release(self):
        """Execute release action"""
        self.move_to_position(self.gripper_limits['max'])  # Fully open

    def move_to_position(self, target_position: float):
        """Move gripper to target position"""
        # Clamp to limits
        target_position = max(self.gripper_limits['min'],
                             min(self.gripper_limits['max'], target_position))

        # Create and publish command
        command_msg = JointState()
        command_msg.name = ['gripper_left', 'gripper_right']
        command_msg.position = [target_position/2, target_position/2]  # Symmetric gripper
        command_msg.header.stamp = self.get_clock().now().to_msg()

        self.gripper_command_pub.publish(command_msg)
        self.get_logger().info(f'Gripper commanded to position: {target_position:.3f}')
```

<h3 className="third-heading">
- Manipulation Safety and Control
</h3>
<div className="underline-class"></div>

Safety is paramount in manipulation systems, especially for humanoid robots operating near humans:

```python
# Example: Manipulation safety and control system
class ManipulationSafetySystem:
    def __init__(self):
        self.safety_thresholds = {
            'max_force': 50.0,      # Newtons
            'max_torque': 20.0,     # Nm
            'max_velocity': 0.5,    # m/s
            'max_acceleration': 2.0, # m/s^2
            'min_distance_human': 0.5 # meters
        }

        self.emergency_stop = False
        self.safety_lock = threading.RLock()
        self.force_sensors_enabled = True
        self.torque_limits_enabled = True

    def validate_manipulation_command(self, joint_commands: np.ndarray,
                                    current_state: Dict[str, Any],
                                    object_info: Dict[str, Any]) -> Dict[str, Any]:
        """Validate manipulation command for safety"""
        with self.safety_lock:
            if self.emergency_stop:
                return {
                    'safe': False,
                    'reason': 'Emergency stop active',
                    'recommended_action': 'Stop immediately'
                }

            # Check joint limits
            joint_limit_check = self.check_joint_limits(joint_commands)
            if not joint_limit_check['safe']:
                return joint_limit_check

            # Check velocity limits
            velocity_check = self.check_velocity_limits(joint_commands, current_state)
            if not velocity_check['safe']:
                return velocity_check

            # Check force/torque limits if sensors available
            if self.force_sensors_enabled:
                force_check = self.check_force_limits(current_state)
                if not force_check['safe']:
                    return force_check

            # Check for safe object handling
            object_check = self.check_object_safety(object_info, current_state)
            if not object_check['safe']:
                return object_check

            return {
                'safe': True,
                'reason': 'All safety checks passed',
                'recommended_action': 'Proceed with manipulation'
            }

    def check_joint_limits(self, joint_commands: np.ndarray) -> Dict[str, Any]:
        """Check if joint commands are within limits"""
        joint_limits = self.get_robot_joint_limits()

        for i, command in enumerate(joint_commands):
            if i < len(joint_limits):
                limit = joint_limits[i]
                if command < limit['min'] or command > limit['max']:
                    return {
                        'safe': False,
                        'reason': f'Joint {i} command {command:.3f} exceeds limits [{limit["min"]:.3f}, {limit["max"]:.3f}]',
                        'recommended_action': f'Command joint {i} within limits'
                    }

        return {'safe': True}

    def check_velocity_limits(self, joint_commands: np.ndarray,
                            current_state: Dict[str, Any]) -> Dict[str, Any]:
        """Check if velocity commands are within safe limits"""
        if 'position' not in current_state or 'timestamp' not in current_state:
            return {'safe': True}

        # Calculate joint velocities
        dt = 0.01  # Assuming 100Hz control loop
        current_positions = list(current_state['position'].values())

        if len(current_positions) == len(joint_commands):
            velocities = (joint_commands - np.array(current_positions)) / dt

            max_velocity = max(abs(v) for v in velocities)
            if max_velocity > self.safety_thresholds['max_velocity']:
                return {
                    'safe': False,
                    'reason': f'Max joint velocity {max_velocity:.3f} exceeds limit {self.safety_thresholds["max_velocity"]:.3f}',
                    'recommended_action': f'Reduce velocity command to <= {self.safety_thresholds["max_velocity"]:.3f}'
                }

        return {'safe': True}

    def check_force_limits(self, current_state: Dict[str, Any]) -> Dict[str, Any]:
        """Check if forces are within safe limits"""
        if 'wrench' in current_state:
            wrench = current_state['wrench']
            force_magnitude = np.linalg.norm([wrench['force_x'], wrench['force_y'], wrench['force_z']])

            if force_magnitude > self.safety_thresholds['max_force']:
                return {
                    'safe': False,
                    'reason': f'Force magnitude {force_magnitude:.3f}N exceeds limit {self.safety_thresholds["max_force"]:.3f}N',
                    'recommended_action': 'Reduce applied force or stop manipulation'
                }

        return {'safe': True}

    def check_object_safety(self, object_info: Dict[str, Any],
                          current_state: Dict[str, Any]) -> Dict[str, Any]:
        """Check if object manipulation is safe"""
        # Check object properties
        object_mass = object_info.get('mass', 0.5)  # Default 0.5kg
        max_payload = self.get_robot_max_payload()

        if object_mass > max_payload:
            return {
                'safe': False,
                'reason': f'Object mass {object_mass:.3f}kg exceeds robot payload limit {max_payload:.3f}kg',
                'recommended_action': 'Select lighter object or use appropriate manipulator'
            }

        # Check object fragility (if known)
        object_fragile = object_info.get('fragile', False)
        if object_fragile:
            # Recommend gentle manipulation
            pass  # For now, just log that object is fragile

        return {'safe': True}

    def get_robot_joint_limits(self) -> List[Dict[str, float]]:
        """Get robot joint limits"""
        # This would interface with robot's URDF or configuration
        # For this example, return default limits
        return [
            {'min': -3.14, 'max': 3.14},
            {'min': -1.57, 'max': 1.57},
            {'min': -3.14, 'max': 3.14},
            {'min': -2.0, 'max': 2.0},
            {'min': -2.0, 'max': 2.0},
            {'min': -3.14, 'max': 3.14}
        ]

    def get_robot_max_payload(self) -> float:
        """Get robot maximum payload capacity"""
        # This would interface with robot specifications
        return 5.0  # 5kg default

    def trigger_emergency_stop(self):
        """Trigger emergency stop for safety"""
        with self.safety_lock:
            self.emergency_stop = True

    def clear_emergency_stop(self):
        """Clear emergency stop condition"""
        with self.safety_lock:
            self.emergency_stop = False

class ManipulationController:
    """Controller for executing manipulation tasks"""
    def __init__(self):
        self.safety_system = ManipulationSafetySystem()
        self.control_mode = 'position'  # position, velocity, or effort
        self.gains = self.initialize_gains()

    def initialize_gains(self) -> Dict[str, float]:
        """Initialize control gains"""
        return {
            'position_kp': 5.0,   # Proportional gain for position
            'position_ki': 0.1,   # Integral gain for position
            'position_kd': 0.5,   # Derivative gain for position
            'velocity_kp': 2.0,   # Proportional gain for velocity
            'velocity_ki': 0.05,  # Integral gain for velocity
            'velocity_kd': 0.2    # Derivative gain for velocity
        }

    def execute_grasp(self, object_info: Dict[str, Any],
                     grasp_pose: Dict[str, Any],
                     robot_config: Dict[str, Any]) -> Dict[str, Any]:
        """Execute grasp action with safety monitoring"""
        # Validate grasp parameters
        validation_result = self.safety_system.validate_manipulation_command(
            np.zeros(len(robot_config['current_joints'])),  # Placeholder
            {'position': {}, 'timestamp': time.time()},
            object_info
        )

        if not validation_result['safe']:
            return {
                'success': False,
                'error': validation_result['reason'],
                'safety_violation': True
            }

        # Plan and execute approach trajectory
        approach_trajectory = self.plan_approach_trajectory(grasp_pose, robot_config)
        approach_result = self.execute_trajectory(approach_trajectory, robot_config)

        if not approach_result['success']:
            return approach_result

        # Execute grasp
        grasp_result = self.execute_precise_grasp(grasp_pose, object_info)

        if not grasp_result['success']:
            return grasp_result

        # Verify grasp success
        grasp_verified = self.verify_grasp_success(object_info)

        return {
            'success': grasp_verified,
            'grasp_verified': grasp_verified,
            'execution_time': time.time() - approach_result.get('start_time', time.time())
        }

    def plan_approach_trajectory(self, grasp_pose: Dict[str, Any],
                               robot_config: Dict[str, Any]) -> List[np.ndarray]:
        """Plan approach trajectory to grasp pose"""
        # This would use the trajectory planner
        # For this example, return a simple trajectory
        current_pose = self.get_current_end_effector_pose(robot_config)

        # Plan path from current to grasp pose
        trajectory = []
        num_waypoints = 10

        for i in range(num_waypoints + 1):
            ratio = i / num_waypoints
            pos = (1 - ratio) * np.array(current_pose['position']) + ratio * np.array(grasp_pose['position'])
            traj_point = np.array(pos.tolist())
            trajectory.append(traj_point)

        return trajectory

    def execute_trajectory(self, trajectory: List[np.ndarray],
                          robot_config: Dict[str, Any]) -> Dict[str, Any]:
        """Execute a joint space trajectory"""
        start_time = time.time()

        for i, waypoint in enumerate(trajectory):
            # Validate each waypoint
            validation = self.safety_system.validate_manipulation_command(
                waypoint,
                {'position': dict(enumerate(robot_config['current_joints'])), 'timestamp': time.time()},
                {}
            )

            if not validation['safe']:
                return {
                    'success': False,
                    'error': f'Safety violation at waypoint {i}: {validation["reason"]}',
                    'waypoint': i,
                    'start_time': start_time
                }

            # Send command to robot (in practice, this would interface with robot controller)
            self.send_joint_command(waypoint)

            # Wait for execution
            time.sleep(0.01)  # 100Hz control loop

        return {
            'success': True,
            'execution_time': time.time() - start_time,
            'waypoints_executed': len(trajectory),
            'start_time': start_time
        }

    def execute_precise_grasp(self, grasp_pose: Dict[str, Any],
                            object_info: Dict[str, Any]) -> Dict[str, Any]:
        """Execute precise grasp at specified pose"""
        try:
            # Move to grasp pose
            self.move_to_pose(grasp_pose)

            # Close gripper
            grasp_force = self.calculate_grasp_force(object_info)
            self.close_gripper_with_force(grasp_force)

            # Wait for grasp completion
            time.sleep(0.5)

            return {'success': True}
        except Exception as e:
            return {
                'success': False,
                'error': str(e)
            }

    def calculate_grasp_force(self, object_info: Dict[str, Any]) -> float:
        """Calculate appropriate grasp force based on object properties"""
        object_mass = object_info.get('mass', 0.5)
        object_material = object_info.get('material', 'unknown')

        # Base force calculation
        base_force = object_mass * 9.81 * 1.5  # Weight * safety factor

        # Adjust for material
        if object_material == 'fragile':
            base_force *= 0.5  # Gentle grasp for fragile objects
        elif object_material == 'metal':
            base_force *= 1.2  # Stronger grasp for slippery metal

        return min(base_force, 20.0)  # Cap at 20N

    def verify_grasp_success(self, object_info: Dict[str, Any]) -> bool:
        """Verify that grasp was successful"""
        # This would use tactile sensors, force sensors, or visual confirmation
        # For this example, return True (in practice, implement actual verification)
        return True

    def get_current_end_effector_pose(self, robot_config: Dict[str, Any]) -> Dict[str, Any]:
        """Get current end-effector pose from forward kinematics"""
        # This would implement forward kinematics
        # For this example, return placeholder
        return {
            'position': [0.5, 0.0, 1.0],
            'orientation': [0, 0, 0, 1]
        }

    def send_joint_command(self, joint_angles: np.ndarray):
        """Send joint position command to robot"""
        # This would interface with robot controller
        pass

    def move_to_pose(self, pose: Dict[str, Any]):
        """Move end-effector to specified pose"""
        # This would solve inverse kinematics and send joint commands
        pass

    def close_gripper_with_force(self, force: float):
        """Close gripper with specified force"""
        # This would interface with gripper controller
        pass
```

<h2 className="second-heading">
Performance Optimization and Real-Time Considerations
</h2>
<div className="underline-class"></div>

<h3 className="third-heading">
- Real-Time Manipulation Performance
</h3>
<div className="underline-class"></div>

Manipulation systems must meet real-time constraints while maintaining precision and safety:

```python
# Example: Real-time manipulation performance optimization
class RealTimeManipulationOptimizer:
    def __init__(self):
        self.timing_requirements = {
            'perception_update': 0.1,      # 10Hz
            'grasp_planning': 0.5,        # 2Hz
            'trajectory_planning': 0.05,  # 20Hz
            'control_update': 0.01,       # 100Hz
            'safety_check': 0.005        # 200Hz
        }

        self.performance_monitor = PerformanceMonitor()
        self.adaptive_resolution = True
        self.multi_threading_enabled = True

    def optimize_perception_performance(self, point_cloud: np.ndarray) -> Dict[str, Any]:
        """Optimize perception for real-time performance"""
        start_time = time.time()

        # Adaptive resolution based on computational load
        if self.performance_monitor.get_avg_time('perception') > 0.08:  # 80ms budget
            # Reduce point cloud resolution
            point_cloud = self.downsample_point_cloud(point_cloud, factor=0.5)

        # Perform perception
        result = self.perform_perception_optimized(point_cloud)

        execution_time = time.time() - start_time
        self.performance_monitor.record_time('perception', execution_time)

        return result

    def downsample_point_cloud(self, point_cloud: np.ndarray, factor: float) -> np.ndarray:
        """Downsample point cloud for faster processing"""
        step = max(1, int(1.0 / factor))
        return point_cloud[::step]

    def perform_perception_optimized(self, point_cloud: np.ndarray) -> Dict[str, Any]:
        """Optimized perception using efficient algorithms"""
        # Use efficient clustering algorithms
        # Implement spatial indexing for faster queries
        # Use approximate methods where precision allows

        # Placeholder for optimized implementation
        return {
            'objects': [],
            'timestamp': time.time()
        }

    def optimize_grasp_planning_performance(self, object_info: Dict[str, Any],
                                          robot_config: Dict[str, Any]) -> List[Dict[str, Any]]:
        """Optimize grasp planning for real-time performance"""
        start_time = time.time()

        # Use hierarchical approach: coarse-to-fine planning
        coarse_grasps = self.generate_coarse_grasps(object_info)

        if coarse_grasps:
            # Evaluate only top candidates
            top_candidates = coarse_grasps[:5]  # Limit to top 5 candidates
            refined_grasps = self.evaluate_grasps_efficiently(top_candidates, object_info, robot_config)
        else:
            refined_grasps = []

        execution_time = time.time() - start_time
        self.performance_monitor.record_time('grasp_planning', execution_time)

        return refined_grasps

    def generate_coarse_grasps(self, object_info: Dict[str, Any]) -> List[Dict[str, Any]]:
        """Generate coarse grasp candidates quickly"""
        # Use geometric heuristics for fast grasp generation
        # Avoid expensive physics simulations
        # Focus on geometric feasibility first

        object_type = object_info['type']
        dimensions = object_info['dimensions']

        grasps = []

        # Generate simple grasp types based on object type
        if object_type == 'box':
            # Face and edge grasps
            for axis in range(3):
                for sign in [-1, 1]:
                    approach = np.zeros(3)
                    approach[axis] = sign
                    grasps.append({
                        'approach': approach.tolist(),
                        'type': 'face',
                        'quality': 0.6
                    })
        elif object_type in ['cup', 'bottle']:
            # Cylindrical grasps
            for angle in np.linspace(0, 2*np.pi, 4):
                approach = [np.cos(angle), np.sin(angle), 0]
                grasps.append({
                    'approach': approach,
                    'type': 'cylindrical',
                    'quality': 0.7
                })

        return grasps

    def evaluate_grasps_efficiently(self, grasp_candidates: List[Dict[str, Any]],
                                  object_info: Dict[str, Any],
                                  robot_config: Dict[str, Any]) -> List[Dict[str, Any]]:
        """Efficiently evaluate grasp candidates"""
        evaluated = []

        for grasp in grasp_candidates:
            # Quick feasibility check
            if self.quick_feasibility_check(grasp, object_info, robot_config):
                # Detailed evaluation
                evaluation = self.evaluate_grasp_detailed(grasp, object_info, robot_config)
                evaluated.append({
                    'grasp': grasp,
                    'evaluation': evaluation,
                    'score': evaluation['stability'] * 0.6 + evaluation['quality'] * 0.4
                })

        # Sort by score
        evaluated.sort(key=lambda x: x['score'], reverse=True)
        return evaluated

    def quick_feasibility_check(self, grasp: Dict[str, Any],
                              object_info: Dict[str, Any],
                              robot_config: Dict[str, Any]) -> bool:
        """Quick feasibility check for grasp"""
        # Check basic geometric constraints
        grasp_width = grasp.get('grasp_width', 0.05)
        object_dims = object_info['dimensions']
        min_object_dim = min(object_dims)
        max_object_dim = max(object_dims)

        # Basic size compatibility
        if grasp_width > max_object_dim * 1.5 or grasp_width < min_object_dim * 0.3:
            return False

        # Check robot aperture
        max_aperture = robot_config.get('max_aperture', 0.1)
        if grasp_width > max_aperture:
            return False

        return True

    def evaluate_grasp_detailed(self, grasp: Dict[str, Any],
                              object_info: Dict[str, Any],
                              robot_config: Dict[str, Any]) -> Dict[str, Any]:
        """Detailed grasp evaluation"""
        # This would perform more detailed analysis
        # For this example, return a simplified evaluation
        return {
            'feasible': True,
            'stability': 0.7,
            'quality': grasp.get('quality', 0.6),
            'force_closure': True
        }

class PerformanceMonitor:
    """Monitor and optimize manipulation system performance"""
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
            'perception': 0.1,      # 100ms
            'grasp_planning': 0.5,  # 500ms
            'trajectory_planning': 0.05,  # 50ms
            'control': 0.01,        # 10ms
            'safety': 0.005        # 5ms
        }
        return budgets.get(component, 0.1)

    def get_performance_summary(self) -> Dict[str, Any]:
        """Get performance summary"""
        summary = {}
        for component, times in self.component_times.items():
            if times:
                summary[component] = {
                    'avg_time': np.mean(times),
                    'min_time': min(times),
                    'max_time': max(times),
                    'std_dev': np.std(times),
                    'utilization': (np.mean(times) / self.get_time_budget(component)) if self.get_time_budget(component) > 0 else 0
                }
        return summary
```

<div className="border-line"></div>
---
<h2 className="second-heading">
 Summary
</h2>
<div className="underline-class"></div>

The robotic manipulation system for humanoid robots represents a sophisticated integration of perception, planning, control, and safety technologies. The system must handle the unique challenges of dexterous manipulation, including complex kinematics, grasp planning, and the need to operate safely in human-populated environments.

Key components of the manipulation system include:

1. • **Perception**: 3D object detection and pose estimation systems that accurately identify objects in the environment and their spatial relationships.

2. • **Grasp Planning**: Sophisticated grasp synthesis and evaluation systems that determine optimal ways to grasp objects based on their geometry, material properties, and the robot's capabilities.

3. • **Trajectory Planning**: Advanced path planning systems that create safe, collision-free trajectories for manipulation tasks while considering robot kinematics and environmental constraints.

4. • **Isaac Integration**: Specialized components that leverage the Isaac ecosystem for enhanced manipulation capabilities.

5. • **Safety Systems**: Comprehensive safety and control systems that ensure safe operation around humans and objects.

6. • **Performance Optimization**: Real-time optimization techniques that ensure manipulation systems meet timing constraints while maintaining precision.

The success of the manipulation system depends on careful integration of these components, with particular attention to the unique requirements of humanoid robots, including their complex kinematics, dexterous capabilities, and the need for safe human interaction.

<div className="border-line"></div>
---
<h2 className="second-heading">
 Exercises
</h2>
<div className="underline-class"></div>

1. • Implement a 3D object detection system using point cloud data from RGB-D sensors
2. • Create a grasp planner that considers object properties and robot capabilities
3. • Design a trajectory planning system for complex manipulation tasks
4. • Build a safety validation system for manipulation performance
5. • Optimize manipulation algorithms for real-time performance on your robot platform

<div className="border-line"></div>
---
<h2 className="second-heading">
 Further Reading
</h2>
<div className="underline-class"></div>

- • "Handbook of Robotics" by Siciliano and Khatib
- • "Robotics: Control, Sensing, Vision, and Intelligence" by Fu et al.
- • "Grasping in Robotics" by Bicchi and Kumar
- • "Planning Algorithms" by LaValle
- • NVIDIA Isaac documentation on manipulation

</div>