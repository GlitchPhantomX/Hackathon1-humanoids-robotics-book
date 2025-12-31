# Robotics Reasoning Core Skill

---
name: robotics_reasoning_core
description: Reusable reasoning engine for Physical AI and humanoid robotics concepts
version: 1.0.0
project: physical-ai-robotics-textbook
parameters:
  - name: topic
    description: Robotics topic to explain (ros2, sensors, control, simulation, kinematics)
    required: false
  - name: depth
    description: Explanation depth (beginner, intermediate, advanced)
    required: false
    default: intermediate
  - name: include_code
    description: Include code examples
    required: false
    default: true
  - name: include_math
    description: Include mathematical formulations
    required: false
    default: true
---

## Purpose

Create a reusable reasoning engine that:
- **Explains core robotics concepts** with technical depth
- **Adapts to learner level** (beginner → advanced)
- **Abstracts across chapters** - no chapter-specific assumptions
- **Provides multi-modal explanations** - text, code, math
- **Maps concept relationships** - prerequisites and dependencies

## Knowledge Domains

The skill covers these core domains:

1. **ROS 2 Architecture** - Nodes, topics, services, actions
2. **Humanoid Sensors** - IMU, force/torque, vision, proprioception
3. **Control Loops** - PID, MPC, whole-body control
4. **Simulation** - Gazebo, Isaac Sim, physics engines
5. **Kinematics** - Forward/inverse kinematics, Jacobians
6. **Vision-Language-Action (VLA)** - Embodied AI, policy learning

## Core Principles

**MUST FOLLOW:**
1. **Explanation-Focused**: Clarity over brevity, educational over terse
2. **Hardware-Agnostic**: Generic principles, not specific robots
3. **Depth-Adaptive**: Adjust complexity to learner level
4. **Chapter-Neutral**: No hardcoded chapter references
5. **Backend Logic**: Pure reasoning, no UI/rendering concerns
6. **Traceable Sources**: Cite standard robotics references

## Instructions

### Step 1: Define Knowledge Base Structure

Create a structured knowledge representation:

```python
# app/ai/robotics_knowledge.py

from dataclasses import dataclass
from typing import List, Dict, Optional
from enum import Enum

class ConceptDepth(Enum):
    """Learning depth levels"""
    BEGINNER = "beginner"
    INTERMEDIATE = "intermediate"
    ADVANCED = "advanced"

class ConceptDomain(Enum):
    """Robotics knowledge domains"""
    ROS2 = "ros2"
    SENSORS = "sensors"
    CONTROL = "control"
    SIMULATION = "simulation"
    KINEMATICS = "kinematics"
    VLA = "vla"

@dataclass
class RoboticsConcept:
    """Represents a robotics concept with metadata"""
    id: str
    name: str
    domain: ConceptDomain
    description: str
    prerequisites: List[str]  # IDs of prerequisite concepts
    related_concepts: List[str]
    key_equations: List[Dict]  # Mathematical formulations
    code_examples: List[Dict]  # Code snippets
    practical_applications: List[str]
    common_pitfalls: List[str]
    references: List[str]

class RoboticsKnowledgeBase:
    """Central knowledge repository for robotics concepts"""

    def __init__(self):
        self.concepts = self._initialize_concepts()

    def get_concept(self, concept_id: str) -> Optional[RoboticsConcept]:
        """Retrieve a concept by ID"""
        return self.concepts.get(concept_id)

    def get_concepts_by_domain(self, domain: ConceptDomain) -> List[RoboticsConcept]:
        """Get all concepts in a domain"""
        return [c for c in self.concepts.values() if c.domain == domain]

    def get_prerequisites(self, concept_id: str) -> List[RoboticsConcept]:
        """Get prerequisite concepts"""
        concept = self.get_concept(concept_id)
        if not concept:
            return []
        return [self.concepts[pid] for pid in concept.prerequisites if pid in self.concepts]

    def _initialize_concepts(self) -> Dict[str, RoboticsConcept]:
        """Initialize the knowledge base with core concepts"""
        concepts = {}

        # ROS 2 Concepts
        concepts["ros2_nodes"] = RoboticsConcept(
            id="ros2_nodes",
            name="ROS 2 Nodes",
            domain=ConceptDomain.ROS2,
            description="Fundamental units of computation in ROS 2. Each node is a process that performs a specific task.",
            prerequisites=[],
            related_concepts=["ros2_topics", "ros2_services", "ros2_parameters"],
            key_equations=[],
            code_examples=[
                {
                    "language": "python",
                    "title": "Basic ROS 2 Node",
                    "code": """
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')
        self.get_logger().info('Node started')

def main():
    rclpy.init()
    node = MinimalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
"""
                }
            ],
            practical_applications=[
                "Sensor data processing nodes",
                "Control algorithm nodes",
                "Visualization and monitoring nodes"
            ],
            common_pitfalls=[
                "Not calling rclpy.init() before creating nodes",
                "Forgetting to destroy nodes and shutdown",
                "Blocking the executor thread with long-running operations"
            ],
            references=[
                "ROS 2 Design - http://design.ros2.org/",
                "ROS 2 Concepts - https://docs.ros.org/en/rolling/Concepts.html"
            ]
        )

        concepts["ros2_topics"] = RoboticsConcept(
            id="ros2_topics",
            name="ROS 2 Topics",
            domain=ConceptDomain.ROS2,
            description="Named buses over which nodes exchange messages using publish-subscribe pattern.",
            prerequisites=["ros2_nodes"],
            related_concepts=["ros2_messages", "ros2_qos"],
            key_equations=[],
            code_examples=[
                {
                    "language": "python",
                    "title": "Publisher and Subscriber",
                    "code": """
from rclpy.node import Node
from std_msgs.msg import String

class Publisher(Node):
    def __init__(self):
        super().__init__('publisher')
        self.pub = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello ROS 2'
        self.pub.publish(msg)

class Subscriber(Node):
    def __init__(self):
        super().__init__('subscriber')
        self.sub = self.create_subscription(
            String, 'topic', self.callback, 10)

    def callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')
"""
                }
            ],
            practical_applications=[
                "Sensor data streaming (e.g., /camera/image_raw)",
                "Robot state publishing (e.g., /joint_states)",
                "Command velocity for mobile robots (e.g., /cmd_vel)"
            ],
            common_pitfalls=[
                "Mismatched message types between publisher and subscriber",
                "Incorrect QoS settings causing message loss",
                "Publishing too fast without considering network bandwidth"
            ],
            references=[
                "ROS 2 Topics Tutorial - https://docs.ros.org/en/rolling/Tutorials.html"
            ]
        )

        # Sensor Concepts
        concepts["imu_sensor"] = RoboticsConcept(
            id="imu_sensor",
            name="Inertial Measurement Unit (IMU)",
            domain=ConceptDomain.SENSORS,
            description="Sensor that measures specific force, angular velocity, and sometimes magnetic field using accelerometers, gyroscopes, and magnetometers.",
            prerequisites=[],
            related_concepts=["sensor_fusion", "orientation_estimation"],
            key_equations=[
                {
                    "name": "Angular Velocity Integration",
                    "latex": r"\theta(t) = \theta_0 + \int_0^t \omega(\tau) d\tau",
                    "description": "Orientation from gyroscope angular velocity"
                },
                {
                    "name": "Accelerometer Gravity Vector",
                    "latex": r"\vec{g} = [a_x, a_y, a_z]^T",
                    "description": "Acceleration vector in sensor frame"
                }
            ],
            code_examples=[
                {
                    "language": "python",
                    "title": "Reading IMU Data in ROS 2",
                    "code": """
from sensor_msgs.msg import Imu

class IMUProcessor(Node):
    def __init__(self):
        super().__init__('imu_processor')
        self.sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)

    def imu_callback(self, msg: Imu):
        # Extract orientation (quaternion)
        quat = msg.orientation

        # Extract angular velocity
        omega = msg.angular_velocity

        # Extract linear acceleration
        accel = msg.linear_acceleration

        self.get_logger().info(
            f'Orientation: [{quat.x:.3f}, {quat.y:.3f}, '
            f'{quat.z:.3f}, {quat.w:.3f}]'
        )
"""
                }
            ],
            practical_applications=[
                "Robot balance and stabilization",
                "Orientation estimation for drones and humanoids",
                "Dead reckoning for navigation"
            ],
            common_pitfalls=[
                "Gyroscope drift over time (requires sensor fusion)",
                "Accelerometer noise in dynamic motion",
                "Not calibrating IMU before use",
                "Magnetic interference affecting magnetometer"
            ],
            references=[
                "IMU Sensor Fusion - Madgwick Filter",
                "Complementary Filter for IMU"
            ]
        )

        # Control Concepts
        concepts["pid_control"] = RoboticsConcept(
            id="pid_control",
            name="PID Control",
            domain=ConceptDomain.CONTROL,
            description="Proportional-Integral-Derivative controller - fundamental feedback control algorithm.",
            prerequisites=[],
            related_concepts=["control_loop", "system_dynamics"],
            key_equations=[
                {
                    "name": "PID Control Law",
                    "latex": r"u(t) = K_p e(t) + K_i \int_0^t e(\tau)d\tau + K_d \frac{de(t)}{dt}",
                    "description": "Control output based on error"
                },
                {
                    "name": "Discrete PID",
                    "latex": r"u[k] = K_p e[k] + K_i \sum_{i=0}^k e[i]\Delta t + K_d \frac{e[k] - e[k-1]}{\Delta t}",
                    "description": "Discretized for digital implementation"
                }
            ],
            code_examples=[
                {
                    "language": "python",
                    "title": "PID Controller Implementation",
                    "code": """
class PIDController:
    def __init__(self, kp: float, ki: float, kd: float, dt: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt

        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, setpoint: float, measurement: float) -> float:
        # Calculate error
        error = setpoint - measurement

        # Proportional term
        p_term = self.kp * error

        # Integral term (with anti-windup)
        self.integral += error * self.dt
        self.integral = max(-10, min(10, self.integral))  # Clamp
        i_term = self.ki * self.integral

        # Derivative term
        derivative = (error - self.prev_error) / self.dt
        d_term = self.kd * derivative

        # Update state
        self.prev_error = error

        # Compute control output
        output = p_term + i_term + d_term
        return output

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
"""
                }
            ],
            practical_applications=[
                "Joint position control in robot arms",
                "Speed control for mobile robots",
                "Temperature control in 3D printers",
                "Altitude hold for drones"
            ],
            common_pitfalls=[
                "Integral windup in saturated systems",
                "Derivative kick on setpoint changes",
                "Not tuning gains systematically (Ziegler-Nichols, etc.)",
                "Ignoring system dynamics and delays"
            ],
            references=[
                "Control Systems Engineering - Nise",
                "PID Control Theory - Åström & Hägglund"
            ]
        )

        # Kinematics Concepts
        concepts["forward_kinematics"] = RoboticsConcept(
            id="forward_kinematics",
            name="Forward Kinematics",
            domain=ConceptDomain.KINEMATICS,
            description="Computing end-effector position and orientation from joint angles.",
            prerequisites=[],
            related_concepts=["inverse_kinematics", "jacobian", "dh_parameters"],
            key_equations=[
                {
                    "name": "Homogeneous Transformation",
                    "latex": r"T = \begin{bmatrix} R & p \\ 0 & 1 \end{bmatrix}",
                    "description": "4x4 matrix encoding rotation R and position p"
                },
                {
                    "name": "Chain Rule",
                    "latex": r"T_{0,n} = T_{0,1} \cdot T_{1,2} \cdot ... \cdot T_{n-1,n}",
                    "description": "Multiply transforms along kinematic chain"
                }
            ],
            code_examples=[
                {
                    "language": "python",
                    "title": "2-Link Planar Forward Kinematics",
                    "code": """
import numpy as np

def forward_kinematics_2link(theta1: float, theta2: float,
                              l1: float, l2: float) -> np.ndarray:
    \"\"\"
    Compute end-effector position for 2-link planar arm.

    Args:
        theta1: Joint 1 angle (radians)
        theta2: Joint 2 angle (radians)
        l1: Link 1 length
        l2: Link 2 length

    Returns:
        [x, y] position of end-effector
    \"\"\"
    x = l1 * np.cos(theta1) + l2 * np.cos(theta1 + theta2)
    y = l1 * np.sin(theta1) + l2 * np.sin(theta1 + theta2)

    return np.array([x, y])

# Example usage
theta1 = np.pi / 4  # 45 degrees
theta2 = np.pi / 3  # 60 degrees
l1, l2 = 1.0, 0.8

pos = forward_kinematics_2link(theta1, theta2, l1, l2)
print(f"End-effector position: x={pos[0]:.3f}, y={pos[1]:.3f}")
"""
                }
            ],
            practical_applications=[
                "Robot arm endpoint tracking",
                "Collision detection",
                "Workspace analysis",
                "Teleoperation visualization"
            ],
            common_pitfalls=[
                "Incorrect transformation order (not commutative)",
                "Mixing degrees and radians",
                "Not handling singularities",
                "Ignoring joint limits"
            ],
            references=[
                "Introduction to Robotics - Craig",
                "Modern Robotics - Lynch & Park"
            ]
        )

        concepts["inverse_kinematics"] = RoboticsConcept(
            id="inverse_kinematics",
            name="Inverse Kinematics",
            domain=ConceptDomain.KINEMATICS,
            description="Computing joint angles required to reach a desired end-effector pose.",
            prerequisites=["forward_kinematics"],
            related_concepts=["jacobian", "numerical_ik", "analytical_ik"],
            key_equations=[
                {
                    "name": "Jacobian Pseudoinverse Method",
                    "latex": r"\Delta\theta = J^{\dagger} \Delta x",
                    "description": "Iterative IK using Jacobian pseudoinverse"
                },
                {
                    "name": "Damped Least Squares",
                    "latex": r"\Delta\theta = J^T(JJ^T + \lambda^2 I)^{-1} \Delta x",
                    "description": "Numerically stable near singularities"
                }
            ],
            code_examples=[
                {
                    "language": "python",
                    "title": "Numerical IK with Jacobian",
                    "code": """
import numpy as np

def jacobian_2link(theta1: float, theta2: float,
                   l1: float, l2: float) -> np.ndarray:
    \"\"\"Compute Jacobian for 2-link planar arm\"\"\"
    J = np.array([
        [-l1*np.sin(theta1) - l2*np.sin(theta1+theta2),
         -l2*np.sin(theta1+theta2)],
        [l1*np.cos(theta1) + l2*np.cos(theta1+theta2),
         l2*np.cos(theta1+theta2)]
    ])
    return J

def inverse_kinematics_2link(target_pos: np.ndarray,
                              l1: float, l2: float,
                              max_iter: int = 100,
                              tol: float = 1e-4) -> np.ndarray:
    \"\"\"
    Numerical IK using Jacobian pseudoinverse.

    Args:
        target_pos: Desired [x, y] position
        l1, l2: Link lengths
        max_iter: Maximum iterations
        tol: Convergence tolerance

    Returns:
        [theta1, theta2] joint angles
    \"\"\"
    # Initial guess
    theta = np.array([0.0, 0.0])

    for _ in range(max_iter):
        # Compute current position
        current_pos = forward_kinematics_2link(theta[0], theta[1], l1, l2)

        # Compute error
        error = target_pos - current_pos

        # Check convergence
        if np.linalg.norm(error) < tol:
            break

        # Compute Jacobian
        J = jacobian_2link(theta[0], theta[1], l1, l2)

        # Compute pseudoinverse
        J_pinv = np.linalg.pinv(J)

        # Update joint angles
        delta_theta = J_pinv @ error
        theta += delta_theta

    return theta
"""
                }
            ],
            practical_applications=[
                "Pick-and-place operations",
                "Path following for manipulators",
                "Humanoid reaching tasks",
                "Surgical robot positioning"
            ],
            common_pitfalls=[
                "Multiple solutions (ambiguity)",
                "No solution (target out of workspace)",
                "Singularities causing instability",
                "Local minima in optimization-based methods"
            ],
            references=[
                "Robotics: Modelling, Planning and Control - Siciliano",
                "A Mathematical Introduction to Robotic Manipulation - Murray"
            ]
        )

        # Simulation Concepts
        concepts["physics_simulation"] = RoboticsConcept(
            id="physics_simulation",
            name="Physics Simulation",
            domain=ConceptDomain.SIMULATION,
            description="Computational modeling of physical interactions, rigid body dynamics, and contact forces.",
            prerequisites=[],
            related_concepts=["gazebo", "isaac_sim", "collision_detection"],
            key_equations=[
                {
                    "name": "Newton-Euler Dynamics",
                    "latex": r"F = ma, \quad \tau = I\alpha",
                    "description": "Force-acceleration and torque-angular acceleration"
                },
                {
                    "name": "Contact Constraint",
                    "latex": r"\lambda \geq 0, \quad d \geq 0, \quad \lambda d = 0",
                    "description": "Complementarity constraint for contact"
                }
            ],
            code_examples=[
                {
                    "language": "python",
                    "title": "Gazebo World Setup",
                    "code": """
# worlds/robot_world.world
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="default">
    <include>
      <uri>model://sun</uri>
    </include>

    <include>
      <uri>model://ground_plane</uri>
    </include>

    <model name="my_robot">
      <pose>0 0 0.5 0 0 0</pose>
      <link name="base_link">
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.1</ixx>
            <iyy>0.1</iyy>
            <izz>0.1</izz>
          </inertia>
        </inertial>

        <collision name="collision">
          <geometry>
            <box><size>1 1 1</size></box>
          </geometry>
        </collision>

        <visual name="visual">
          <geometry>
            <box><size>1 1 1</size></box>
          </geometry>
        </visual>
      </link>
    </model>

    <physics name="default_physics">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
  </world>
</sdf>
"""
                }
            ],
            practical_applications=[
                "Algorithm testing before hardware deployment",
                "Training reinforcement learning policies",
                "Hardware design validation",
                "Failure mode analysis"
            ],
            common_pitfalls=[
                "Sim-to-real gap (dynamics mismatch)",
                "Timestep too large causing instability",
                "Not modeling sensor noise and delays",
                "Overly optimistic friction/contact models"
            ],
            references=[
                "Gazebo Simulation - http://gazebosim.org/",
                "NVIDIA Isaac Sim - https://developer.nvidia.com/isaac-sim"
            ]
        )

        return concepts
```

### Step 2: Create Reasoning Engine

Implement adaptive explanation generation:

```python
# app/services/robotics_reasoning_service.py

from typing import List, Dict, Optional
from app.ai.robotics_knowledge import (
    RoboticsKnowledgeBase,
    RoboticsConcept,
    ConceptDepth,
    ConceptDomain
)
from app.models.responses import RoboticsExplanation

class RoboticsReasoningService:
    """Core reasoning engine for robotics concepts"""

    def __init__(self):
        self.knowledge_base = RoboticsKnowledgeBase()

    def explain_concept(
        self,
        concept_id: str,
        depth: ConceptDepth = ConceptDepth.INTERMEDIATE,
        include_code: bool = True,
        include_math: bool = True,
        include_prerequisites: bool = True
    ) -> RoboticsExplanation:
        """
        Generate adaptive explanation for a robotics concept.

        Args:
            concept_id: ID of concept to explain
            depth: Learning level (beginner/intermediate/advanced)
            include_code: Include code examples
            include_math: Include mathematical formulations
            include_prerequisites: Include prerequisite concepts

        Returns:
            RoboticsExplanation with tailored content
        """
        # Retrieve concept
        concept = self.knowledge_base.get_concept(concept_id)
        if not concept:
            raise ValueError(f"Concept '{concept_id}' not found")

        # Build explanation based on depth
        explanation_text = self._build_explanation(concept, depth)

        # Gather prerequisites if requested
        prerequisites = []
        if include_prerequisites:
            prereq_concepts = self.knowledge_base.get_prerequisites(concept_id)
            prerequisites = [
                {"id": c.id, "name": c.name, "description": c.description}
                for c in prereq_concepts
            ]

        # Include math if requested and depth allows
        equations = []
        if include_math and depth in [ConceptDepth.INTERMEDIATE, ConceptDepth.ADVANCED]:
            equations = concept.key_equations

        # Include code if requested
        code_examples = []
        if include_code:
            code_examples = self._filter_code_examples(concept.code_examples, depth)

        # Build response
        return RoboticsExplanation(
            concept_id=concept.id,
            concept_name=concept.name,
            domain=concept.domain.value,
            explanation=explanation_text,
            depth_level=depth.value,
            prerequisites=prerequisites,
            related_concepts=[
                self.knowledge_base.get_concept(cid).name
                for cid in concept.related_concepts
                if self.knowledge_base.get_concept(cid)
            ],
            mathematical_formulations=equations,
            code_examples=code_examples,
            practical_applications=concept.practical_applications,
            common_pitfalls=concept.common_pitfalls,
            references=concept.references
        )

    def _build_explanation(
        self,
        concept: RoboticsConcept,
        depth: ConceptDepth
    ) -> str:
        """Generate depth-appropriate explanation"""

        if depth == ConceptDepth.BEGINNER:
            return self._beginner_explanation(concept)
        elif depth == ConceptDepth.INTERMEDIATE:
            return self._intermediate_explanation(concept)
        else:  # ADVANCED
            return self._advanced_explanation(concept)

    def _beginner_explanation(self, concept: RoboticsConcept) -> str:
        """Simplified, intuitive explanation"""
        return f"""
{concept.description}

**What is it?**
{concept.name} is a fundamental concept in {concept.domain.value}. Think of it as a building block that helps robots {self._get_domain_purpose(concept.domain)}.

**Why is it important?**
This concept is used in real-world applications like:
{chr(10).join('- ' + app for app in concept.practical_applications[:3])}

**Key Takeaway:**
{self._get_key_takeaway(concept)}
"""

    def _intermediate_explanation(self, concept: RoboticsConcept) -> str:
        """Technical explanation with details"""
        return f"""
{concept.description}

**Technical Overview:**
{concept.name} is a core concept in {concept.domain.value} that enables robots to {self._get_technical_purpose(concept)}.

**How it works:**
{self._get_technical_details(concept)}

**Prerequisites:**
To fully understand this concept, you should be familiar with:
{chr(10).join('- ' + self.knowledge_base.get_concept(pid).name for pid in concept.prerequisites if self.knowledge_base.get_concept(pid))}

**Applications:**
{chr(10).join('- ' + app for app in concept.practical_applications)}

**Common Challenges:**
{chr(10).join('- ' + pitfall for pitfall in concept.common_pitfalls)}
"""

    def _advanced_explanation(self, concept: RoboticsConcept) -> str:
        """Deep technical explanation with theory"""
        return f"""
{concept.description}

**Theoretical Foundation:**
{self._get_theoretical_basis(concept)}

**Implementation Considerations:**
{self._get_implementation_details(concept)}

**Advanced Topics:**
- Relationship to: {', '.join(concept.related_concepts)}
- Extensions and variations
- Current research directions

**Known Limitations:**
{chr(10).join('- ' + pitfall for pitfall in concept.common_pitfalls)}

**Further Reading:**
{chr(10).join('- ' + ref for ref in concept.references)}
"""

    def _filter_code_examples(
        self,
        examples: List[Dict],
        depth: ConceptDepth
    ) -> List[Dict]:
        """Filter code examples by complexity"""
        if depth == ConceptDepth.BEGINNER:
            return examples[:1]  # Show simplest example
        elif depth == ConceptDepth.INTERMEDIATE:
            return examples[:2]
        else:
            return examples  # Show all

    def _get_domain_purpose(self, domain: ConceptDomain) -> str:
        """High-level purpose by domain"""
        purposes = {
            ConceptDomain.ROS2: "communicate and coordinate tasks",
            ConceptDomain.SENSORS: "perceive their environment",
            ConceptDomain.CONTROL: "move accurately and respond to changes",
            ConceptDomain.KINEMATICS: "calculate positions and movements",
            ConceptDomain.SIMULATION: "test behaviors safely before real deployment",
            ConceptDomain.VLA: "understand and act on natural language commands"
        }
        return purposes.get(domain, "perform their functions")

    def _get_technical_purpose(self, concept: RoboticsConcept) -> str:
        """Get technical purpose (concept-specific logic)"""
        # Simplified - would have detailed mapping
        return f"perform critical functions in {concept.domain.value} systems"

    def _get_key_takeaway(self, concept: RoboticsConcept) -> str:
        """Generate beginner-friendly takeaway"""
        return f"{concept.name} is essential for robots to work effectively in {concept.domain.value} applications."

    def _get_technical_details(self, concept: RoboticsConcept) -> str:
        """Generate technical details"""
        # Would have concept-specific implementations
        return f"The {concept.name} system operates by coordinating multiple components to achieve the desired behavior."

    def _get_theoretical_basis(self, concept: RoboticsConcept) -> str:
        """Generate theoretical explanation"""
        return f"The theoretical foundation of {concept.name} draws from control theory, linear algebra, and computational geometry."

    def _get_implementation_details(self, concept: RoboticsConcept) -> str:
        """Generate implementation guidance"""
        return "Implementation requires careful consideration of numerical stability, computational efficiency, and real-time constraints."

    def explain_by_domain(
        self,
        domain: ConceptDomain,
        depth: ConceptDepth = ConceptDepth.INTERMEDIATE
    ) -> List[RoboticsExplanation]:
        """Explain all concepts in a domain"""
        concepts = self.knowledge_base.get_concepts_by_domain(domain)
        return [
            self.explain_concept(c.id, depth=depth)
            for c in concepts
        ]

    def get_learning_path(self, target_concept_id: str) -> List[str]:
        """Generate prerequisite learning path"""
        visited = set()
        path = []

        def dfs(concept_id: str):
            if concept_id in visited:
                return
            visited.add(concept_id)

            concept = self.knowledge_base.get_concept(concept_id)
            if not concept:
                return

            # First add prerequisites
            for prereq_id in concept.prerequisites:
                dfs(prereq_id)

            # Then add this concept
            path.append(concept_id)

        dfs(target_concept_id)
        return path
```

### Step 3: Define Response Models

```python
# app/models/responses.py (additions)

from pydantic import BaseModel, Field
from typing import List, Dict, Optional

class RoboticsExplanation(BaseModel):
    """Comprehensive explanation of a robotics concept"""
    concept_id: str
    concept_name: str
    domain: str  # ros2, sensors, control, etc.
    explanation: str = Field(..., description="Depth-appropriate explanation")
    depth_level: str  # beginner, intermediate, advanced
    prerequisites: List[Dict] = Field(default_factory=list)
    related_concepts: List[str] = Field(default_factory=list)
    mathematical_formulations: List[Dict] = Field(default_factory=list)
    code_examples: List[Dict] = Field(default_factory=list)
    practical_applications: List[str] = Field(default_factory=list)
    common_pitfalls: List[str] = Field(default_factory=list)
    references: List[str] = Field(default_factory=list)

    class Config:
        json_schema_extra = {
            "example": {
                "concept_id": "inverse_kinematics",
                "concept_name": "Inverse Kinematics",
                "domain": "kinematics",
                "explanation": "Computing joint angles to reach desired pose...",
                "depth_level": "intermediate",
                "prerequisites": [
                    {"id": "forward_kinematics", "name": "Forward Kinematics"}
                ],
                "mathematical_formulations": [
                    {
                        "name": "Jacobian Pseudoinverse",
                        "latex": "\\Delta\\theta = J^{\\dagger} \\Delta x"
                    }
                ],
                "code_examples": [{"language": "python", "code": "..."}]
            }
        }

class LearningPath(BaseModel):
    """Ordered prerequisite learning path"""
    target_concept: str
    ordered_concepts: List[Dict] = Field(
        ...,
        description="Concepts in dependency order"
    )
    total_concepts: int
    estimated_learning_time: Optional[str] = None
```

### Step 4: Create API Routes

```python
# app/routes/robotics_reasoning.py

from fastapi import APIRouter, HTTPException, Query, Depends
from app.services.robotics_reasoning_service import RoboticsReasoningService
from app.models.responses import RoboticsExplanation, LearningPath
from app.dependencies import get_reasoning_service
from typing import Optional

router = APIRouter()

@router.get("/explain/{concept_id}", response_model=RoboticsExplanation)
async def explain_concept(
    concept_id: str,
    depth: str = Query("intermediate", regex="^(beginner|intermediate|advanced)$"),
    include_code: bool = Query(True),
    include_math: bool = Query(True),
    service: RoboticsReasoningService = Depends(get_reasoning_service)
):
    """
    Get comprehensive explanation of a robotics concept.

    **Supported Concepts:**
    - ROS 2: `ros2_nodes`, `ros2_topics`, `ros2_services`
    - Sensors: `imu_sensor`, `force_torque_sensor`
    - Control: `pid_control`, `mpc_control`
    - Kinematics: `forward_kinematics`, `inverse_kinematics`
    - Simulation: `physics_simulation`, `gazebo_integration`

    **Depth Levels:**
    - `beginner`: Intuitive, minimal math
    - `intermediate`: Technical with code/math
    - `advanced`: Deep theory and implementation
    """
    try:
        from app.ai.robotics_knowledge import ConceptDepth

        explanation = service.explain_concept(
            concept_id=concept_id,
            depth=ConceptDepth(depth),
            include_code=include_code,
            include_math=include_math
        )
        return explanation
    except ValueError as e:
        raise HTTPException(status_code=404, detail=str(e))
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@router.get("/domain/{domain_name}", response_model=List[RoboticsExplanation])
async def explain_domain(
    domain_name: str,
    depth: str = Query("intermediate", regex="^(beginner|intermediate|advanced)$"),
    service: RoboticsReasoningService = Depends(get_reasoning_service)
):
    """
    Get all concepts in a domain.

    **Domains:**
    - `ros2` - ROS 2 architecture and communication
    - `sensors` - Humanoid sensor systems
    - `control` - Control loops and algorithms
    - `kinematics` - Forward/inverse kinematics
    - `simulation` - Physics simulation
    - `vla` - Vision-Language-Action models
    """
    try:
        from app.ai.robotics_knowledge import ConceptDomain, ConceptDepth

        explanations = service.explain_by_domain(
            domain=ConceptDomain(domain_name),
            depth=ConceptDepth(depth)
        )
        return explanations
    except ValueError as e:
        raise HTTPException(status_code=404, detail=str(e))
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@router.get("/learning-path/{concept_id}", response_model=LearningPath)
async def get_learning_path(
    concept_id: str,
    service: RoboticsReasoningService = Depends(get_reasoning_service)
):
    """
    Get prerequisite learning path for a concept.

    Returns concepts in dependency order (prerequisites first).
    """
    try:
        path = service.get_learning_path(concept_id)

        ordered_concepts = [
            {
                "id": cid,
                "name": service.knowledge_base.get_concept(cid).name,
                "domain": service.knowledge_base.get_concept(cid).domain.value
            }
            for cid in path
            if service.knowledge_base.get_concept(cid)
        ]

        return LearningPath(
            target_concept=concept_id,
            ordered_concepts=ordered_concepts,
            total_concepts=len(ordered_concepts)
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
```

### Step 5: Validation Checklist

After implementation, verify:

- [ ] **Domain Coverage**: All key domains represented (ROS2, sensors, control, kinematics, simulation, VLA)
- [ ] **Depth Adaptation**: Explanations adjust to beginner/intermediate/advanced
- [ ] **Hardware-Agnostic**: No specific robot models assumed
- [ ] **Chapter-Neutral**: No hardcoded chapter references
- [ ] **Backend-Only**: Pure reasoning logic, no UI concerns
- [ ] **Prerequisite Tracking**: Learning paths correctly ordered
- [ ] **Code Examples**: Runnable, well-documented code
- [ ] **Math Formulations**: Proper LaTeX formatting
- [ ] **Reusability**: Components can be imported across services
- [ ] **Testing**: Unit tests for reasoning logic

## Example Usage

```python
# Example 1: Explain inverse kinematics at intermediate level
GET /api/robotics/explain/inverse_kinematics?depth=intermediate&include_code=true

Response:
{
  "concept_id": "inverse_kinematics",
  "concept_name": "Inverse Kinematics",
  "domain": "kinematics",
  "explanation": "Computing joint angles required to reach a desired end-effector pose...",
  "depth_level": "intermediate",
  "prerequisites": [
    {"id": "forward_kinematics", "name": "Forward Kinematics"}
  ],
  "mathematical_formulations": [
    {
      "name": "Jacobian Pseudoinverse Method",
      "latex": "\\Delta\\theta = J^{\\dagger} \\Delta x",
      "description": "Iterative IK using Jacobian pseudoinverse"
    }
  ],
  "code_examples": [
    {
      "language": "python",
      "title": "Numerical IK with Jacobian",
      "code": "def inverse_kinematics_2link(...)..."
    }
  ],
  "practical_applications": [
    "Pick-and-place operations",
    "Path following for manipulators"
  ],
  "common_pitfalls": [
    "Multiple solutions (ambiguity)",
    "Singularities causing instability"
  ]
}

# Example 2: Get all ROS 2 concepts
GET /api/robotics/domain/ros2?depth=beginner

Response: [
  { concept_id: "ros2_nodes", ... },
  { concept_id: "ros2_topics", ... },
  ...
]

# Example 3: Get learning path
GET /api/robotics/learning-path/inverse_kinematics

Response:
{
  "target_concept": "inverse_kinematics",
  "ordered_concepts": [
    {"id": "forward_kinematics", "name": "Forward Kinematics", "domain": "kinematics"},
    {"id": "inverse_kinematics", "name": "Inverse Kinematics", "domain": "kinematics"}
  ],
  "total_concepts": 2
}
```

## Output

When this skill is invoked, it generates:

1. **Knowledge Base** (`app/ai/robotics_knowledge.py`) - Structured concept repository
2. **Reasoning Service** (`app/services/robotics_reasoning_service.py`) - Adaptive explanation engine
3. **Response Models** (`app/models/responses.py`) - RoboticsExplanation, LearningPath
4. **API Routes** (`app/routes/robotics_reasoning.py`) - RESTful endpoints
5. **Tests** (`tests/test_reasoning/`) - Comprehensive test coverage

## Success Criteria

- ✅ Explains ROS 2, sensors, control, kinematics, simulation, VLA
- ✅ Adapts depth (beginner → intermediate → advanced)
- ✅ Hardware-agnostic (generic principles, not specific robots)
- ✅ Chapter-neutral (no hardcoded chapter assumptions)
- ✅ Backend-only (pure reasoning, no UI)
- ✅ Reusable across all textbook services
- ✅ Traceable to standard robotics references
