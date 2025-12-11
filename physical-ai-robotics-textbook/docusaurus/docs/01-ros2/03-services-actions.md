---
sidebar_position: 3
title: "Services and Actions: Synchronous and Goal-Oriented Communication"
id: "01-ros2-services-actions"
---

import ReadingTime from '@site/src/components/ReadingTime';
import ViewToggle from '@site/src/components/ViewToggle';

<ReadingTime minutes={20} />
<ViewToggle />

# Services and Actions: Synchronous and Goal-Oriented Communication

<div className="full-content">

**Module**: 01 - ROS 2
**Learning Objectives**:
- Implement ROS 2 services for synchronous communication with proper request-response handling
- Create and use ROS 2 actions for goal-oriented tasks with feedback and cancellation
- Design appropriate communication patterns for different use cases in humanoid robotics
- Handle service and action responses, feedback, and errors effectively
- Debug common service and action issues and implement proper error handling

**Prerequisites**: Understanding of ROS 2 architecture, nodes, and topics
**Estimated Time**: 3-4 hours

---

## Introduction

While topics enable asynchronous publish-subscribe communication, services and actions provide synchronous request-response and goal-oriented communication patterns respectively. These are essential for humanoid robotics applications requiring reliable, stateful interactions. Services are ideal for simple request-response operations like configuration changes or calibration procedures, while actions are designed for long-running tasks that require feedback and cancellation capabilities, such as navigation or manipulation sequences.

The synchronous nature of services makes them perfect for operations that require immediate results, while the goal-feedback-result pattern of actions enables complex, long-running tasks with progress monitoring. Understanding when and how to use each communication pattern is crucial for building robust humanoid robotics applications.

---

## Services: Synchronous Request-Response Communication

Services provide a synchronous client-server communication pattern where a client sends a request and waits for a response from the server. This pattern is ideal for operations that require immediate results or acknowledgments.

### Service Architecture

- **Service Server**: Provides the service implementation
- **Service Client**: Makes requests to the service
- **Service Type**: Defines request and response message types
- **Blocking Call**: Client waits for server response

### Creating Custom Service Types

Service types are defined in `.srv` files with the request and response separated by `---`:

```
# AddTwoInts.srv
int64 a
int64 b
---
int64 sum
```

### Service Server Implementation

#### Python Service Server

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning {request.a} + {request.b} = {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    minimal_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### C++ Service Server

```cpp
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

class MinimalService : public rclcpp::Node
{
public:
    MinimalService() : Node("minimal_service")
    {
        service_ = this->create_service<example_interfaces::srv::AddTwoInts>(
            "add_two_ints",
            [this](const example_interfaces::srv::AddTwoInts::Request::SharedPtr request,
                   example_interfaces::srv::AddTwoInts::Response::SharedPtr response) {
                response->sum = request->a + request->b;
                RCLCPP_INFO(this->get_logger(), "Incoming request: %ld + %ld = %ld",
                           request->a, request->b, response->sum);
            });
    }

private:
    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service_;
};
```

### Service Client Implementation

#### Python Service Client

```python
import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalClient(Node):
    def __init__(self):
        super().__init__('minimal_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClient()
    response = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    minimal_client.get_logger().info(f'Result of add_two_ints: {response.sum}')
    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### C++ Service Client

```cpp
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using AddTwoInts = example_interfaces::srv::AddTwoInts;

class MinimalClient : public rclcpp::Node
{
public:
    MinimalClient() : Node("minimal_client")
    {
        client_ = this->create_client<AddTwoInts>("add_two_ints");
        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
        }
    }

    void send_request(int64_t a, int64_t b)
    {
        auto request = std::make_shared<AddTwoInts::Request>();
        request->a = a;
        request->b = b;

        auto future = client_->async_send_request(request);
        rclpy::spin_until_future_complete(this->get_node_base_interface(), future);
        RCLCPP_INFO(this->get_logger(), "Result of add_two_ints: %ld", future.get()->sum);
    }

private:
    rclcpp::Client<AddTwoInts>::SharedPtr client_;
};
```

## Actions: Goal-Oriented Communication

Actions are designed for long-running tasks that provide feedback and can be canceled. They use a goal-feedback-result pattern, making them ideal for operations that take time and need to report progress.

### Action Architecture

- **Goal**: Request for a long-running task
- **Feedback**: Periodic updates on task progress
- **Result**: Final outcome of the task
- **Cancelation**: Ability to stop ongoing actions

### Creating Custom Action Types

Action types are defined in `.action` files with goal, result, and feedback separated by `---`:

```
# Fibonacci.action
int32 order
---
int32[] sequence
---
int32[] partial_sequence
```

### Action Server Implementation

#### Python Action Server

```python
import time
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            execute_callback=self.execute_callback,
            callback_group=rclpy.callback_groups.ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()

            feedback_msg.partial_sequence.append(
                feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i-1])

            self.get_logger().info(f'Publishing feedback: {feedback_msg.partial_sequence}')
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.partial_sequence
        self.get_logger().info(f'Returning result: {result.sequence}')

        return result

def main(args=None):
    rclpy.init(args=args)
    fibonacci_action_server = FibonacciActionServer()
    rclpy.spin(fibonacci_action_server)
    fibonacci_action_server.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Action Client Implementation

#### Python Action Client

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class FibonacciActionClient(Node):
    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()
        self.get_logger().info(f'Sending goal with order: {order}')

        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.partial_sequence}')

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.sequence}')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    action_client = FibonacciActionClient()
    action_client.send_goal(10)
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
```

## Hands-On Exercises

:::tip Exercise 1.3.1: Robot Configuration Service Implementation

**Objective**: Implement a service-based robot configuration system that allows remote configuration of robot parameters.

**Difficulty**: ⭐⭐ Medium

**Time Estimate**: 35-45 minutes

**Requirements**:
1. Create a custom service type for robot configuration
2. Implement a service server that handles configuration requests
3. Create a service client that sends configuration requests
4. Test the system with different configuration parameters
5. Implement proper error handling and validation

**Starter Code**:
```python title="robot_config_service.py"
#!/usr/bin/env python3
"""
ROS 2 Services and Actions Exercise - Robot Configuration Service

This script implements a service-based robot configuration system.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
# TODO: Import custom service types after defining them
# from robot_config_msgs.srv import SetConfiguration

# For now, we'll use SetBool as an example
from std_srvs.srv import SetBool


class RobotConfigServer(Node):
    """Service server that handles robot configuration requests."""

    def __init__(self):
        """Initialize the robot configuration service server."""
        super().__init__('robot_config_server')

        # TODO: Create service for robot configuration
        # self.config_service = self.create_service(
        #     SetConfiguration, 'set_robot_configuration', self.config_callback)

        # For now, using SetBool as example
        self.config_service = self.create_service(
            SetBool, 'set_robot_mode', self.mode_callback)

        # Initialize robot configuration parameters
        self.robot_params = {
            'max_velocity': 1.0,
            'acceleration_limit': 2.0,
            'collision_threshold': 0.5,
            'operation_mode': 'manual'
        }

        self.get_logger().info('Robot Configuration Server initialized')

    def mode_callback(self, request, response):
        """Handle robot mode change requests."""
        # TODO: Implement configuration logic
        # TODO: Validate configuration parameters
        # TODO: Apply configuration changes
        # TODO: Return appropriate response

        # For example implementation
        new_mode = 'autonomous' if request.data else 'manual'
        self.robot_params['operation_mode'] = new_mode

        response.success = True
        response.message = f'Robot mode changed to {new_mode}'

        self.get_logger().info(response.message)
        return response

    def config_callback(self, request, response):
        """Handle configuration requests."""
        # TODO: Implement full configuration logic
        # TODO: Validate all parameters
        # TODO: Apply configuration changes safely
        # TODO: Return success/failure response
        pass


class RobotConfigClient(Node):
    """Service client that sends robot configuration requests."""

    def __init__(self):
        """Initialize the robot configuration client."""
        super().__init__('robot_config_client')

        # TODO: Create client for robot configuration service
        # self.config_client = self.create_client(SetConfiguration, 'set_robot_configuration')

        # For now, using SetBool as example
        self.mode_client = self.create_client(SetBool, 'set_robot_mode')

        # Wait for service to be available
        while not self.mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Configuration service not available, waiting...')

        self.request = SetBool.Request()

        self.get_logger().info('Robot Configuration Client initialized')

    def send_mode_request(self, enable_autonomous):
        """Send a robot mode change request."""
        self.request.data = enable_autonomous

        future = self.mode_client.call_async(self.request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'Configuration response: {response.message}')
            return response.success
        else:
            self.get_logger().error('Configuration service call failed')
            return False


def main(args=None):
    """Main function to run the robot configuration service demo."""
    rclpy.init(args=args)

    # TODO: Create both server and client nodes
    # TODO: Implement service request/response logic
    # TODO: Test configuration changes
    # TODO: Validate parameter changes

    server_node = RobotConfigServer()
    client_node = RobotConfigClient()

    # Example: Send configuration request
    client_node.send_mode_request(True)

    # Keep server running
    try:
        rclpy.spin(server_node)
    except KeyboardInterrupt:
        pass
    finally:
        server_node.destroy_node()
        client_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Deliverable**: A complete robot configuration service system with server and client nodes that handle parameter changes safely.

**Success Criteria**:
- [ ] Custom service type defined for robot configuration
- [ ] Service server handles configuration requests properly
- [ ] Service client sends requests and handles responses
- [ ] Proper validation and error handling implemented
- [ ] System tested with different configuration parameters

**Test Commands**:
```bash
# Create a custom service package
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python robot_config_pkg --dependencies rclpy std_srvs

# Create srv directory and define custom service
mkdir robot_config_pkg/srv
echo "# RobotConfig.srv
string param_name
float64 param_value
---
bool success
string message" > robot_config_pkg/srv/RobotConfig.srv

# Update CMakeLists.txt and package.xml for service generation
# Then build the workspace
cd ~/ros2_ws
colcon build --packages-select robot_config_pkg
source install/setup.bash

# Run the configuration service
ros2 run robot_config_pkg robot_config_service
```

**Expected Output**:
The service should accept configuration requests, validate parameters, apply changes, and return appropriate success/error responses.

**Challenge**: Add authentication to the configuration service to ensure only authorized clients can change robot parameters.

**Hints**:
<details>
<summary>Click for hint 1</summary>

Custom service types go in the `srv/` directory of your package and need to be listed in CMakeLists.txt for generation.
</details>

<details>
<summary>Click for hint 2</summary>

Use `rclpy.callback_groups.MutuallyExclusiveCallbackGroup()` for services that modify shared resources.
</details>
:::

:::tip Exercise 1.3.2: Navigation Action with Feedback and Cancellation

**Objective**: Implement a complete navigation action system with progress feedback and cancellation capability.

**Difficulty**: ⭐⭐⭐ Hard

**Time Estimate**: 50-65 minutes

**Requirements**:
1. Create a custom action type for robot navigation
2. Implement an action server that simulates navigation with feedback
3. Create an action client that monitors progress and can cancel goals
4. Test the system with different navigation scenarios
5. Implement proper state management and error handling

**Starter Code**:
```python title="navigation_action_system.py"
#!/usr/bin/env python3
"""
ROS 2 Services and Actions Exercise - Navigation Action System

This script implements a complete navigation action system with feedback and cancellation.
"""

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile
import time
import math

# TODO: Import custom action types after defining them
# from navigation_msgs.action import NavigateToPose

# For now, we'll use Fibonacci as an example
from example_interfaces.action import Fibonacci


class NavigationActionServer(Node):
    """Action server that handles navigation goals with feedback and cancellation."""

    def __init__(self):
        """Initialize the navigation action server."""
        super().__init__('navigation_action_server')

        # TODO: Create action server for navigation
        # self.nav_action_server = ActionServer(
        #     self,
        #     NavigateToPose,
        #     'navigate_to_pose',
        #     execute_callback=self.execute_navigation,
        #     callback_group=ReentrantCallbackGroup(),
        #     goal_callback=self.goal_callback,
        #     cancel_callback=self.cancel_callback)

        # For now, using Fibonacci as example
        self.fibonacci_action_server = ActionServer(
            self,
            Fibonacci,
            'navigate_fibonacci',  # Using as example
            execute_callback=self.execute_navigation,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)

        # Navigation state management
        self.current_position = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.is_navigating = False
        self.obstacles = [{'x': 5.0, 'y': 5.0, 'radius': 1.0}]  # Example obstacles

        self.get_logger().info('Navigation Action Server initialized')

    def goal_callback(self, goal_request):
        """Handle incoming navigation goal requests."""
        # TODO: Validate navigation goal
        # TODO: Check if navigation is possible
        # TODO: Return appropriate goal response

        self.get_logger().info(f'Received navigation goal: {goal_request}')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Handle navigation cancellation requests."""
        self.get_logger().info('Received navigation cancellation request')
        return CancelResponse.ACCEPT

    def execute_navigation(self, goal_handle):
        """Execute the navigation goal with feedback and cancellation."""
        # TODO: Implement navigation algorithm
        # TODO: Publish progress feedback
        # TODO: Handle cancellation requests
        # TODO: Return final result

        self.get_logger().info('Starting navigation execution...')

        # Example implementation with Fibonacci feedback
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            # Check for cancellation
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Navigation goal canceled')

                # TODO: Return appropriate result for cancellation
                result = Fibonacci.Result()
                result.sequence = feedback_msg.partial_sequence
                return result

            # Simulate navigation progress
            feedback_msg.partial_sequence.append(
                feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i-1])

            # Publish feedback
            goal_handle.publish_feedback(feedback_msg)

            self.get_logger().info(f'Navigation progress: {feedback_msg.partial_sequence[-1]}')

            # Simulate time delay
            time.sleep(1)

        # Navigation completed successfully
        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.partial_sequence

        self.get_logger().info(f'Navigation completed: {result.sequence}')
        return result


class NavigationActionClient(Node):
    """Action client that sends navigation goals and monitors progress."""

    def __init__(self):
        """Initialize the navigation action client."""
        super().__init__('navigation_action_client')

        # TODO: Create action client for navigation
        # self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # For now, using Fibonacci as example
        self.fibonacci_client = ActionClient(self, Fibonacci, 'navigate_fibonacci')

        self.get_logger().info('Navigation Action Client initialized')

    def send_navigation_goal(self, target_x, target_y, target_theta=0.0):
        """Send a navigation goal to the action server."""
        # TODO: Create navigation goal message
        # TODO: Wait for action server
        # TODO: Send goal asynchronously
        # TODO: Add callbacks for response and result

        # For example implementation with Fibonacci
        goal_msg = Fibonacci.Goal()
        goal_msg.order = 10  # Using as example

        self.fibonacci_client.wait_for_server()
        self.get_logger().info(f'Sending navigation goal to Fibonacci action (example)')

        send_goal_future = self.fibonacci_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle the goal response."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Navigation goal rejected')
            return

        self.get_logger().info('Navigation goal accepted')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """Handle navigation feedback."""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Navigation feedback: {feedback.partial_sequence[-1]}')

    def get_result_callback(self, future):
        """Handle the navigation result."""
        result = future.result().result
        self.get_logger().info(f'Navigation result: {result.sequence}')


def main(args=None):
    """Main function to run the navigation action system."""
    rclpy.init(args=args)

    # TODO: Create both action server and client nodes
    # TODO: Implement navigation logic
    # TODO: Test with different scenarios
    # TODO: Verify feedback and cancellation

    server_node = NavigationActionServer()
    client_node = NavigationActionClient()

    # Example: Send navigation goal
    client_node.send_navigation_goal(10.0, 10.0)

    # Keep server running
    try:
        rclpy.spin(server_node)
    except KeyboardInterrupt:
        pass
    finally:
        server_node.destroy_node()
        client_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Deliverable**: A complete navigation action system with proper feedback, cancellation, and state management.

**Success Criteria**:
- [ ] Custom navigation action type defined
- [ ] Action server handles goals with feedback and cancellation
- [ ] Action client monitors progress and can cancel goals
- [ ] Proper state management and error handling
- [ ] System tested with different navigation scenarios

**Test Commands**:
```bash
# Create navigation action package
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python navigation_action_pkg --dependencies rclpy action_msgs

# Create action directory and define custom action
mkdir navigation_action_pkg/action
echo "# NavigateToPose.action
geometry_msgs/PoseStamped target_pose
float64 tolerance
---
bool success
string message
int32 result_code
---
geometry_msgs/PoseStamped current_pose
float64 distance_remaining
float64 time_elapsed
int32[] path_progress" > navigation_action_pkg/action/NavigateToPose.action

# Update CMakeLists.txt and package.xml for action generation
# Then build the workspace
cd ~/ros2_ws
colcon build --packages-select navigation_action_pkg
source install/setup.bash

# Run the navigation action system
ros2 run navigation_action_pkg navigation_action_system

# Test cancellation
ros2 action send_goal /navigate_to_pose navigation_msgs/action/NavigateToPose '{target_pose: {pose: {position: {x: 10.0, y: 10.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, tolerance: 0.5}'
```

**Expected Output**:
The navigation action should accept goals, provide regular feedback on progress, handle cancellation requests gracefully, and return appropriate results.

**Challenge**: Implement obstacle avoidance in the navigation action that dynamically adjusts the path when obstacles are detected.

**Hints**:
<details>
<summary>Click for hint 1</summary>

Use `rclpy.callback_groups.ReentrantCallbackGroup()` for action servers to allow concurrent execution of callbacks.
</details>

<details>
<summary>Click for hint 2</summary>

Check `goal_handle.is_cancel_requested` regularly during long-running operations to respond to cancellation requests promptly.
</details>
:::

:::tip Exercise 1.3.3: Communication Pattern Selection and Optimization

**Objective**: Analyze and implement the most appropriate communication patterns for different humanoid robotics scenarios.

**Difficulty**: ⭐⭐ Medium

**Time Estimate**: 40-50 minutes

**Requirements**:
1. Identify appropriate communication patterns for different robot operations
2. Implement a hybrid system using topics, services, and actions appropriately
3. Optimize the system for performance and reliability
4. Test the system with realistic humanoid robotics scenarios
5. Document the rationale for each communication pattern choice

**Starter Code**:
```python title="communication_pattern_system.py"
#!/usr/bin/env python3
"""
ROS 2 Services and Actions Exercise - Communication Pattern Selection

This script implements a system using appropriate communication patterns for different operations.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String, Float32
from std_srvs.srv import Trigger, SetBool
from example_interfaces.action import Fibonacci
import time
import threading


class RobotCommunicationManager(Node):
    """Node that demonstrates appropriate use of communication patterns."""

    def __init__(self):
        """Initialize the communication pattern manager."""
        super().__init__('robot_communication_manager')

        # TOPICS: Continuous sensor data and robot state
        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.sensor_pub = self.create_publisher(Float32, 'robot/sensors/joint_position', qos_profile)
        self.state_pub = self.create_publisher(String, 'robot/state/current_mode', 10)

        # SERVICES: One-time operations that require immediate response
        self.calibrate_srv = self.create_service(
            Trigger, 'robot/calibrate_sensors', self.calibrate_callback)
        self.emergency_stop_srv = self.create_service(
            SetBool, 'robot/emergency_stop', self.emergency_stop_callback)

        # ACTIONS: Long-running operations requiring feedback
        self.nav_action_server = ActionServer(
            self,
            Fibonacci,  # Using as example
            'robot/navigation/move_to_pose',
            execute_callback=self.execute_navigation,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)

        # Timers for continuous operations
        self.sensor_timer = self.create_timer(0.1, self.publish_sensor_data)  # 10 Hz
        self.state_timer = self.create_timer(1.0, self.publish_state)         # 1 Hz

        # Robot state
        self.joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Example 6 joints
        self.current_mode = "idle"
        self.emergency_stopped = False

        self.get_logger().info('Robot Communication Manager initialized')

    def publish_sensor_data(self):
        """Publish continuous sensor data using topics."""
        # TODO: Publish realistic sensor data
        # TODO: Update joint positions with simulated movement
        # TODO: Publish other sensor data as appropriate

        msg = Float32()
        # Simulate changing joint position
        self.joint_positions[0] += 0.01
        if self.joint_positions[0] > 3.14:
            self.joint_positions[0] = -3.14

        msg.data = self.joint_positions[0]
        self.sensor_pub.publish(msg)

    def publish_state(self):
        """Publish robot state information."""
        msg = String()
        msg.data = self.current_mode
        self.state_pub.publish(msg)

    def calibrate_callback(self, request, response):
        """Handle sensor calibration service request."""
        # TODO: Implement calibration logic
        # TODO: Return success/failure response
        # TODO: Update robot state appropriately

        self.get_logger().info('Starting sensor calibration...')

        # Simulate calibration process
        time.sleep(2)  # Simulated calibration time

        response.success = True
        response.message = 'Sensors calibrated successfully'

        self.get_logger().info(response.message)
        return response

    def emergency_stop_callback(self, request, response):
        """Handle emergency stop service request."""
        # TODO: Implement emergency stop logic
        # TODO: Stop all ongoing operations
        # TODO: Return appropriate response

        self.emergency_stopped = request.data
        self.current_mode = "emergency_stopped" if self.emergency_stopped else "idle"

        response.success = True
        response.message = f'Emergency stop {"activated" if self.emergency_stopped else "deactivated"}'

        self.get_logger().info(response.message)
        return response

    def goal_callback(self, goal_request):
        """Handle navigation goal requests."""
        self.get_logger().info('Received navigation goal')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Handle navigation cancellation requests."""
        self.get_logger().info('Received navigation cancellation')
        return CancelResponse.ACCEPT

    def execute_navigation(self, goal_handle):
        """Execute navigation with feedback."""
        self.get_logger().info('Starting navigation execution...')

        # TODO: Implement navigation logic
        # TODO: Publish feedback during execution
        # TODO: Handle cancellation requests
        # TODO: Return appropriate result

        # Example with Fibonacci action
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = Fibonacci.Result()
                result.sequence = feedback_msg.partial_sequence
                return result

            feedback_msg.partial_sequence.append(
                feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i-1])

            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.5)  # Simulated navigation time

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.partial_sequence
        return result


def main(args=None):
    """Main function to run the communication pattern system."""
    rclpy.init(args=args)

    # TODO: Create and run the communication manager node
    # TODO: Test different communication patterns
    # TODO: Verify appropriate pattern usage
    # TODO: Measure performance and responsiveness

    comm_manager = RobotCommunicationManager()

    try:
        rclpy.spin(comm_manager)
    except KeyboardInterrupt:
        pass
    finally:
        comm_manager.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Deliverable**: A comprehensive system that uses appropriate communication patterns for different robot operations with performance optimization.

**Success Criteria**:
- [ ] Appropriate communication pattern selected for each operation type
- [ ] Topics used for continuous data streams
- [ ] Services used for immediate request-response operations
- [ ] Actions used for long-running operations with feedback
- [ ] System performance optimized for each pattern

**Test Commands**:
```bash
# Create communication pattern package
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python communication_pattern_pkg --dependencies rclpy std_msgs std_srvs example_interfaces

# Add the communication pattern system code to communication_pattern_pkg/communication_pattern_system.py

# Build and run the system
cd ~/ros2_ws
colcon build --packages-select communication_pattern_pkg
source install/setup.bash

# Run the communication pattern system
ros2 run communication_pattern_pkg communication_pattern_system

# Test different communication patterns
ros2 service call /robot/calibrate_sensors std_srvs/srv/Trigger
ros2 service call /robot/emergency_stop std_srvs/srv/SetBool "{data: true}"
ros2 topic echo /robot/sensors/joint_position
```

**Expected Output**:
The system should demonstrate appropriate use of topics for continuous data, services for immediate operations, and actions for long-running tasks with feedback.

**Challenge**: Implement a communication pattern analyzer that monitors system performance and suggests optimal patterns for different operations.

**Hints**:
<details>
<summary>Click for hint 1</summary>

Use topics for data that changes frequently and doesn't require acknowledgment (sensors, state).
</details>

<details>
<summary>Click for hint 2</summary>

Use services for operations that need immediate results and are relatively quick (configuration, calibration).
</details>
:::

---

## Common Issues and Debugging

:::caution Common Problems

**Problem 1: Service Connection and Discovery Issues**

**Symptoms**:
- Service client cannot find the service server
- Error messages about service not being available
- Service calls timing out
- Nodes running on different machines cannot discover services

**Cause**: Service discovery in ROS 2 relies on domain IDs, network configuration, and proper service initialization. If any of these aren't set up correctly, clients cannot connect to services.

**Solution**:
```bash
# Check domain ID consistency:
echo $ROS_DOMAIN_ID  # Should be the same on all machines
export ROS_DOMAIN_ID=0  # Set to default if needed

# For multi-machine communication:
# 1. Ensure same ROS_DOMAIN_ID on all machines
# 2. Check firewall settings for DDS ports (7400-7500)
# 3. Verify network connectivity with ping
# 4. Use specific network interface if needed
```

**Verification**:
```bash
# Test service discovery:
ros2 service list  # Should show all available services
ros2 service types  # Check service types
ros2 service info /service_name  # Get detailed service information
ros2 node info /node_name  # Check node services
```

---

**Problem 2: Action Server and Client Communication Issues**

**Symptoms**:
- Action goals not being accepted by the server
- Feedback messages not being received by the client
- Action results not being delivered
- Cancellation requests not being processed

**Cause**: Action communication requires proper initialization of both server and client, correct action type matching, and appropriate callback group configurations.

**Solution**:
```python
# Proper action server implementation:
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup

# Use reentrant callback group for concurrent access
action_server = ActionServer(
    node,
    ActionType,
    'action_name',
    execute_callback=execute_callback,
    callback_group=ReentrantCallbackGroup(),  # Allow concurrent callbacks
    goal_callback=goal_callback,
    cancel_callback=cancel_callback
)

# Proper action client implementation:
from rclpy.action import ActionClient

action_client = ActionClient(node, ActionType, 'action_name')

# Always wait for server before sending goals:
action_client.wait_for_server()

# Check for proper action type matching between client and server
```

**Verification**:
```bash
# Check action availability:
ros2 action list  # Should show all available actions
ros2 action types  # Check action types
ros2 action info /action_name  # Get detailed action information
```

---

**Problem 3: Service Timeout and Blocking Issues**

**Symptoms**:
- Service calls timing out unexpectedly
- Client nodes becoming unresponsive during service calls
- High latency in service responses
- Thread blocking in service callbacks

**Cause**: Service calls are synchronous and blocking by default. If the service server takes too long to respond or gets stuck, the client will wait indefinitely or until timeout.

**Solution**:
```python
# For service clients - use async calls to avoid blocking:
future = client.call_async(request)
# Don't use spin_until_future_complete in main thread, use callbacks instead

# In service callbacks - avoid blocking operations:
def service_callback(self, request, response):
    # DON'T do this in callback thread:
    # time.sleep(10)  # Blocking operation
    # result = complex_calculation()  # Long-running operation

    # DO use separate threads for long operations:
    import threading
    thread = threading.Thread(target=self.long_running_task, args=(request, response))
    thread.start()
    return response  # Return immediately

# Set appropriate timeouts:
while not client.wait_for_service(timeout_sec=1.0):
    self.get_logger().info('Waiting for service...')
```

**Verification**:
```bash
# Monitor service response times:
# Use custom logging to measure service execution time
# Check for blocking operations in service callbacks
```

---

**Problem 4: Action Feedback and Result Issues**

**Symptoms**:
- Feedback messages not being published during action execution
- Result not being returned after action completion
- Inconsistent feedback intervals
- Lost feedback messages

**Cause**: Improper feedback publishing during action execution, incorrect goal handle usage, or issues with the action server implementation.

**Solution**:
```python
# Proper feedback publishing in action execution:
def execute_callback(self, goal_handle):
    # Initialize feedback message
    feedback_msg = ActionType.Feedback()

    # Publish feedback regularly during execution
    for step in range(goal_handle.request.steps):
        # Check for cancellation requests
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result = ActionType.Result()
            return result

        # Update feedback message
        feedback_msg.progress = step / goal_handle.request.steps
        feedback_msg.status = f"Step {step} of {goal_handle.request.steps}"

        # Publish feedback to client
        goal_handle.publish_feedback(feedback_msg)

        # Simulate work
        time.sleep(0.1)

    # Complete the action successfully
    goal_handle.succeed()
    result = ActionType.Result()
    result.success = True
    result.message = "Action completed successfully"
    return result

# Proper client-side feedback handling:
def feedback_callback(self, feedback_msg):
    # Process feedback immediately
    self.get_logger().info(f'Progress: {feedback_msg.feedback.progress}')
```

**Verification**:
```bash
# Test feedback publishing:
# Monitor feedback frequency and content
# Verify feedback messages are received during execution
# Check that result is returned after completion
```

---

**Problem 5: Callback Group and Concurrency Issues**

**Symptoms**:
- Service callbacks not executing concurrently
- Action callbacks interfering with each other
- Deadlocks when using services/actions in complex systems
- Unexpected callback ordering

**Cause**: Improper callback group configuration can lead to serialization of callbacks or deadlocks when multiple callbacks try to access shared resources.

**Solution**:
```python
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

# For services that modify shared resources:
exclusive_group = MutuallyExclusiveCallbackGroup()

service1 = node.create_service(
    ServiceType1, 'service1', callback1,
    callback_group=exclusive_group)

service2 = node.create_service(
    ServiceType2, 'service2', callback2,
    callback_group=exclusive_group)  # Same group - serialized execution

# For actions that need concurrent access:
reentrant_group = ReentrantCallbackGroup()

action_server = ActionServer(
    node,
    ActionType,
    'action_name',
    execute_callback=execute_callback,
    callback_group=reentrant_group,  # Allows concurrent callbacks
    goal_callback=goal_callback,
    cancel_callback=cancel_callback
)

# For mixed systems, use appropriate groups:
regular_group = node.default_callback_group  # Used for most entities
```

**Verification**:
```bash
# Monitor callback execution:
# Use timestamps to verify concurrent execution
# Check for deadlocks during stress testing
# Verify callback group assignments
```
:::

---

## When to Use Each Communication Pattern

### Topics vs Services vs Actions

| Pattern | Use Case | Characteristics |
|---------|----------|-----------------|
| **Topics** | Continuous data streams | Asynchronous, one-to-many, fire-and-forget |
| **Services** | Request-response operations | Synchronous, one-to-one, blocking |
| **Actions** | Long-running tasks | Asynchronous, with feedback and cancellation |

### Communication Pattern Selection Guide

- **Use Topics** for:
  - Sensor data streams (LIDAR, cameras, IMU)
  - Robot state updates (odometry, joint states)
  - Continuous monitoring data
  - Broadcast information to multiple subscribers

- **Use Services** for:
  - Calibration procedures
  - Configuration changes
  - One-time operations with immediate results
  - Simple queries that require responses

- **Use Actions** for:
  - Navigation tasks
  - Manipulation sequences
  - Any long-running operation requiring feedback
  - Operations that might need cancellation

## Best Practices

### Service Design
- Keep service calls short and efficient
- Use appropriate timeout values for service calls
- Implement proper error handling in service callbacks
- Avoid blocking operations in service callbacks

### Action Design
- Provide meaningful feedback during long operations
- Implement proper cancellation handling
- Set realistic goal acceptance criteria
- Use appropriate callback groups for concurrency

### Performance Considerations
- Avoid blocking operations in service callbacks
- Use appropriate callback groups for concurrency
- Monitor service and action response times
- Consider QoS settings for different communication patterns

### Error Handling
- Always check for service availability before calling
- Handle action goal rejection appropriately
- Implement timeout mechanisms for long operations
- Use proper exception handling in callbacks

## Summary

In this chapter, you learned:

- ✅ How to implement ROS 2 services for synchronous communication with proper request-response handling
- ✅ How to create and use ROS 2 actions for goal-oriented tasks with feedback and cancellation
- ✅ How to design appropriate communication patterns for different use cases in humanoid robotics
- ✅ How to handle service and action responses, feedback, and errors effectively
- ✅ How to debug common service and action issues and implement proper error handling

**Key Takeaways**:
- Services are ideal for simple request-response operations that require immediate results
- Actions are designed for long-running tasks that need progress feedback and cancellation
- Proper callback group configuration is essential for concurrent execution
- Each communication pattern has specific use cases where it excels
- Error handling and timeout management are critical for robust operation

---

## Additional Resources

**Official Documentation**:
- [ROS 2 Services and Actions](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Using-Async-Callbacks.html)
- [Creating Services](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Single-Package-Define-And-Use-Interface-Python.html)

**Tutorials**:
- [Writing a Simple Service and Client](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html)
- [Writing an Action Server and Client](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-An-Action-Server-Client/Py.html)

**Example Code**:
- [ROS 2 Demos Repository](https://github.com/ros2/demos)

---

**Navigation**: [← Previous Chapter](./02-nodes-topics.md) | [Next Chapter →](./04-python-packages.md)

</div>

<div className="summary-content">

## 📝 Chapter Summary

### Key Concepts
- **Services**: Synchronous request-response communication for immediate operations
- **Actions**: Asynchronous goal-feedback-result pattern for long-running tasks
- **Communication Patterns**: Appropriate selection based on operation characteristics
- **Callback Groups**: Proper concurrency management for multiple callbacks
- **Error Handling**: Robust handling of timeouts, cancellations, and failures

### Essential Code Pattern
```python
# Service Server Pattern
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class ServiceServer(Node):
    def __init__(self):
        super().__init__('service_server')
        self.service = self.create_service(AddTwoInts, 'add_two_ints', self.callback)

    def callback(self, request, response):
        response.sum = request.a + request.b
        return response

# Action Server Pattern
from rclpy.action import ActionServer
from example_interfaces.action import Fibonacci

class ActionServerClass(Node):
    def __init__(self):
        super().__init__('action_server')
        self.action_server = ActionServer(self, Fibonacci, 'fibonacci', self.execute)

    def execute(self, goal_handle):
        # Process goal with feedback
        for i in range(goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return Fibonacci.Result()
            # Publish feedback
            goal_handle.publish_feedback(Fibonacci.Feedback())
        goal_handle.succeed()
        return Fibonacci.Result()
```

### Quick Reference
| Pattern | Use Case | Best Practice |
|---------|----------|---------------|
| Services | Immediate responses | Keep callbacks short, handle timeouts |
| Actions | Long operations | Provide feedback, handle cancellation |
| Topics | Continuous data | Use appropriate QoS for reliability |
| Callback Groups | Concurrency | Use Reentrant for actions, Exclusive for shared resources |

### What You Built
- Service-based configuration system
- Action-based navigation system
- Communication pattern selection system
- Error handling and debugging skills

### Next Steps
Continue to [Python Packages for Robotics](./04-python-packages.md) to learn about organizing and packaging your robotics code effectively.

</div>