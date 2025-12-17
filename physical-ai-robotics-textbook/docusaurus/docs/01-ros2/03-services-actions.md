---
sidebar_position: 3
title: "Services and Actions: Synchronous and Goal-Oriented Communication"
id: "01-ros2-services-actions"
---

import ReadingTime from '@site/src/components/ReadingTime';

<ReadingTime minutes={6} />

<h1 className="main-heading">Services and Actions: Synchronous and Goal-Oriented Communication</h1>
<div className="underline-class"></div>

**Learning Objectives**:
- • Implement ROS 2 services for synchronous communication
- • Create and use actions for goal-oriented tasks
- • Design appropriate communication patterns
- • Handle responses, feedback, and errors
- • Debug common issues

**Prerequisites**: ROS 2 architecture, nodes, topics | **Time**: 3-4 hours

<div className="border-line"></div>

<h2 className="second-heading">Introduction</h2>
<div className="underline-class"></div>

Services provide synchronous request-response communication. Actions enable goal-oriented tasks with feedback and cancellation. Services are ideal for immediate operations; actions for long-running tasks.

<div className="border-line"></div>

<h2 className="second-heading">Services</h2>
<div className="underline-class"></div>

Synchronous client-server pattern for operations requiring immediate results.

<h3 className="third-heading">Architecture</h3>
<div className="underline-class"></div>

- • **Server**: Provides service
- • **Client**: Makes requests
- • **Service Type**: Defines request/response
- • **Blocking Call**: Client waits

<h3 className="third-heading">Custom Service Types</h3>
<div className="underline-class"></div>
```
# AddTwoInts.srv
int64 a
int64 b
---
int64 sum
```

<h3 className="third-heading">Server Implementation</h3>
<div className="underline-class"></div>

<h4 className="fourth-heading">Python</h4>
<div className="underline-class"></div>
```python
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class ServiceServer(Node):
    def __init__(self):
        super().__init__('service_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.callback)
    
    def callback(self, request, response):
        response.sum = request.a + request.b
        return response
```

<h4 className="fourth-heading">C++</h4>
<div className="underline-class"></div>
```cpp
class MinimalService : public rclcpp::Node {
public:
    MinimalService() : Node("service") {
        service_ = this->create_service<AddTwoInts>(
            "add_two_ints",
            [this](auto req, auto resp) {
                resp->sum = req->a + req->b;
            });
    }
};
```

<h3 className="third-heading">Client Implementation</h3>
<div className="underline-class"></div>
```python
class ServiceClient(Node):
    def __init__(self):
        super().__init__('client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting...')
    
    def send_request(self, a, b):
        req = AddTwoInts.Request()
        req.a, req.b = a, b
        future = self.cli.call_async(req)
        return future.result()
```

<div className="border-line"></div>

<h2 className="second-heading">Actions</h2>
<div className="underline-class"></div>

Long-running tasks with feedback and cancellation.

<h3 className="third-heading">Architecture</h3>
<div className="underline-class"></div>

- • **Goal**: Task request
- • **Feedback**: Progress updates
- • **Result**: Final outcome
- • **Cancellation**: Stop capability

<h3 className="third-heading">Custom Action Types</h3>
<div className="underline-class"></div>
```
# Fibonacci.action
int32 order
---
int32[] sequence
---
int32[] partial_sequence
```

<h3 className="third-heading">Action Server</h3>
<div className="underline-class"></div>
```python
from rclpy.action import ActionServer
from example_interfaces.action import Fibonacci

class ActionServerClass(Node):
    def __init__(self):
        super().__init__('action_server')
        self._action = ActionServer(self, Fibonacci, 'fibonacci', self.execute)
    
    def execute(self, goal_handle):
        feedback = Fibonacci.Feedback()
        for i in range(goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return Fibonacci.Result()
            goal_handle.publish_feedback(feedback)
        goal_handle.succeed()
        return Fibonacci.Result()
```

<h3 className="third-heading">Action Client</h3>
<div className="underline-class"></div>
```python
from rclpy.action import ActionClient

class ActionClientClass(Node):
    def __init__(self):
        super().__init__('action_client')
        self._client = ActionClient(self, Fibonacci, 'fibonacci')
    
    def send_goal(self, order):
        goal = Fibonacci.Goal()
        goal.order = order
        self._client.wait_for_server()
        future = self._client.send_goal_async(goal, feedback_callback=self.feedback_cb)
    
    def feedback_cb(self, feedback_msg):
        self.get_logger().info(f'Feedback: {feedback_msg.feedback}')
```

<div className="border-line"></div>

<h2 className="second-heading">Exercises</h2>
<div className="underline-class"></div>

**Exercise 1.3.1**: Robot Configuration Service (⭐⭐, 35-45 min)
- Create custom service for robot config
- Implement server with validation
- Create client with error handling

**Exercise 1.3.2**: Navigation Action (⭐⭐⭐, 50-65 min)
- Create navigation action type
- Implement server with feedback
- Create client with cancellation

**Exercise 1.3.3**: Communication Patterns (⭐⭐, 40-50 min)
- Implement hybrid system
- Use appropriate patterns
- Optimize performance

<div className="border-line"></div>

<h2 className="second-heading">Common Issues</h2>
<div className="underline-class"></div>

**Service Discovery**:
```bash
echo $ROS_DOMAIN_ID
ros2 service list
```

**Action Communication**:
```python
action_server = ActionServer(
    node, ActionType, 'action',
    callback_group=ReentrantCallbackGroup()
)
```

**Timeouts**:
```python
future = client.call_async(request)
# Use callbacks instead of blocking
```

**Callback Groups**:
```python
from rclpy.callback_groups import ReentrantCallbackGroup
# For concurrent access
```

<div className="border-line"></div>

<h2 className="second-heading">When to Use</h2>
<div className="underline-class"></div>

| Pattern | Use Case | Characteristics |
|---------|----------|-----------------|
| **Topics** | Continuous data | Asynchronous, one-to-many |
| **Services** | Request-response | Synchronous, one-to-one |
| **Actions** | Long-running | Asynchronous, feedback |

**Use Topics for**: Sensors, state updates, monitoring
**Use Services for**: Calibration, config, queries
**Use Actions for**: Navigation, manipulation, long tasks

<div className="border-line"></div>

<h2 className="second-heading">Best Practices</h2>
<div className="underline-class"></div>

- • Keep service calls short
- • Use timeouts
- • Provide meaningful feedback
- • Handle cancellation
- • Use appropriate callback groups
- • Monitor response times

<div className="border-line"></div>

<h2 className="second-heading">Summary</h2>
<div className="underline-class"></div>

Services enable synchronous request-response for immediate operations. Actions provide goal-feedback-result pattern for long-running tasks with cancellation. Choose patterns based on operation characteristics.

**Key Takeaways**:
- • Services for immediate results
- • Actions for long tasks with feedback
- • Proper callback groups for concurrency
- • Error handling is critical

<h2 className="second-heading">Resources</h2>
<div className="underline-class"></div>

- • [ROS 2 Services](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html)
- • [ROS 2 Actions](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-An-Action-Server-Client/Py.html)