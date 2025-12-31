---
sidebar_position: 2
title: "सिस्टम वास्तुकला डिज़ाइन"
description: "ह्यूमनॉइड रोबोटिक्स के लिए सिस्टम वास्तुकला डिज़ाइन करना"
---

import ReadingTime from '@site/src/components/ReadingTime';

<ReadingTime minutes={25} />


<h1 className="main-heading">सिस्टम वास्तुकला डिज़ाइन</h1>
<div className="underline-class"></div>

<div className="full-content">

<div className="border-line"></div>

<h2 className="second-heading">सीखने के उद्देश्य</h2>
<div className="underline-class"></div>

- ह्यूमनॉइड रोबोट के लिए व्यापक सिस्टम वास्तुकला डिज़ाइन करना
- रोबोटिक्स एकीकरण के लिए वास्तुकला पैटर्न लागू करना
- मॉड्यूलर, स्केलेबल, बनाए रखने योग्य सॉफ्टवेयर बनाना
- घटकों के बीच संचार पैटर्न लागू करना
- सुरक्षा और विश्वसनीयता तंत्र स्थापित करना

<div className="border-line"></div>

<h2 className="second-heading">परिचय</h2>
<div className="underline-class"></div>

सिस्टम वास्तुकला निर्धारित करती है कि घटक कैसे इंटरैक्ट, संचार और समन्वय करते हैं। रोबोटिक्स वास्तुकला को रीयल-टाइम बाधाओं, सुरक्षा-महत्वपूर्ण संचालन, और विविध हार्डवेयर/सॉफ्टवेयर एकीकरण को संभालना चाहिए। वास्तुकला विजन-भाषा-एक्शन पैराडाइम का उपयोग करके धारणा, संज्ञान और क्रिया को जोड़ते हुए मॉड्यूलरता को तंग एकीकरण के साथ संतुलित करती है।

<div className="border-line"></div>

<h2 className="second-heading">वास्तुकला सिद्धांत</h2>
<div className="underline-class"></div>

<h3 className="third-heading">घटक-आधारित डिज़ाइन</h3>

```python
from abc import ABC, abstractmethod
import logging

class RobotComponent(ABC):
    def __init__(self, name: str):
        self.name = name
        self.logger = logging.getLogger(f"Robot.{name}")
        self.active = False

    @abstractmethod
    def initialize(self) -> bool:
        pass

    @abstractmethod
    def execute(self) -> bool:
        pass

    @abstractmethod
    def shutdown(self) -> bool:
        pass

    def start(self):
        if self.initialize():
            self.active = True
            return True
        return False

class PerceptionComponent(RobotComponent):
    def __init__(self, name: str):
        super().__init__(name)
        self.sensors = {}

    def initialize(self):
        self.sensors['camera'] = MockCamera()
        self.sensors['lidar'] = MockLidar()
        return True

    def execute(self):
        sensor_data = {name: sensor.capture() for name, sensor in self.sensors.items()}
        return True

    def shutdown(self):
        for sensor in self.sensors.values():
            sensor.shutdown()
        return True
```

<div className="border-line"></div>

<h3 className="third-heading">परत वास्तुकला</h3>

```python
class RobotArchitecture:
    def __init__(self):
        self.layers = {
            'application': ApplicationLayer(),
            'planning': PlanningLayer(),
            'control': ControlLayer(),
            'hardware': HardwareLayer()
        }
        self.comm_bus = CommunicationBus()
        self.safety = SafetyManager()

    def initialize(self):
        return all(layer.initialize() for layer in self.layers.values())

    def execute_cycle(self):
        if not self.safety.is_safe_to_operate():
            self.safety.emergency_stop()
            return False

        # क्रम में निष्पादित करें
        for layer in ['hardware', 'control', 'planning', 'application']:
            if not self.layers[layer].execute():
                return False

        self.comm_bus.update()
        return True

class ApplicationLayer:
    def __init__(self):
        self.task_manager = TaskManager()
        self.user_interface = UserInterface()

    def initialize(self):
        return self.task_manager.initialize() and self.user_interface.initialize()

    def execute(self):
        commands = self.user_interface.get_commands()
        for cmd in commands:
            self.task_manager.add_task(cmd)
        return True

class ControlLayer:
    def __init__(self):
        self.motion_controller = MotionController()
        self.state_estimator = StateEstimator()

    def execute(self):
        state = self.state_estimator.estimate()
        commands = self.motion_controller.generate_commands(state)
        return True
```

<div className="border-line"></div>

<h2 className="second-heading">संचार वास्तुकला</h2>
<div className="underline-class"></div>

<h3 className="third-heading">संदेश-आधारित संचार</h3>

```python
from dataclasses import dataclass
from enum import Enum
import queue
import threading

class MessageType(Enum):
    SENSOR_DATA = "sensor_data"
    PLANNING_REQUEST = "planning_request"
    EXECUTION_COMMAND = "execution_command"
    SAFETY_ALERT = "safety_alert"

@dataclass
class RobotMessage:
    msg_type: MessageType
    source: str
    destination: str
    timestamp: float
    data: any
    priority: int = 0

class CommunicationBus:
    def __init__(self):
        self.message_queues = {}
        self.subscribers = {}
        self.lock = threading.RLock()

    def subscribe(self, msg_type, callback, component_name):
        if msg_type not in self.subscribers:
            self.subscribers[msg_type] = []
        self.subscribers[msg_type].append((callback, component_name))

    def publish(self, message):
        with self.lock:
            if message.msg_type in self.subscribers:
                for callback, _ in self.subscribers[message.msg_type]:
                    callback(message)

            if message.destination != "broadcast":
                if message.destination not in self.message_queues:
                    self.message_queues[message.destination] = queue.PriorityQueue()
                self.message_queues[message.destination].put((-message.priority, message))
        return True

class ComponentCommunicator:
    def __init__(self, name, comm_bus):
        self.name = name
        self.comm_bus = comm_bus

    def send_message(self, msg_type, destination, data, priority=0):
        msg = RobotMessage(msg_type, self.name, destination, time.time(), data, priority)
        return self.comm_bus.publish(msg)
```

<div className="border-line"></div>

<h3 className="third-heading">सेवा-आधारित संचार</h3>

```python
class ServiceRegistry:
    def __init__(self):
        self.services = {}
        self.lock = threading.RLock()

    def register_service(self, service_name, service_callable):
        with self.lock:
            self.services[service_name] = service_callable
        return True

    def call_service(self, service_name, request_data):
        with self.lock:
            if service_name not in self.services:
                raise ValueError(f"Service {service_name} not found")
            return self.services[service_name](request_data)

class NavigationService:
    def __init__(self, service_registry):
        self.registry = service_registry
        self.path_planner = PathPlanner()

    def initialize(self):
        self.registry.register_service("plan_path", self.plan_path_service)
        self.registry.register_service("navigate_to", self.navigate_to_service)
        return True

    def plan_path_service(self, request):
        try:
            path = self.path_planner.plan(request['start'], request['goal'])
            return {'success': True, 'path': path}
        except Exception as e:
            return {'success': False, 'error': str(e)}
```

<div className="border-line"></div>

<h2 className="second-heading">Isaac एकीकरण</h2>
<div className="underline-class"></div>

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class IsaacIntegrationLayer:
    def __init__(self):
        self.isaac_nodes = []
        rclpy.init()

    def initialize_components(self):
        self.isaac_nodes.append(IsaacPerceptionNode())
        self.isaac_nodes.append(IsaacNavigationNode())
        return True

    def start_nodes(self):
        for node in self.isaac_nodes:
            thread = threading.Thread(target=rclpy.spin, args=(node,))
            thread.daemon = True
            thread.start()

class IsaacPerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        self.perception_pub = self.create_publisher(String, '/perception_results', 10)
        self.image_sub = self.create_subscription(Image, '/camera/image', self.image_cb, 10)
        self.object_detector = MockObjectDetector()

    def image_cb(self, msg):
        detections = self.object_detector.detect(msg)
        result_msg = String()
        result_msg.data = json.dumps(detections)
        self.perception_pub.publish(result_msg)

class IsaacNavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal', self.goal_cb, 10)
        self.planner = GlobalPlanner()

    def goal_cb(self, msg):
        path = self.planner.plan(msg.pose)
        if path:
            self.execute_navigation(path)
```

<div className="border-line"></div>

<h2 className="second-heading">सुरक्षा वास्तुकला</h2>
<div className="underline-class"></div>

```python
class SafetyManager:
    def __init__(self):
        self.safety_systems = {
            'collision': CollisionSafetySystem(),
            'velocity': VelocitySafetySystem(),
            'human_awareness': HumanAwarenessSafetySystem()
        }
        self.emergency_stop = False

    def is_safe_to_operate(self):
        if self.emergency_stop:
            return False

        for system in self.safety_systems.values():
            if not system.is_safe():
                return False
        return True

    def emergency_stop(self):
        self.emergency_stop = True
        for system in self.safety_systems.values():
            system.emergency_stop()
        return True

class CollisionSafetySystem:
    def __init__(self):
        self.min_distance = 0.3
        self.collision_threshold = 0.1

    def is_safe(self):
        distances = self.get_proximity_distances()
        return all(d > self.collision_threshold for d in distances)

    def get_proximity_distances(self):
        return [0.5, 0.8, 0.3, 1.2]  # Mock distances

class VelocitySafetySystem:
    def __init__(self):
        self.max_linear_vel = 1.0
        self.max_angular_vel = 0.5
        self.current_vel = {'linear': 0.0, 'angular': 0.0}

    def is_safe(self):
        return (abs(self.current_vel['linear']) <= self.max_linear_vel and
                abs(self.current_vel['angular']) <= self.max_angular_vel)

class HumanAwarenessSafetySystem:
    def __init__(self):
        self.min_human_distance = 1.0

    def is_safe(self):
        humans = self.detect_humans()
        distances = [self.calc_distance(h) for h in humans]
        return all(d >= self.min_human_distance for d in distances)

    def detect_humans(self):
        return [{'x': 1.5, 'y': 0.0}]  # Mock detection
```

<div className="border-line"></div>

<h2 className="second-heading">प्रदर्शन निगरानी</h2>
<div className="underline-class"></div>

```python
import psutil

class PerformanceMonitor:
    def __init__(self):
        self.component_timings = {}
        self.thresholds = {
            'cpu_usage': 80.0,
            'memory_usage': 80.0,
            'latency': 0.1
        }

    def collect_metrics(self):
        return {
            'timestamp': time.time(),
            'cpu_percent': psutil.cpu_percent(),
            'memory_percent': psutil.virtual_memory().percent,
            'component_timings': self.component_timings.copy()
        }

    def check_violations(self, metrics):
        violations = []

        if metrics['cpu_percent'] > self.thresholds['cpu_usage']:
            violations.append(f"CPU: {metrics['cpu_percent']:.1f}%")

        if metrics['memory_percent'] > self.thresholds['memory_usage']:
            violations.append(f"Memory: {metrics['memory_percent']:.1f}%")

        return violations

    def record_timing(self, component, execution_time):
        if component not in self.component_timings:
            self.component_timings[component] = {
                'total': 0.0, 'count': 0, 'avg': 0.0
            }

        timing = self.component_timings[component]
        timing['total'] += execution_time
        timing['count'] += 1
        timing['avg'] = timing['total'] / timing['count']

class RealTimeScheduler:
    def __init__(self):
        self.tasks = []
        self.running = False

    def add_task(self, task, period, priority=0, name=""):
        self.tasks.append({
            'task': task, 'period': period, 'priority': priority,
            'name': name, 'next_run': time.time()
        })

    def start(self):
        self.running = True
        while self.running:
            current = time.time()
            ready = [t for t in self.tasks if current >= t['next_run']]
            ready.sort(key=lambda x: x['priority'], reverse=True)

            for task_info in ready:
                task_info['task']()
                task_info['next_run'] = current + task_info['period']

            time.sleep(0.001)
```

<div className="border-line"></div>

<h2 className="second-heading">सारांश</h2>
<div className="underline-class"></div>

**मुख्य वास्तुकला तत्व:**

1. **मॉड्यूलर डिज़ाइन**: घटक विशिष्ट कार्यक्षमता को स्पष्ट इंटरफेस के साथ संलग्न करते हैं
2. **परत वास्तुकला**: एप्लिकेशन, योजना, नियंत्रण और हार्डवेयर परतों का पृथक्करण
3. **संचार प्रणालियां**: समन्वय के लिए संदेश-आधारित और सेवा-आधारित पैटर्न
4. **Isaac एकीकरण**: NVIDIA हार्डवेयर एक्सेलरेशन का लाभ उठाने वाले विशिष्ट घटक
5. **सुरक्षा प्रणालियां**: व्यापक टक्कर, वेग और मानव जागरूकता निगरानी
6. **प्रदर्शन निगरानी**: समय बाधाओं के लिए रीयल-टाइम निगरानी और अनुसूची

**सफलता कारक**: उचित घटक इंटरफेस, सिस्टम में सूचना प्रवाह, सुरक्षा-प्रथम डिज़ाइन, और रीयल-टाइम प्रदर्शन गारंटी।

<div className="border-line"></div>

<h2 className="second-heading">अभ्यास</h2>
<div className="underline-class"></div>

1. अपने रोबोट के लिए घटक वास्तुकला डिज़ाइन करें
2. संचार के लिए इंटरफेस विनिर्देश बनाएं
3. सुरक्षा प्रणाली एकीकरण की योजना बनाएं
4. प्रदर्शन निगरानी रणनीतियां डिज़ाइन करें
5. मूल संचार वास्तुकला लागू करें

<div className="border-line"></div>

<h2 className="second-heading">आगे की पढ़ाई</h2>
<div className="underline-class"></div>

- "Software Architecture in Practice" बैस एट अल. द्वारा
- "Real-Time Systems" कोपेट्ज़ द्वारा
- "Designing Data-Intensive Applications" क्लेपमैन द्वारा
- "Robotics Middleware" खैरखाहन एट अल. द्वारा

</div>