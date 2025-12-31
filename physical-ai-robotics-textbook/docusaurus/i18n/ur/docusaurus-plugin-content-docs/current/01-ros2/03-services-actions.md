---
sidebar_position: 3
title: "سروسز اور ایکشنز: ہم وقت اور گول-اورینٹڈ مواصلات"
id: "01-ros2-services-actions"
---

import ReadingTime from '@site/src/components/ReadingTime';


<ReadingTime minutes={6} />

<h1 className="main-heading">سروسز اور ایکشنز: ہم وقت اور گول-اورینٹڈ مواصلات</h1>
<div className="underline-class"></div>

**سیکھنے کے اہداف**:
- • ہم وقت مواصلات کے لیے ROS 2 سروسز نافذ کرنا
- • گول-اورینٹڈ کاموں کے لیے ایکشنز بنانا اور استعمال کرنا
- • مناسب مواصلاتی پیٹرنز ڈیزائن کرنا
- • جوابات، فیڈ بیک، اور غلطیوں کو ہینڈل کرنا
- • عام مسائل کی ڈیبگنگ کرنا

**ضروریات**: ROS 2 آرکیٹیکچر، نوڈس، ٹاپکس | **وقت**: 3-4 گھنٹے

<div className="border-line"></div>

<h2 className="second-heading">تعارف</h2>
<div className="underline-class"></div>

سروسز ہم وقت درخواست-جواب مواصلات فراہم کرتی ہیں۔ ایکشنز فیڈ بیک اور منسوخی کے ساتھ گول-اورینٹڈ کاموں کو فعال کرتی ہیں۔ سروسز فوری کاموں کے لیے مثالی ہیں؛ ایکشنز طویل مدتی کاموں کے لیے۔

<div className="border-line"></div>

<h2 className="second-heading">سروسز</h2>
<div className="underline-class"></div>

فوری نتائج کی ضرورت والے کاموں کے لیے ہم وقت کلائنٹ-سرور پیٹرن۔

<h3 className="third-heading">آرکیٹیکچر</h3>
<div className="underline-class"></div>

- • **سرور**: سروس فراہم کرتا ہے
- • **کلائنٹ**: درخواستیں کرتا ہے
- • **سروس قسم**: درخواست/جواب کی وضاحت کرتا ہے
- • **بلاکنگ کال**: کلائنٹ انتظار کرتا ہے

<h3 className="third-heading">اپنی سروس قسمیں</h3>
<div className="underline-class"></div>
```
# AddTwoInts.srv
int64 a
int64 b
---
int64 sum
```

<h3 className="third-heading">سرور نافذ کاری</h3>
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

<h3 className="third-heading">کلائنٹ نافذ کاری</h3>
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

<h2 className="second-heading">ایکشنز</h2>
<div className="underline-class"></div>

فیڈ بیک اور منسوخی کے ساتھ طویل مدتی کام۔

<h3 className="third-heading">آرکیٹیکچر</h3>
<div className="underline-class"></div>

- • **گول**: کام کی درخواست
- • **فیڈ بیک**: پیشرفت کی اپ ڈیٹس
- • **رزلٹ**: حتمی نتیجہ
- • **منسوخی**: روکنے کی صلاحیت

<h3 className="third-heading">اپنی ایکشن قسمیں</h3>
<div className="underline-class"></div>
```
# Fibonacci.action
int32 order
---
int32[] sequence
---
int32[] partial_sequence
```

<h3 className="third-heading">ایکشن سرور</h3>
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

<h3 className="third-heading">ایکشن کلائنٹ</h3>
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

<h2 className="second-heading">مشقیں</h2>
<div className="underline-class"></div>

**مشق 1.3.1**: روبوٹ کنفیگریشن سروس (⭐⭐, 35-45 منٹ)
- روبوٹ کنفیگ کے لیے اپنی سروس بنائیں
- درستگی کے ساتھ سرور نافذ کریں
- غلطی کے انتظام کے ساتھ کلائنٹ بنائیں

**مشق 1.3.2**: نیویگیشن ایکشن (⭐⭐⭐, 50-65 منٹ)
- نیویگیشن ایکشن قسم بنائیں
- فیڈ بیک کے ساتھ سرور نافذ کریں
- منسوخی کے ساتھ کلائنٹ بنائیں

**مشق 1.3.3**: مواصلاتی پیٹرنز (⭐⭐, 40-50 منٹ)
- ہائبرڈ سسٹم نافذ کریں
- مناسب پیٹرنز استعمال کریں
- کارکردگی کو بہتر بنائیں

<div className="border-line"></div>

<h2 className="second-heading">عام مسائل</h2>
<div className="underline-class"></div>

**سروس دریافت**:
```bash
echo $ROS_DOMAIN_ID
ros2 service list
```

**ایکشن مواصلات**:
```python
action_server = ActionServer(
    node, ActionType, 'action',
    callback_group=ReentrantCallbackGroup()
)
```

**ٹائم آوٹس**:
```python
future = client.call_async(request)
# بلاکنگ کے بجائے کال بیکس استعمال کریں
```

**کال بیک گروپس**:
```python
from rclpy.callback_groups import ReentrantCallbackGroup
# متوازی رسائی کے لیے
```

<div className="border-line"></div>

<h2 className="second-heading">کب استعمال کریں</h2>
<div className="underline-class"></div>

| پیٹرن | استعمال کا معاملہ | خصوصیات |
|---------|----------|-----------------|
| **ٹاپکس** | جاری ڈیٹا | اسینکرونس، ون ٹو مین |
| **سروسز** | درخواست-جواب | ہم وقت، ون ٹو ون |
| **ایکشنز** | طویل مدتی | اسینکرونس، فیڈ بیک |

**ٹاپکس استعمال کریں برائے**: سینسرز، اسٹیٹ اپ ڈیٹس، مانیٹرنگ
**سروسز استعمال کریں برائے**: کیلیبریشن، کنفیگ، سوالات
**ایکشنز استعمال کریں برائے**: نیویگیشن، مینوپولیشن، طویل کام

<div className="border-line"></div>

<h2 className="second-heading">بہترین مشقیں</h2>
<div className="underline-class"></div>

- • سروس کالز کو چھوٹا رکھیں
- • ٹائم آوٹس استعمال کریں
- • معنی خیز فیڈ بیک فراہم کریں
- • منسوخی کو ہینڈل کریں
- • مناسب کال بیک گروپس استعمال کریں
- • جواب کے اوقات کو مانیٹر کریں

<div className="border-line"></div>

<h2 className="second-heading">خلاصہ</h2>
<div className="underline-class"></div>

سروسز فوری کاموں کے لیے ہم وقت درخواست-جواب کو فعال کرتی ہیں۔ ایکشنز فیڈ بیک کے ساتھ طویل مدتی کاموں کے لیے گول-فیڈ بیک-رزلٹ پیٹرن فراہم کرتی ہیں۔ کام کی خصوصیات کے مطابق پیٹرنز کا انتخاب کریں۔

**اہم نکات**:
- • فوری نتائج کے لیے سروسز
- • فیڈ بیک کے ساتھ طویل کاموں کے لیے ایکشنز
- • متوازی کارکردگی کے لیے مناسب کال بیک گروپس
- • غلطی کا انتظام انتہائی ضروری ہے

<h2 className="second-heading">وسائل</h2>
<div className="underline-class"></div>

- • [ROS 2 سروسز](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html)
- • [ROS 2 ایکشنز](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-An-Action-Server-Client/Py.html)