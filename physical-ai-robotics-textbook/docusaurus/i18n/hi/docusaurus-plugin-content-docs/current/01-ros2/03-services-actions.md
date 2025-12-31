---
sidebar_position: 3
title: "सेवाएं और क्रियाएं: समतुल्यकालिक और लक्ष्य-उन्मुख संचार"
id: "01-ros2-services-actions"
---

import ReadingTime from '@site/src/components/ReadingTime';

<ReadingTime minutes={6} />

<h1 className="main-heading">सेवाएं और क्रियाएं: समतुल्यकालिक और लक्ष्य-उन्मुख संचार</h1>
<div className="underline-class"></div>

**सीखने के उद्देश्य**:
- • समतुल्यकालिक संचार के लिए आरओएस 2 सेवाएं लागू करें
- • लक्ष्य-उन्मुख कार्यों के लिए क्रियाएं बनाएं और उपयोग करें
- • उपयुक्त संचार पैटर्न डिज़ाइन करें
- • प्रतिक्रियाओं, प्रतिक्रिया और त्रुटियों को संभालें
- • सामान्य समस्याओं का डिबग करें

**पूर्वापेक्षाएं**: आरओएस 2 संरचना, नोड्स, टॉपिक्स | **समय**: 3-4 घंटे

<div className="border-line"></div>

<h2 className="second-heading">परिचय</h2>
<div className="underline-class"></div>

सेवाएं समतुल्यकालिक अनुरोध-प्रतिक्रिया संचार प्रदान करती हैं। क्रियाएं प्रतिक्रिया और रद्दीकरण के साथ लक्ष्य-उन्मुख कार्यों को सक्षम बनाती हैं। सेवाएं तत्काल संचालन के लिए आदर्श हैं; क्रियाएं लंबे समय तक चलने वाले कार्यों के लिए।

<div className="border-line"></div>

<h2 className="second-heading">सेवाएं</h2>
<div className="underline-class"></div>

तत्काल परिणामों की आवश्यकता वाले संचालन के लिए समतुल्यकालिक क्लाइंट-सर्वर पैटर्न।

<h3 className="third-heading">संरचना</h3>
<div className="underline-class"></div>

- • **सर्वर**: सेवा प्रदान करता है
- • **क्लाइंट**: अनुरोध करता है
- • **सेवा प्रकार**: अनुरोध/प्रतिक्रिया परिभाषित करता है
- • **ब्लॉकिंग कॉल**: क्लाइंट प्रतीक्षा करता है

<h3 className="third-heading">कस्टम सेवा प्रकार</h3>
<div className="underline-class"></div>
```
# AddTwoInts.srv
int64 a
int64 b
---
int64 sum
```

<h3 className="third-heading">सर्वर कार्यान्वयन</h3>
<div className="underline-class"></div>

<h4 className="fourth-heading">पायथन</h4>
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

<h4 className="fourth-heading">सी++</h4>
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

<h3 className="third-heading">क्लाइंट कार्यान्वयन</h3>
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

<h2 className="second-heading">क्रियाएं</h2>
<div className="underline-class"></div>

प्रतिक्रिया और रद्दीकरण के साथ लंबे समय तक चलने वाले कार्य।

<h3 className="third-heading">संरचना</h3>
<div className="underline-class"></div>

- • **लक्ष्य**: कार्य अनुरोध
- • **प्रतिक्रिया**: प्रगति अद्यतन
- • **परिणाम**: अंतिम परिणाम
- • **रद्दीकरण**: रोकने की क्षमता

<h3 className="third-heading">कस्टम क्रिया प्रकार</h3>
<div className="underline-class"></div>
```
# Fibonacci.action
int32 order
---
int32[] sequence
---
int32[] partial_sequence
```

<h3 className="third-heading">क्रिया सर्वर</h3>
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

<h3 className="third-heading">क्रिया क्लाइंट</h3>
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

<h2 className="second-heading">अभ्यास</h2>
<div className="underline-class"></div>

**अभ्यास 1.3.1**: रोबोट कॉन्फ़िगरेशन सेवा (⭐⭐, 35-45 मिनट)
- रोबोट कॉन्फ़िगरेशन के लिए कस्टम सेवा बनाएं
- मान्यकरण के साथ सर्वर लागू करें
- त्रुटि संभाल के साथ क्लाइंट बनाएं

**अभ्यास 1.3.2**: नेविगेशन क्रिया (⭐⭐⭐, 50-65 मिनट)
- नेविगेशन क्रिया प्रकार बनाएं
- प्रतिक्रिया के साथ सर्वर लागू करें
- रद्दीकरण के साथ क्लाइंट बनाएं

**अभ्यास 1.3.3**: संचार पैटर्न (⭐⭐, 40-50 मिनट)
- हाइब्रिड सिस्टम लागू करें
- उपयुक्त पैटर्न का उपयोग करें
- प्रदर्शन का अनुकूलन करें

<div className="border-line"></div>

<h2 className="second-heading">सामान्य समस्याएं</h2>
<div className="underline-class"></div>

**सेवा खोज**:
```bash
echo $ROS_DOMAIN_ID
ros2 service list
```

**क्रिया संचार**:
```python
action_server = ActionServer(
    node, ActionType, 'action',
    callback_group=ReentrantCallbackGroup()
)
```

**समय समाप्ति**:
```python
future = client.call_async(request)
# ब्लॉकिंग के बजाय कॉलबैक का उपयोग करें
```

**कॉलबैक समूह**:
```python
from rclpy.callback_groups import ReentrantCallbackGroup
# समवर्ती पहुंच के लिए
```

<div className="border-line"></div>

<h2 className="second-heading">कब उपयोग करें</h2>
<div className="underline-class"></div>

| पैटर्न | उपयोग मामला | विशेषताएं |
|---------|----------|-----------------|
| **टॉपिक्स** | निरंतर डेटा | अतुल्यकालिक, एक-से-कई |
| **सेवाएं** | अनुरोध-प्रतिक्रिया | समतुल्यकालिक, एक-से-एक |
| **क्रियाएं** | लंबे समय तक चलने वाले | अतुल्यकालिक, प्रतिक्रिया |

**टॉपिक्स का उपयोग करें**: सेंसर, स्थिति अद्यतन, निगरानी
**सेवाओं का उपयोग करें**: कैलिब्रेशन, कॉन्फ़िग, क्वेरी
**क्रियाओं का उपयोग करें**: नेविगेशन, मैनिपुलेशन, लंबे कार्य

<div className="border-line"></div>

<h2 className="second-heading">सर्वोत्तम प्रथाएं</h2>
<div className="underline-class"></div>

- • सेवा कॉल छोटे रखें
- • समय समाप्ति का उपयोग करें
- • अर्थपूर्ण प्रतिक्रिया प्रदान करें
- • रद्दीकरण संभालें
- • उपयुक्त कॉलबैक समूह का उपयोग करें
- • प्रतिक्रिया समय की निगरानी करें

<div className="border-line"></div>

<h2 className="second-heading">सारांश</h2>
<div className="underline-class"></div>

सेवाएं तत्काल संचालन के लिए समतुल्यकालिक अनुरोध-प्रतिक्रिया सक्षम बनाती हैं। क्रियाएं प्रतिक्रिया के साथ लंबे समय तक चलने वाले कार्यों के लिए लक्ष्य-प्रतिक्रिया-परिणाम पैटर्न प्रदान करती हैं। संचालन विशेषताओं के आधार पर पैटर्न चुनें।

**मुख्य बातें**:
- • तत्काल परिणामों के लिए सेवाएं
- • प्रतिक्रिया के साथ लंबे कार्यों के लिए क्रियाएं
- • समवर्तीता के लिए उचित कॉलबैक समूह
- • त्रुटि संभाल महत्वपूर्ण है

<h2 className="second-heading">संसाधन</h2>
<div className="underline-class"></div>

- • [आरओएस 2 सेवाएं](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html)
- • [आरओएस 2 क्रियाएं](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-An-Action-Server-Client/Py.html)