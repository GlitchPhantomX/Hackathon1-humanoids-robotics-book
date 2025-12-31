---
sidebar_position: 1
title: "ROS 2 آرکیٹیکچر اور تصورات"
id: "01-ros2-architecture"
---

import ReadingTime from '@site/src/components/ReadingTime';


<ReadingTime minutes={5} />

<h1 className="main-heading">ROS 2 آرکیٹیکچر اور تصورات</h1>
<div className="underline-class"></div>

**سیکھنے کے اہداف**:
- • ROS 2 کور آرکیٹیکچر کی وضاحت کرنا
- • DDS-بیسڈ مواصلاتی ماڈل کی وضاحت کرنا
- • QoS پروفائلز کو سمجھنا
- • نوڈس، ٹاپکس، سروسز، ایکشنز کی شناخت کرنا
- • ROS 2 ماحول سیٹ اپ کرنا

**ضروریات**: روبوٹکس کی بنیاد، پائی تھون | **وقت**: 2-3 گھنٹے

<div className="border-line"></div>

<h2 className="second-heading">تعارف</h2>
<div className="underline-class"></div>

ROS 2 ہیومنوائڈ روبوٹ کی مواصلات کے لیے مڈل ویئر انفراسٹرکچر فراہم کرتا ہے۔ DDS پر تعمیر کیا گیا، یہ بہتر قابلیت، ریل ٹائم سپورٹ، اور پروڈکشن ڈیپلائمنٹ کی صلاحیتیں فراہم کرتا ہے۔

<div className="border-line"></div>

<h2 className="second-heading">ROS 1 بمقابلہ ROS 2</h2>
<div className="underline-class"></div>

**اہم بہتریاں**:
- • **ریل ٹائم سپورٹ**: اہم ایپلی کیشنز کے لیے بہتر
- • **میلٹی-روبوٹ سسٹم**: بہتر تال میل
- • **پروڈکشن ڈیپلائمنٹ**: سیکورٹی اور استحکام
- • **مڈل ویئر لچک**: DDS ایپلی کیشنز

**اہم فرق**:
- • **مواصلات**: DDS-بیسڈ مڈل ویئر
- • **QoS**: قابل ترتیب قابلیت
- • **سیکورٹی**: بلٹ ان فیچرز
- • **لائف سائیکل**: مضبوط انتظام

<div className="border-line"></div>

<h2 className="second-heading">کور اجزاء</h2>
<div className="underline-class"></div>

<h3 className="third-heading">نوڈس</h3>
<div className="underline-class"></div>

- • الگ الگ عمل
- • انٹر-نوڈ مواصلات
- • ایکسپلیٹڈ فعالیت
- • متعدد زبانوں کی حمایت
```python
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')
        self.get_logger().info('Hello from node!')
```

<h3 className="third-heading">DDS مڈل ویئر</h3>
<div className="underline-class"></div>

- • ڈیٹا-سینٹرک مواصلات
- • پبلیشر-سبسکرائیبر ماڈل
- • خودکار دریافت
- • قابل ترتیب QoS

<h3 className="third-heading">ٹاپکس</h3>
<div className="underline-class"></div>

- • اسینکرونس pub/sub
- • ون ٹو مین مواصلات
- • میسج قسمیں (.msg فائلز)
- • ریل ٹائم قابل

<h3 className="third-heading">سروسز</h3>
<div className="underline-class"></div>

- • سینکرونس درخواست-جواب
- • کلائنٹ-سرور پیٹرن
- • سروس قسمیں (.srv فائلز)
- • بلاکنگ کالز

<h3 className="third-heading">ایکشنز</h3>
<div className="underline-class"></div>

- • گول-فیڈ بیک-رزلٹ پیٹرن
- • طویل مدتی کام
- • منسوخی کی حمایت
- • ایکشن قسمیں (.action فائلز)

<div className="border-line"></div>

<h2 className="second-heading">QoS پروفائلز</h2>
<div className="underline-class"></div>

**قابلیت**: قابل اعتماد (TCP-جیسا) بمقابلہ بہترین کوشش (UDP-جیسا)
**دوام**: ٹرنسجینٹ لوکل بمقابلہ وولیٹائل
**تاریخ**: آخری رکھیں (N میسج) بمقابلہ سب رکھیں
```python
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

qos = QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE
)
```

<div className="border-line"></div>

<h2 className="second-heading">کلائنٹ لائبریریز</h2>
<div className="underline-class"></div>

**rclcpp (C++)**: ہائی-پرفارمنس، ریل ٹائم سسٹم
**rclpy (Python)**: پروٹو ٹائپنگ، تیز ترقی

<div className="border-line"></div>

<h2 className="second-heading">مشقیں</h2>
<div className="underline-class"></div>

**مشق 1.1.1**: ماحول سیٹ اپ (⭐⭐, 25-35 منٹ)
- ROS 2 انسٹالیشن کی تصدیق کریں
- ورک سپیس اور پیکیج بنائیں
- پبلیشر نوڈ نافذ کریں
- ٹیسٹ اور تصدیق کریں

**مشق 1.1.2**: QoS کنفیگریشن (⭐⭐⭐, 35-45 منٹ)
- مختلف QoS کے ساتھ پبلیشرز بنائیں
- قابل اعتماد بمقابلہ بہترین کوشش ٹیسٹ کریں
- کارکردگی کا موازنہ کریں
- استعمال کے معاملات کا تجزیہ کریں

**مشق 1.1.3**: مواصلات کا تجزیہ (⭐⭐, 30-40 منٹ)
- متعدد نوڈس بنائیں
- ROS 2 ٹولز کے ساتھ ٹوپولوجی کا تجزیہ کریں
- فلو دستاویز کریں
- وژوئلائزیشنز بنائیں

<div className="border-line"></div>

<h2 className="second-heading">عام مسائل</h2>
<div className="underline-class"></div>

**نوڈ دریافت**:
```bash
echo $ROS_DOMAIN_ID
export ROS_DOMAIN_ID=0
ros2 node list
```

**QoS میچ نہیں**:
```python
qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
# پبلیشر اور سبسکرائیبر پر میچ کریں
```

**ورک سپیس سیٹ اپ**:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
source install/setup.bash
```

**کارکردگی**:
```python
qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
```

**لائف سائیکل**:
```python
def destroy_node(self):
    if self.timer:
        self.timer.cancel()
    super().destroy_node()
```

<div className="border-line"></div>

<h2 className="second-heading">خلاصہ</h2>
<div className="underline-class"></div>

ROS 2 DDS مڈل ویئر استعمال کرتا ہے جس میں قابل ترتیب QoS پروفائلز ہیں۔ کور اجزاء میں نوڈس (ایگزیکیوشن یونٹس)، ٹاپکس (pub/sub)، سروسز (req/resp)، اور ایکشنز (گول/فیڈ بیک/رزلٹ) شامل ہیں۔

**اہم نکات**:
- • DDS بہتر قابلیت فراہم کرتا ہے
- • QoS مواصلات کو فائن ٹیون کرنے کی اجازت دیتا ہے
- • ریل ٹائم اور میلٹی-روبوٹ سسٹم کی حمایت کرتا ہے
- • مناسب سیٹ اپ ضروری ہے
- • پیٹرنز کو سمجھنا انتہائی ضروری ہے

<h2 className="second-heading">وسائل</h2>
<div className="underline-class"></div>

- • [ROS 2 آرکیٹیکچر](https://docs.ros.org/en/humble/Concepts/About-ROS-2-Architecture.html)
- • [QoS گائیڈ](https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service.html)
- • [ROS 2 ڈیمو](https://github.com/ros2/demos)

**نیویگیشن**: [← پچھلا](../00-introduction/05-syllabus.md) | [اگلا →](./02-nodes-topics.md)

<h2 className="second-heading">فوری حوالہ</h2>
<div className="underline-class"></div>

| جزو | مقصد | بہترین مشق |
|-----------|---------|---------------|
| نوڈس | ایگزیکیوشن یونٹس | واحد ذمہ داری |
| ٹاپکس | اسینک مواصلات | مناسب QoS استعمال کریں |
| سروسز | سینک مواصلات | درخواست/جواب |
| ایکشنز | طویل کام | فیڈ بیک/پیشرفت |
| QoS | ترتیب | pub/sub پروفائلز میچ کریں |