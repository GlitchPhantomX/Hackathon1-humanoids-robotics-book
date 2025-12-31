---
sidebar_position: 4
title: 'ادراک: AI-پاورڈ سینسنگ'
description: 'روبوٹکس اطلاقیات کے لیے AI اور ڈیپ لرننگ کا استعمال کرتے ہوئے اعلیٰ ادراک سسٹم'
---

import ReadingTime from '@site/src/components/ReadingTime';
import ViewToggle from '@site/src/components/ViewToggle';

<ReadingTime minutes={20} />
<!-- <ViewToggle /> -->

<h1 className="main-heading">ادراک: AI-پاورڈ سینسنگ اور سمجھ</h1>

<div className="underline-class"></div>

<h2 className="second-heading">سیکھنے کے اہداف</h2>

<div className="border-line"></div>

اس باب کے اختتام تک، آپ کے اہل ہوں گے:
- • روبوٹکس کے لیے AI-پاورڈ ادراک پائپ لائنز نافذ کرنا
- • ہارڈ ویئر-ایکسلریٹڈ کمپیوٹر وژن کے لیے Isaac ROS استعمال کرنا
- • متعدد سینسر ماڈلیٹیز کو انضمام کرنے والے ادراک سسٹم ڈیزائن کرنا
- • آبجیکٹ ڈیٹیکشن اور ریکوگنیشن کے لیے ڈیپ لرننگ تکنیکیں لاگو کرنا
- • ریل ٹائم روبوٹکس اطلاقیات کے لیے ادراک الگورتھم کو بہتر بنانا
- • ادراک سسٹم کارکردگی اور درستگی کا جائزہ لینا

<div className="border-line"></div>

<h2 className="second-heading">مشقیں</h2>

<div className="border-line"></div>

<details>
<summary>مشق 3.4.1: Isaac ROS ادراک پائپ لائن سیٹ اپ (⭐, ~30 منٹ)</summary>

### مشق 3.4.1: Isaac ROS ادراک پائپ لائن سیٹ اپ
**پیچیدگی**: ⭐ | **وقت**: 30 منٹ

#### کام
GPU ایکسلریشن کے ساتھ Isaac ROS ادراک پائپ لائن سیٹ اپ کریں

#### ٹیسٹ کمانڈز
```bash
# انسٹالیشن چیک کریں
apt list --installed | grep "isaac-ros"
nvidia-smi

# پائپ لائن لانچ کریں
ros2 launch isaac_ros_perceptor isaac_ros_perceptor.launch.py

# ڈیٹیکشنز ٹیسٹ کریں
ros2 topic echo /isaac_ros/detections
ros2 topic hz /isaac_ros/detections
```

#### کامیابی کے معیار
- [ ] Isaac ROS پیکیجز انسٹال ہیں
- [ ] GPU ایکسلریشن فعال ہے
- [ ] ڈیٹیکشن ٹوپکس پبلش ہو رہے ہیں
- [ ] ریل ٹائم کارکردگی حاصل کی گئی

</details>

<details>
<summary>مشق 3.4.2: ملٹی-سینسر فیوژن (⭐⭐, ~45 منٹ)</summary>

### مشق 3.4.2: بہتر ادراک کے لیے ملٹی-سینسر فیوژن
**پیچیدگی**: ⭐⭐ | **وقت**: 45 منٹ

#### کام
فیوژن کے لیے کیمرہ + LIDAR کا انضمام

#### ٹیسٹ کمانڈز
```bash
# فیوژن لانچ کریں
ros2 launch isaac_ros_fusion multi_sensor_fusion.launch.py

# ہم وقت سینسرز مانیٹر کریں
ros2 topic hz /synchronized/camera/image_rect_color
ros2 topic hz /synchronized/lidar/points

# فیوژن ڈیٹیکشنز چیک کریں
ros2 topic echo /fused_detections
```

#### کامیابی کے معیار
- [ ] سینسرز مناسب طریقے سے ہم وقت ہیں
- [ ] فیوژن درستگی میں بہتری ہے
- [ ] ویژولائزیشن نتائج دکھاتا ہے
- [ ] کارکردگی قابل قبول ہے

</details>

<details>
<summary>مشق 3.4.3: مصنوعی ڈیٹا جنریشن (⭐⭐⭐, ~60 منٹ)</summary>

### مشق 3.4.3: ادراک تربیت کے لیے مصنوعی ڈیٹا جنریشن
**پیچیدگی**: ⭐⭐⭐ | **وقت**: 60 منٹ

#### کام
Isaac Sim میں متنوع تربیتی ڈیٹا سیٹس تخلیق کریں

#### ٹیسٹ کمانڈز
```bash
# Isaac Sim ڈیٹا جنریشن لانچ کریں
isaac-sim --exec "from examples.synthetic_data_gen import run_data_generation"

# تیار کردہ ڈیٹا چیک کریں
ls -la /generated_datasets/perception_training/

# ڈیٹا کی معیار کی تصدیق کریں
python3 -c "
import cv2
img = cv2.imread('/generated_datasets/rgb/frame_000001.png')
print('Image shape:', img.shape)
"
```

#### کامیابی کے معیار
- [ ] متنوع مناظر تیار ہوتے ہیں
- [ ] ملٹی-موڈل ڈیٹا حاصل کیا جاتا ہے
- [ ] لیبلز درست طریقے سے تیار ہوتے ہیں
- [ ] مصنوعی-سے-حقیقی ٹرانسفر کی توثیق ہوتی ہے

</details>

<div className="border-line"></div>

<h2 className="second-heading">ٹربل شوٹنگ</h2>

<div className="border-line"></div>

<details>
<summary>ٹربل شوٹنگ: Isaac ROS ادراک کے مسائل</summary>

<h3 className="third-heading">ٹربل شوٹنگ: Isaac ROS ادراک کے مسائل</h3>

<div className="border-line"></div>

<h4 className="fourth-heading">مسئلہ: نوڈس شروع ہونے میں ناکام ہوتے ہیں</h4>

<div className="border-line"></div>

**علامات**:
- • ادراک نوڈس ابتداء میں کریش ہوتے ہیں
- • GPU ایکسلریشن کا پتہ نہیں چلتا
- • CUDA رن ٹائم کی خرابیاں

<div className="border-line"></div>

**حل**:
```bash
# انسٹالیشن کی تصدیق کریں
ros2 pkg list | grep isaac_ros
nvidia-smi
nvcc --version

# انحصار انسٹال کریں
rosdep install --from-paths src/isaac_ros --ignore-src -r -y
```

<div className="border-line"></div>

<h4 className="fourth-heading">مسئلہ: کم ڈیٹیکشن درستگی</h4>

<div className="border-line"></div>

**علامات**:
- • کم درستگی یا زیادہ جھوٹی مثبت
- • سست پروسیسنگ رفتاریں
- • زیادہ GPU/CPU استعمال

<div className="border-line"></div>

**حل**:
```bash
# پیرامیٹرز ایڈجسٹ کریں
ros2 param set /isaac_ros_detection confidence_threshold 0.5
ros2 param set /isaac_ros_detection input_width 640

# کارکردگی مانیٹر کریں
nvidia-smi dmon -s u -d 1
ros2 topic hz /isaac_ros/detections
```

<div className="border-line"></div>

<h4 className="fourth-heading">مسئلہ: سینسر ہم وقت سازی کے مسائل</h4>

<div className="border-line"></div>

**علامات**:
- • کیمرہ/لیڈار ڈیٹا میل نہیں کھاتا
- • ٹائم اسٹیمپ کی خرابیاں
- • غیر مسلسل فیوژن کے نتائج

<div className="border-line"></div>

**حل**:
```bash
# سینسر کی شرح چیک کریں
ros2 topic hz /camera/image_rect_color
ros2 topic hz /lidar/points

# ہم وقت کی برداشت ایڈجسٹ کریں
ros2 param set /sensor_fusion_sync time_tolerance 0.15
```

</details>

<div className="border-line"></div>

<h2 className="second-heading">ادراک کے بنیادیات</h2>

<div className="border-line"></div>

<h3 className="third-heading">جائزہ</h3>

<div className="border-line"></div>

روبوٹک ادراک ماحول کو سمجھنے کے لیے سینسر ڈیٹا کی تشریح کرتا ہے:
- • **سینسر ڈیٹا ایکویزیشن**: کیمرہ، LIDAR، ریڈار، IMU
- • **پری پروسیسنگ**: فلٹرنگ، ریکٹیفکیشن، نارملائزیشن
- • **فیچر ایکسٹریکشن**: ایج، کورنر، کی پوائنٹس
- • **سمجھ**: آبجیکٹ ڈیٹیکشن، سیگمینٹیشن، پوز ایسٹیمیشن
- • **فیصلہ سازی**: نیویگیشن، مینوپولیشن منصوبہ بندی

<div className="border-line"></div>

<h3 className="third-heading">ادراک کے کاموں کی اقسام</h3>

<div className="border-line"></div>

- • **آبجیکٹ ڈیٹیکشن**: اشیاء کی شناخت اور مقام کاری
- • **سیمینٹک سیگمینٹیشن**: ہر پکسل کی ت Classification
- • **انسٹینس سیگمینٹیشن**: انفرادی اشیاء کو الگ کرنا
- • **پوز ایسٹیمیشن**: 6D آبجیکٹ پوزز کا تعین
- • **منظر کی سمجھ**: مجموعی سیاق و سباق کی تشریح
- • **سرگرمی کی پہچان**: انسانی اعمال کو سمجھنا

<div className="border-line"></div>

<h2 className="second-heading">کوڈ کی مثالیں</h2>

<div className="border-line"></div>

<h3 className="third-heading">Isaac ROS ادراک</h3>

<div className="border-line"></div>

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray

class IsaacROSPeception(Node):
    def __init__(self):
        super().__init__('perception')
        self.sub = self.create_subscription(
            Image, '/camera/image', self.callback, 10)
        self.pub = self.create_publisher(
            Detection2DArray, '/detections', 10)

    def callback(self, msg):
        detections = self.detect(msg)
        self.pub.publish(detections)
```

<div className="border-line"></div>

<h3 className="third-heading">ملٹی-سینسر فیوژن</h3>

<div className="border-line"></div>

```python
from message_filters import ApproximateTimeSynchronizer, Subscriber

class SensorFusion(Node):
    def __init__(self):
        super().__init__('fusion')
        cam = Subscriber(self, Image, '/camera/image')
        lidar = Subscriber(self, PointCloud2, '/lidar/points')

        sync = ApproximateTimeSynchronizer([cam, lidar], 10, 0.1)
        sync.registerCallback(self.fuse_callback)

    def fuse_callback(self, cam_msg, lidar_msg):
        fused = self.process(cam_msg, lidar_msg)
```

<div className="border-line"></div>

<h3 className="third-heading">مصنوعی ڈیٹا جنریشن</h3>

<div className="border-line"></div>

```python
import omni
from omni.isaac.core import World

class DataGenerator:
    def __init__(self, output_dir="dataset"):
        self.world = World()
        self.output_dir = output_dir

    def generate(self, num_frames=100):
        for i in range(num_frames):
            self.world.step(render=True)
            self.capture_frame(i)
```

<div className="border-line"></div>

<h2 className="second-heading">بہترین مشقیں</h2>

<div className="border-line"></div>

<h3 className="third-heading">سسٹم ڈیزائن</h3>

<div className="border-line"></div>

- • **ماڈیولر آرکیٹیکچر**: قابل تبدیل اجزاء
- • **ریل ٹائم پروسیسنگ**: کارکردگی کے لیے بہتر بنانا
- • **مضبوطی**: سینسر کی ناکامیوں کو نرمی سے ہینڈل کرنا
- • **سکیل ایبلیٹی**: اضافی سینسرز/صلاحیات کی حمایت
- • **کیلیبریشن**: درست سینسر کیلیبریشن برقرار رکھنا

<div className="border-line"></div>

<h3 className="third-heading">کارکردگی کی بہتری</h3>

<div className="border-line"></div>

- • **GPU استعمال**: ہارڈ ویئر ایکسلریشن کو زیادہ سے زیادہ استعمال کریں
- • **میموری مینجمنٹ**: موثر GPU/سسٹم میموری استعمال
- • **پائپ لائن پیراللزم**: ممکنہ جگہوں پر متوازی پروسیسنگ
- • **ایڈاپٹیو پروسیسنگ**: منظر کی پیچیدگی کے مطابق ایڈجسٹ کریں
- • **ریسورس مانیٹرنگ**: سسٹم کارکردگی کو مسلسل ٹریک کریں

<div className="border-line"></div>

<h2 className="second-heading">خلاصہ</h2>

<div className="border-line"></div>

ادراک سسٹم روبوٹس کو AI-پاورڈ سینسنگ کے ذریعے ماحول کو سمجھنے کی اجازت دیتا ہے۔ Isaac ROS GPU-ایکسلریٹڈ پروسیسنگ فراہم کرتا ہے، جبکہ Isaac Sim تربیتی ڈیٹا تیار کرتا ہے۔ کامیابی کے لیے سینسر فیوژن، بہتر ڈیپ لرننگ ماڈلز، اور مضبوط روبوٹکس اطلاقیات کے لیے ایڈاپٹیو ریل ٹائم پروسیسنگ کی ضرورت ہوتی ہے۔