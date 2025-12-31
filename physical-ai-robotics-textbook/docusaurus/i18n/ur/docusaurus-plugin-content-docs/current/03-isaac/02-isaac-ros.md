---
sidebar_position: 2
title: 'Isaac ROS: ہارڈ ویئر ایکسلریٹڈ ادراک'
description: 'Isaac ROS NVIDIA GPUs کا استعمال کرتے ہوئے روبوٹکس ادراک پائپ لائنز کو ہارڈ ویئر ایکسلریشن لاتا ہے'
---

import ReadingTime from '@site/src/components/ReadingTime';
import ViewToggle from '@site/src/components/ViewToggle';


<ReadingTime minutes={20} />
<!-- <ViewToggle /> -->

<h1 className="main-heading">Isaac ROS: ہارڈ ویئر ایکسلریٹڈ روبوٹکس ادراک</h1>

<div className="underline-class"></div>

<h2 className="second-heading">سیکھنے کے اہداف</h2>

<div className="border-line"></div>

اس باب کے اختتام تک، آپ کے اہل ہوں گے:
- • Isaac ROS کے آرکیٹیکچر اور صلاحیات کو سمجھنا
- • ہارڈ ویئر-ایکسلریٹڈ ادراک کے لیے Isaac ROS سیٹ اپ کرنا
- • ایکسلریٹڈ ادراک پائپ لائنز نافذ کرنا
- • Isaac ROS کو ROS 2 اطلاقیات کے ساتھ انضمام کرنا
- • کارکردگی کے لیے ادراک پائپ لائنز کو بہتر بنانا
- • AI-پاورڈ روبوٹکس کے لیے Isaac ROS کا استعمال کرنا

<div className="border-line"></div>

<h2 className="second-heading">مشقیں</h2>

<div className="border-line"></div>

<details>
<summary>مشق 3.2.1: Isaac ROS انسٹالیشن (⭐, ~35 منٹ)</summary>

<h3 className="third-heading">مشق 3.2.1: Isaac ROS انسٹالیشن اور بنیادی پائپ لائن</h3>

<div className="border-line"></div>

**پیچیدگی**: ⭐ | **وقت**: 35 منٹ

<h4 className="fourth-heading">کام</h4>

<div className="border-line"></div>

Isaac ROS کو بنیادی ادراک پائپ لائن کے ساتھ سیٹ اپ کریں

<h4 className="fourth-heading">ٹیسٹ کمانڈز</h4>

<div className="border-line"></div>

```bash
# انسٹالیشن کی تصدیق کریں
dpkg -l | grep isaac-ros
nvidia-smi
ros2 pkg list | grep isaac_ros

# بنیادی نوڈ ٹیسٹ کریں
ros2 run isaac_ros_common test_node

# GPU استعمال چیک کریں
watch -n 1 nvidia-smi
```

<h4 className="fourth-heading">کامیابی کے معیار</h4>

<div className="border-line"></div>

- [ ] Isaac ROS پیکیجز انسٹال ہیں
- [ ] GPU ایکسلریشن فعال ہے
- [ ] بنیادی نوڈ کامیابی سے چلتا ہے
- [ ] کارکردگی کے فوائد نظر آتے ہیں

</details>

<details>
<summary>مشق 3.2.2: گہرائی کی پروسیسنگ پائپ لائن (⭐⭐, ~50 منٹ)</summary>

<h3 className="third-heading">مشق 3.2.2: ایکسلریٹڈ گہرائی کی پروسیسنگ</h3>

<div className="border-line"></div>

**پیچیدگی**: ⭐⭐ | **وقت**: 50 منٹ

<h4 className="fourth-heading">کام</h4>

<div className="border-line"></div>

GPU-ایکسلریٹڈ اسٹیریو گہرائی پائپ لائن بنائیں

<h4 className="fourth-heading">ٹیسٹ کمانڈز</h4>

<div className="border-line"></div>

```bash
# اسٹیریو پروسیسنگ لانچ کریں
ros2 launch isaac_ros_stereo_image_proc stereo_image_proc.launch.py

# آؤٹ پٹ مانیٹر کریں
ros2 topic echo /stereo_camera/disparity
ros2 topic echo /stereo_camera/points

# GPU استعمال کی چیک کریں
nvidia-smi dmon -s u -d 1
```

<h4 className="fourth-heading">کامیابی کے معیار</h4>

<div className="border-line"></div>

- [ ] اسٹیریو ڈیٹا کارآمدی کے ساتھ پروسیس ہوتا ہے
- [ ] گہرائی کے نقشے معیار کے ساتھ تخلیق ہوتے ہیں
- [ ] پوائنٹ کلاؤڈز درست طور پر تخلیق ہوتے ہیں
- [ ] ریل ٹائم کارکردگی حاصل کی جاتی ہے

</details>

<details>
<summary>مشق 3.2.3: AI آبجیکٹ ڈیٹیکشن (⭐⭐⭐, ~65 منٹ)</summary>

<h3 className="third-heading">مشق 3.2.3: TensorRT آبجیکٹ ڈیٹیکشن</h3>

<div className="border-line"></div>

**پیچیدگی**: ⭐⭐⭐ | **وقت**: 65 منٹ

<h4 className="fourth-heading">کام</h4>

<div className="border-line"></div>

TensorRT-ایکسلریٹڈ آبجیکٹ ڈیٹیکشن نافذ کریں

<h4 className="fourth-heading">ٹیسٹ کمانڈز</h4>

<div className="border-line"></div>

```bash
# TensorRT ٹیسٹ کریں
python3 -c "import tensorrt as trt; print('TensorRT دستیاب')"
# ڈیٹیکشن لانچ کریں
ros2 launch isaac_ros_detectnet detectnet.launch.py

# نتائج مانیٹر کریں
ros2 topic echo /detectnet/detections
ros2 topic hz /detectnet/detections
```

<h4 className="fourth-heading">کامیابی کے معیار</h4>

<div className="border-line"></div>

- [ ] TensorRT ماڈل درست طور پر لوڈ ہوتا ہے
- [ ] GPU ایکسلریشن کے ساتھ ڈیٹیکشن چلتا ہے
- [ ] نتائج میں باؤنڈنگ باکسز شامل ہیں
- [ ] ریل ٹائم کارکردگی برقرار رکھی جاتی ہے

</details>

<div className="border-line"></div>

<h2 className="second-heading">ٹربل شوٹنگ</h2>

<div className="border-line"></div>

<details>
<summary>ٹربل شوٹنگ: Isaac ROS کے مسائل</summary>

<h3 className="third-heading">ٹربل شوٹنگ: Isaac ROS کے مسائل</h3>

<div className="border-line"></div>

<h4 className="fourth-heading">مسئلہ: پیکیجز انسٹال ہونے میں ناکام ہوتے ہیں</h4>

<div className="border-line"></div>

**علامات**:
- • انسٹالیشن انحصار کی خرابیوں کے ساتھ ناکام ہوتی ہے
- • CUDA-سے-متعلقہ تعمیر کی خرابیاں
- • انحصار غائب ہیں

<div className="border-line"></div>

**حل**:
```bash
# مطابقت کی تصدیق کریں
nvcc --version
nvidia-smi
echo $ROS_DISTRO

# انحصار انسٹال کریں
sudo apt update
sudo apt install build-essential cmake
sudo apt install ros-humble-cv-bridge
sudo apt install ros-humble-vision-msgs
```

<div className="border-line"></div>

<h4 className="fourth-heading">مسئلہ: GPU ایکسلریشن کام نہیں کر رہا</h4>

<div className="border-line"></div>

**علامات**:
- • نوڈس GPU استعمال کے بغیر چلتے ہیں
- • زیادہ CPU استعمال، بےکار GPU
- • کنسول میں CUDA کی خرابیاں

<div className="border-line"></div>

**حل**:
```bash
# CUDA چیک کریں
which nvcc
nvidia-smi
nvidia-smi -q -d COMPUTE

# GPU تشکیل دیں
ros2 param set /your_node gpu_index 0
ros2 param set /your_node enable_cuda true
```

<div className="border-line"></div>

<h4 className="fourth-heading">مسئلہ: زیادہ تاخیر</h4>

<div className="border-line"></div>

**علامات**:
- • ان پٹ/آؤٹ پٹ کے درمیان زیادہ تاخیر
- • فریم ڈراپس
- • قطار میں اوور فلو وارننگز

<div className="border-line"></div>

**حل**:
```bash
# پیرامیٹرز کو بہتر بنائیں
ros2 param set /your_node input_queue_size 1
ros2 param set /your_node enable_async_processing true

# کارکردگی مانیٹر کریں
ros2 topic hz /camera/image_raw
htop
watch -n 1 nvidia-smi
```

<div className="border-line"></div>

<h4 className="fourth-heading">مسئلہ: نوڈس کریش ہو جاتے ہیں یا سیگ فاؤلٹ</h4>

<div className="border-line"></div>

**علامات**:
- • غیر متوقع ختم ہونا
- • GPU میموری کی خرابیاں
- • CUDA رن ٹائم کی خرابیاں

<div className="border-line"></div>

**حل**:
```bash
# GPU میموری مانیٹر کریں
watch -n 1 'nvidia-smi --query-gpu=memory.used,memory.total --format=csv'

# حدود سیٹ کریں
export CUDA_VISIBLE_DEVICES=0

# ڈرائیورز اپ ڈیٹ کریں
sudo apt install nvidia-driver-535
sudo reboot
```

</details>

<div className="border-line"></div>

<h2 className="second-heading">Isaac ROS کا تعارف</h2>

<div className="border-line"></div>

<h3 className="third-heading">جائزہ</h3>

<div className="border-line"></div>

Isaac ROS NVIDIA کا ہارڈ ویئر-ایکسلریٹڈ ادراک پائپ لائن ہے روبوٹکس کے لیے۔ یہ GPU کمپیوٹنگ کو ROS 2 کے ساتھ جوڑتا ہے، فراہم کرتا ہے:
- • **ہارڈ ویئر ایکسلریشن**: GPU متوازی پروسیسنگ
- • **پلگ اینڈ پلے انضمام**: بے داغ ROS 2 انضمام
- • **بہتر الگورتھم**: GPU-بہتر ادراک کے کام
- • **ریل ٹائم کارکردگی**: کم-تاخیر پروسیسنگ
- • **توانائی کارآمد**: کنارے کے پلیٹ فارمز کے لیے بہتر

<div className="border-line"></div>

<h2 className="second-heading">انسٹالیشن اور سیٹ اپ</h2>

<div className="border-line"></div>

<h3 className="third-heading">سسٹم کی ضروریات</h3>

<div className="border-line"></div>

- • **GPU**: CUDA سپورٹ کے ساتھ NVIDIA GPU
- • **CUDA**: 11.8 یا بعد کا
- • **OS**: Ubuntu 20.04/22.04 LTS
- • **ROS 2**: Humble یا بعد کا
- • **TensorRT**: 8.5 یا بعد کا

<div className="border-line"></div>

<h3 className="third-heading">انسٹالیشن</h3>

<div className="border-line"></div>

```bash
# بائنری انسٹالیشن
sudo apt update
sudo apt install ros-humble-isaac-ros-common

# Docker انسٹالیشن
docker pull nvcr.io/nvidia/isaac-ros/isaac_ros_common:latest

# سورس تعمیر
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
colcon build --packages-select isaac_ros_common
```

<div className="border-line"></div>

<h2 className="second-heading">کوڈ کی مثالیں</h2>

<div className="border-line"></div>

<h3 className="third-heading">گہرائی کی پروسیسنگ</h3>

<div className="border-line"></div>

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from stereo_msgs.msg import DisparityImage

class DepthProcessor(Node):
    def __init__(self):
        super().__init__('depth_processor')
        self.left_sub = self.create_subscription(
            Image, '/stereo/left/image', self.callback, 10)
        self.disp_pub = self.create_publisher(
            DisparityImage, '/stereo/disparity', 10)

    def callback(self, msg):
        # GPU-ایکسلریٹڈ اسٹیریو میچنگ
        disparity = self.compute_disparity(msg)
        self.disp_pub.publish(disparity)
```

<div className="border-line"></div>

<h3 className="third-heading">آبجیکٹ ڈیٹیکشن</h3>

<div className="border-line"></div>

```python
from vision_msgs.msg import Detection2DArray

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        self.image_sub = self.create_subscription(
            Image, '/camera/image', self.detect, 10)
        self.det_pub = self.create_publisher(
            Detection2DArray, '/detections', 10)

    def detect(self, msg):
        # TensorRT-ایکسلریٹڈ ڈیٹیکشن
        detections = self.run_inference(msg)
        self.det_pub.publish(detections)
```

<div className="border-line"></div>

<h3 className="third-heading">پوائنٹ کلاؤڈ پروسیسنگ</h3>

<div className="border-line"></div>

```python
from sensor_msgs.msg import PointCloud2

class PointCloudProcessor(Node):
    def __init__(self):
        super().__init__('pc_processor')
        self.pc_sub = self.create_subscription(
            PointCloud2, '/depth/points', self.process, 10)
        self.filtered_pub = self.create_publisher(
            PointCloud2, '/filtered_points', 10)

    def process(self, msg):
        # GPU-ایکسلریٹڈ فلٹرنگ
        filtered = self.filter_pointcloud(msg)
        self.filtered_pub.publish(filtered)
```

<div className="border-line"></div>

<h2 className="second-heading">بہترین مشقیں</h2>

<div className="border-line"></div>

<h3 className="third-heading">پائپ لائن ڈیزائن</h3>

<div className="border-line"></div>

- • **ریسورس مینجمنٹ**: GPU میموری کارآمدی کے ساتھ مینج کریں
- • **پائپ لائن سنکرونائزیشن**: مناسب ٹائم کی تصدیق کریں
- • **غلطی کا انتظام**: مضبوط ہارڈ ویئر ناکامی کا انتظام
- • **ماڈیولر ڈیزائن**: دوبارہ استعمال کے قابل نوڈس بنائیں
- • **کارکردگی مانیٹرنگ**: مسلسل بہتری

<div className="border-line"></div>

<h3 className="third-heading">ہارڈ ویئر کی بہتری</h3>

<div className="border-line"></div>

- • **GPU میموری**: کارآمد بفر مینجمنٹ
- • **سٹریم پروسیسنگ**: اوور لیپ آپریشنز
- • **کرنل کی بہتری**: CUDA کرنلز کو بہتر بنائیں
- • **ڈیٹا ٹرانسفرز**: CPU-GPU ٹرانسفرز کو کم کریں
- • **بیچ پروسیسنگ**: بیچز میں پروسیس کریں

<div className="border-line"></div>

<h2 className="second-heading">عام مسائل</h2>

<div className="border-line"></div>

**انسٹالیشن ناکامیاں**
- • ہارڈ ویئر مطابقت کی تصدیق کریں
- • CUDA/TensorRT ورژن چیک کریں
- • ROS 2 مناسب انسٹالیشن یقینی بنائیں

**GPU کا پتہ نہیں چلتا**
- • GPU ڈرائیور انسٹالیشن چیک کریں
- • CUDA رن ٹائم ورژن کی تصدیق کریں
- • صارف GPU اجازت چیک کریں

**زیادہ تاخیر**
- • پائپ لائن بٹلنیکس کو پروفائل کریں
- • بفر سائز کو بہتر بنائیں
- • CPU-GPU سنکرونائزیشن چیک کریں

**زیادہ GPU میموری**
- • میموری پولنگ نافذ کریں
- • ڈیٹا ریزولوشن کم کریں
- • بیچ سائز کو بہتر بنائیں

<div className="border-line"></div>

<h2 className="second-heading">خلاصہ</h2>

<div className="border-line"></div>

Isaac ROS NVIDIA GPU ٹیکنالوجی کو استعمال کرتے ہوئے روبوٹکس کے لیے ہارڈ ویئر-ایکسلریٹڈ ادراک فراہم کرتا ہے۔ یہ GPU کمپیوٹنگ کو ROS 2 کے ساتھ جوڑتا ہے، زیادہ کارکردگی والی ادراک پائپ لائنز کو فعال کرتا ہے۔ کامیابی کے لیے آرکیٹیکچر کو سمجھنا، مناسب انسٹالیشن، اور مخصوص ہارڈ ویئر پلیٹ فارمز کے لیے بہتری کی ضرورت ہوتی ہے۔