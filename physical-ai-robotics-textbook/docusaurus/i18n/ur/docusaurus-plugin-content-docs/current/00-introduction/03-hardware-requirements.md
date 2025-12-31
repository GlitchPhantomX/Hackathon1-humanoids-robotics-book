---
sidebar_position: 3
title: "فزیکل ای آئی کے لیے ہارڈ ویئر کی ضروریات"
---

import ReadingTime from '@site/src/components/ReadingTime';


<ReadingTime minutes={4} />

<h1 className="main-heading">فزیکل ای آئی کے لیے ہارڈ ویئر کی ضروریات</h1>
<div className="underline-class"></div>

**سیکھنے کے اہداف**:
- • فزیکل ای آئی کے لیے ہارڈ ویئر اجزاء کو سمجھنا
- • کمپیوٹنگ کی ضروریات کا جائزہ لینا
- • روبوٹ پلیٹ فارم کا جائزہ لینا
- • سینسر انضمام کی منصوبہ بندی کرنا
- • سیفٹی اور بجٹ کو مدنظر رکھنا

**ضروریات**: بنیادی ہارڈ ویئر کا علم | **وقت**: 2-3 گھنٹے

<div className="border-line"></div>

<h2 className="second-heading">تعارف</h2>
<div className="underline-class"></div>

فزیکل ای آئی سسٹم کو حقیقی دنیا میں تعامل کے لیے مخصوص ہارڈ ویئر کی ضرورت ہوتی ہے. یہ باب سیمولیشن سے لے کر مکمل جسمانی سسٹم تک کی ضروریات کو مکمل کرتا ہے.

<div className="border-line"></div>

<h2 className="second-heading">صرف سیمولیشن ڈویلپمنٹ</h2>
<div className="underline-class"></div>

<h3 className="third-heading">کمپیوٹنگ کی ضروریات</h3>
<div className="underline-class"></div>

- • **CPU**: ملٹی-کور (i5/Ryzen 5+)
- • **GPU**: NVIDIA GTX 1060+
- • **RAM**: 16GB کم از کم، 32GB تجویز کردہ
- • **اسٹوریج**: 100GB SSD
- • **OS**: Ubuntu 20.04/22.04 یا Windows+WSL2

<h3 className="third-heading">اعلی درجے کی سیمولیشن</h3>
<div className="underline-class"></div>

- • Isaac Sim کے لیے NVIDIA RTX کے ساتھ CUDA
- • VR ہیڈ سیٹ (اختیاری)
- • موشن کیپچر (اختیاری)

<div className="border-line"></div>

<h2 className="second-heading">جسمانی روبوٹ پلیٹ فارمز</h2>
<div className="underline-class"></div>

<h3 className="third-heading">داخلہ کی سطح</h3>
<div className="underline-class"></div>

**TurtleBot3** ($1,000-$1,500): 2D LIDAR، RGB-D کیمرہ، ڈیفرنشل ڈرائیو
**JetBot** ($400-$600): Jetson Nano، کیمرے، ایج ای آئی

<h3 className="third-heading">متوسط</h3>
<div className="underline-class"></div>

**Unitree Go1** ($20k-$30k): کواڈرپیڈ، متحرک لوکوموشن
**Stretch RE1** ($15k-$20k): موبل مینوپولیٹر، 7-DOF آرم

<h3 className="third-heading">اعلی درجے کا ہیومنوائڈ</h3>
<div className="underline-class"></div>

**NAO** ($8k-$15k): 25+ DOF، کیمرے، مائیکروفونز
**Pepper** ($20k-$30k): انسانی تعامل، جذبات کی شناخت

<div className="border-line"></div>

<h2 className="second-heading">اہم اجزاء</h2>
<div className="underline-class"></div>

<h3 className="third-heading">ایکچوایٹرز</h3>
<div className="underline-class"></div>

- • **سروو موتورز**: درست پوزیشن کنٹرول (DYNAMIXEL، Herkulex)
- • **سٹیپر موتورز**: زاویہ کی پوزیشننگ، ڈرائیورز کی ضرورت ہوتی ہے
- • **برش لیس DC**: ہائی-پرفارمنس، ESC کی ضرورت ہوتی ہے

<h3 className="third-heading">سینسرز</h3>
<div className="underline-class"></div>

- • **کیمرے**: RGB، اسٹیریو، RGB-D (RealSense، ZED)
- • **LIDAR**: 2D (RPLIDAR)، 3D (Ouster، Velodyne)
- • **IMU**: سمت، تیزی
- • **فورس/ٹورک**: مینوپولیشن فیڈ بیک

<div className="border-line"></div>

<h2 className="second-heading">مشقیں</h2>
<div className="underline-class"></div>

**مشق 0.3.1**: ہارڈ ویئر کا جائزہ (⭐⭐، 25-35 منٹ)
- موجودہ ہارڈ ویئر اسپیکس کا جائزہ لیں
- ضروریات کے خلاف موازنہ کریں
- اپ گریڈ کی ضروریات کی شناخت کریں
- بجٹ کا منصوبہ بنائیں

**مشق 0.3.2**: روبوٹ پلیٹ فارم کا جائزہ (⭐⭐⭐، 45-60 منٹ)
- 3 پلیٹ فارمز کی تحقیق کریں (داخلہ/متوسط/اعلی درجہ)
- صلاحیتوں، قیمت، ROS مطابقت کا موازنہ کریں
- اہداف کے لیے بہترین فٹ کی شناخت کریں
- فوائد/ناقصانات کو قیمت-فائدہ کے ساتھ دستاویز کریں

**مشق 0.3.3**: سینسر انضمام کی منصوبہ بندی (⭐⭐، 30-40 منٹ)
- ضروری سینسرز کی شناخت کریں
- اسپیکس اور مطابقت کی تحقیق کریں
- ماؤنٹنگ اور کنکشنز کی منصوبہ بندی کریں
- وائرنگ ڈائرام بنائیں

<div className="border-line"></div>

<h2 className="second-heading">عام مسائل</h2>
<div className="underline-class"></div>

**ناکافی کمپیوٹنگ طاقت**:
```bash
nvidia-smi  # GPU چیک کریں
free -h     # میموری چیک کریں
# RTX 20xx+ Isaac Sim کے لیے تجویز کردہ
```

**روبوٹ پلیٹ فارم مطابقت**:
```bash
apt search ros-humble-  # دستیاب پیکیجز چیک کریں
ros2 topic list  # روبوٹ ٹاپکس کی تصدیق کریں
```

**سینسر ڈیٹا کی شرح کے مسائل**:
- سینسر اپ ڈیٹ کی شرح کم کریں
- کمپریشن استعمال کریں
- ڈیٹا فلٹرنگ نافذ کریں
- `ros2 topic hz` کے ساتھ مانیٹر کریں

**پاور ڈیلیوری کے مسائل**:
- کل کرنٹ ڈرا کا حساب لگائیں
- اسٹارٹ اپ سرگی (2-3x) کا احتساب کریں
- ایکچوایٹرز کے لیے الگ سپلائیز استعمال کریں
- 20-30% سیفٹی مارجن شامل کریں

**الیکٹرومیگنیٹک تداخل**:
- شیلڈڈ کیبلز استعمال کریں
- پاور کو سگنل کیبلز سے الگ کریں
- فیریٹ کورز شامل کریں
- مناسب گراؤنڈنگ نافذ کریں

<div className="border-line"></div>

<h2 className="second-heading">خلاصہ</h2>
<div className="underline-class"></div>

ہارڈ ویئر کا انتخاب اہداف اور بجٹ پر منحصر ہے. سیکھنے کے لیے فقط سیمولیشن مؤثر ہے. سینسر انضمام کو ماؤنٹنگ، پاور، اور رابطے کی احتیاط سے منصوبہ بندی کی ضرورت ہوتی ہے.

**اہم نکات**:
- • ہارڈ ویئر کو سیکھنے کے اہداف کے ساتھ مربوط کریں
- • ابتدائی سیکھنے کے لیے سیمولیشن مؤثر ہے
- • سینسر انضمام کی احتیاط سے منصوبہ بندی کریں
- • سیفٹی کے خیالات انتہائی اہم ہیں
- • مناسب پاور مینجمنٹ مسائل کو روکتی ہے

<h2 className="second-heading">وسائل</h2>
<div className="underline-class"></div>

- • [ROS 2 ہارڈ ویئر ضروریات](https://docs.ros.org/en/humble/Installation/Requirements.html)
- • [NVIDIA Isaac Sim گائیڈ](https://docs.nvidia.com/isaac/)
- • [ہارڈ ویئر انٹرفیس کی مثالیں](https://github.com/ros-controls/hardware_interface)

**نیویگیشن**: [← پچھلا](./02-prerequisites.md) | [اگلا →](./04-how-to-use.md)

<h2 className="second-heading">فوری حوالہ</h2>
<div className="underline-class"></div>

| جزو | کم از کم | تجویز کردہ | نوٹس |
|-----------|---------|-------------|-------|
| CPU | کواڈ-کور | ملٹی-کور | سیمولیشن/کنٹرول |
| GPU | ان ٹیگریٹڈ | GTX 1060+ | رینڈرنگ/ای آئی |
| RAM | 16GB | 32GB+ | پیچیدہ سیمولیشنز |
| اسٹوریج | 100GB SSD | 500GB+ NVMe | ماڈل لوڈنگ |
| نیٹ ورک | ایتھرنیٹ | گیگابٹ | سینسر ڈیٹا |