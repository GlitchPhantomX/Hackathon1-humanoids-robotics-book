---
sidebar_position: 2
title: "ضروریات اور سیٹ اپ کی ضروریات"
---

import ReadingTime from '@site/src/components/ReadingTime';


<ReadingTime minutes={3} />

<h1 className="main-heading">ضروریات اور سیٹ اپ کی ضروریات</h1>
<div className="underline-class"></div>

**سیکھنے کے اہداف**:
- • ضروریات کے خلاف مہارت کی سطح کا جائزہ لیں
- • درکار سافٹ ویئر اسٹیک انسٹال کریں
- • ترقی کا ماحول قائم کریں
- • نظام کی ضروریات کی تصدیق کریں

**ضروریات**: بنیادی پروگرامنگ، لینکس CLI | **وقت**: 2-4 گھنٹے

<div className="border-line"></div>

<h2 className="second-heading">ضروریات کا جائزہ</h2>
<div className="underline-class"></div>

فزیکل ای آئی اور ہیومنوائڈ روبوٹکس کی ترقی کے لیے ضروریات.

<div className="border-line"></div>

<h2 className="second-heading">نالج کی ضروریات</h2>
<div className="underline-class"></div>

<h3 className="third-heading">پروگرامنگ کی مہارتیں</h3>
<div className="underline-class"></div>

- **پائی تھون**: متوسط (funcs, کلاسز, modules)
- **C++**: بنیادی OOP
- **لینکس CLI**: ٹرمنل، فائل سسٹم
- **گٹ**: کلون، کمٹ، پل

<h3 className="third-heading">ریاضی کی بنیاد</h3>
<div className="underline-class"></div>

- **لکیری الجبرا**: ویکٹرز، میٹرکس، تبدیلیاں
- **کیلکولس**: مشتق، انتگرل
- **احتمال**: سینسر فیوژن
- **طبیعیات**: قوتیں، حرکت، میکانکس

<div className="border-line"></div>

<h2 className="second-heading">سافٹ ویئر کی ضروریات</h2>
<div className="underline-class"></div>

<h3 className="third-heading">آپریٹنگ سسٹم</h3>
<div className="underline-class"></div>

- **اوبنٹو 20.04/22.04 LTS** (تجویز کردہ)
- **متبادل**: ونڈوز پر WSL2، میک او ایس

<h3 className="third-heading">کور سافٹ ویئر اسٹیک</h3>
<div className="underline-class"></div>

- **ROS 2**: ہمبل ہاکسبل یا رولنگ
- **پائی تھون**: 3.8+
- **C++ کمپائلر**: GCC 9+
- **گٹ**: 2.25+
- **ڈاکر**: کنٹینرز کے لیے

<h3 className="third-heading">ترقی کے اوزار</h3>
<div className="underline-class"></div>

- **IDE**: ROS ایکسٹینشن کے ساتھ VS کوڈ یا CLion
- **بلڈ ٹولز**: CMake، میک، کولکون
- **پیکیج مینیجرز**: APT، pip، conda

<div className="border-line"></div>

<h2 className="second-heading">ہارڈ ویئر کی تجاویز</h2>
<div className="underline-class"></div>

<h3 className="third-heading">کم از کم ضروریات</h3>
<div className="underline-class"></div>

- **CPU**: 4+ کور، 2.5GHz+
- **RAM**: 8GB (16GB تجویز کردہ)
- **اسٹوریج**: 50GB+ خالی
- **گرافکس**: ان ٹیگریٹڈ یا NVIDIA GPU

<h3 className="third-heading">اعلی درجے کا کام</h3>
<div className="underline-class"></div>

- **NVIDIA GPU**: CUDA کے ساتھ RTX سیریز
- **روبوٹ پلیٹ فارم**: ROS-مطابق (ٹرٹل بٹ3، UR5)
- **سینسرز**: RGB-D کیمرہ، IMU، LIDAR

<div className="border-line"></div>

<h2 className="second-heading">مشقیں</h2>
<div className="underline-class"></div>

**مشق 0.2.1**: ماحول کی تنصیب کی تصدیق
- اوبنٹو 20.04/22.04 یا WSL2 انسٹال کریں
- ROS 2 ہمبل انسٹال کریں
- پائی تھون اور C++ ٹولز سیٹ اپ کریں
- گٹ اور ڈاکر کی تصدیق کریں

**مشق 0.2.2**: ROS 2 ورک سپیس کا بنیادی سیٹ اپ
- ورک سپیس ڈائرکٹری بنائیں
- کولکون کے ساتھ شروع کریں
- ورک سپیس بنائیں
- ماحول کو ماخذ بنائیں

**مشق 0.2.3**: پائی تھون مہارت کا جائزہ (⭐⭐، 30-40 منٹ)
- روبوٹ کمانڈ پارسر بنائیں
- کلاسز اور فنکشنز کا استعمال کریں
- خامی کے انتظام کو نافذ کریں
- دستاویزات شامل کریں

<div className="border-line"></div>

<h2 className="second-heading">عام مسائل</h2>
<div className="underline-class"></div>

**ROS 2 انسٹالیشن مسائل**:
```bash
lsb_release -sc  # اوبنٹو ورژن کی تصدیق کریں
sudo apt update
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
ros2 --version  # انسٹالیشن کی تصدیق کریں
```

**اجازت کے مسائل**:
```bash
sudo usermod -a -G dialout $USER
sudo usermod -a -G plugdev $USER
newgrp dialout
groups $USER  # گروپس کی تصدیق کریں
```

**پائی تھون پیکیج کے مسائل**:
```bash
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
python3 -c "import rclpy; print('ROS Python libraries available')"
```

**کولکون بلڈ انحصار**:
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

**نیٹ ورک رابطہ کے مسائل**:
```bash
echo $ROS_DOMAIN_ID
export ROS_DOMAIN_ID=0
sudo ufw allow from [OTHER_IP] to any port 7400:7500 proto udp
ros2 topic list
```

<div className="border-line"></div>

<h2 className="second-heading">خلاصہ</h2>
<div className="underline-class"></div>

مناسب ماحول کا سیٹ اپ انتہائی اہم ہے. ضروریات ہموار سیکھنے کو یقینی بناتی ہیں. روبوٹکس کی ترقی کے لیے مسئلہ حل کرنے کی مہارتیں ضروری ہیں.

**اہم نکات**:
- • ماحول کا سیٹ اپ انتہائی اہم ہے
- • ضروریات ہموار ترقی کو یقینی بناتی ہیں
- • مسئلہ حل کرنے کی مہارتیں ضروری ہیں
- • تصدیق بعد میں مسائل کو روکتی ہے

<h2 className="second-heading">وسائل</h2>
<div className="underline-class"></div>

- • [ROS 2 انسٹالیشن گائیڈ](https://docs.ros.org/en/humble/Installation.html)
- • [روبوٹکس کے لیے پائی تھون](https://roboticsbackend.com/python-for-robotics/)
- • [ROS 2 ٹیوٹوریلز](https://docs.ros.org/en/humble/Tutorials.html)

**نیویگیشن**: [← پچھلا](./01-welcome.md) | [اگلا →](./03-hardware-requirements.md)

<h2 className="second-heading">فوری حوالہ</h2>
<div className="underline-class"></div>
```bash
# ROS 2 ماحول کا سیٹ اپ
sudo apt install ros-humble-desktop-full
source /opt/ros/humble/setup.bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
source install/setup.bash
```

| جزو | ضرورت | تصدیق |
|-----------|-------------|--------------|
| پائی تھون | 3.8+ | `python3 --version` |
| ROS 2 | ہمبل | `ros2 --version` |
| GCC | 9+ | `gcc --version` |
| گٹ | 2.25+ | `git --version` |
| ڈاکر | تازہ ترین | `docker --version` |