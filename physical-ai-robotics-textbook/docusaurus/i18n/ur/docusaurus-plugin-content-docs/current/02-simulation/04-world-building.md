---
sidebar_position: 5
title: 'ورلڈ بلڈنگ: پیچیدہ سیمولیشن ماحول تخلیق کرنا'
description: 'ہیومنوائڈ روبوٹکس کے لیے پیچیدہ سیمولیشن ماحول کو ڈیزائن اور نافذ کرنا'
---



<h1 className="main-heading">ورلڈ بلڈنگ: پیچیدہ سیمولیشن ماحول تخلیق کرنا</h1>
<div className="underline-class"></div>

ہیومنوائڈ روبوٹ ٹیسٹنگ کے لیے حقیقی طرز کے سیمولیشن ماحول بنائیں - انڈور سپیسز سے لے کر آؤٹ ڈور ٹیرین تک۔

<div className="border-line"></div>

<h2 className="second-heading">سیکھنے کے اہداف</h2>
<div className="underline-class"></div>

- • فزکس خصوصیات کے ساتھ سیمولیشن ماحول ڈیزائن کرنا
- • انڈور/آؤٹ ڈور ماحول بنانا
- • ڈائنامک اور انٹرایکٹو اشیاء نافذ کرنا
- • لائٹنگ اور ویژوئل ایفیکٹس تشکیل دینا
- • سیمولیشن کارکردگی کو بہتر بنانا

<div className="border-line"></div>

<h2 className="second-heading">مشقیں</h2>
<div className="underline-class"></div>

<details>
<summary>مشق 2.4.1: بنیادی انڈور ماحول (⭐, ~30 منٹ)</summary>

<h3 className="third-heading">مشق 2.4.1: بنیادی انڈور ماحول</h3>
<div className="underline-class"></div>

**پیچیدگی**: ⭐ | **وقت**: 30 منٹ | **ضروریات**: Gazebo، XML علم

<h4 className="fourth-heading">شروع کرنے والا کوڈ</h4>
<div className="underline-class"></div>

- • کمرہ کی سٹرکچر (دیواریں، فرش، چھت)
- • فرنیچر (میز، کرسی)
- • لائٹنگ اور فزکس

<h4 className="fourth-heading">کامیابی کے معیار</h4>
<div className="underline-class"></div>

- [ ] Gazebo میں کمرہ لوڈ ہوتا ہے
- [ ] کولیژن/ویژوئل عناصر کام کرتے ہیں
- [ ] لائٹنگ کام کرتی ہے
- [ ] فزکس مستحکم ہے

<h4 className="fourth-heading">ٹیسٹ کمانڈز</h4>
<div className="underline-class"></div>
```bash
gz sdf -k your_indoor.world
gazebo your_indoor.world
```

<h4 className="fourth-heading">اشارے</h4>
<div className="underline-class"></div>

- • غیر قابل حرکت اشیاء کے لیے اسٹیٹک فلیگ استعمال کریں
- • ٹیسٹ سے پہلے SDF کی توثیق کریں

</details>

<details>
<summary>مشق 2.4.2: اعلیٰ ماحول (⭐⭐, ~45 منٹ)</summary>

<h3 className="third-heading">مشق 2.4.2: اعلیٰ ماحول</h3>
<div className="underline-class"></div>

**پیچیدگی**: ⭐⭐ | **وقت**: 45 منٹ

<h4 className="fourth-heading">شروع کرنے والا کوڈ</h4>
<div className="underline-class"></div>

- • انڈور/آؤٹ ڈور ٹرانزیشنز
- • ڈائنامک قابل حرکت اشیاء
- • متعدد لائٹنگ منظرنامے
- • مختلف سطح کے مواد

<h4 className="fourth-heading">کامیابی کے معیار</h4>
<div className="underline-class"></div>

- [ ] اسٹیٹک اور ڈائنامک عناصر کام کرتے ہیں
- [ ] سطح کے مواد تشکیل دیے گئے ہیں
- [ ] اچھی کارکردگی

<h4 className="fourth-heading">ٹیسٹ کمانڈز</h4>
<div className="underline-class"></div>
```bash
gazebo your_advanced.world
gz model -m moving_platform -x 1 -y 0 -z 1
```

</details>

<details>
<summary>مشق 2.4.3: ماڈیولر ورلڈ سسٹم (⭐⭐⭐, ~60 منٹ)</summary>

<h3 className="third-heading">مشق 2.4.3: ماڈیولر ورلڈ سسٹم</h3>
<div className="underline-class"></div>

**پیچیدگی**: ⭐⭐⭐ | **وقت**: 60 منٹ

<h4 className="fourth-heading">شروع کرنے والا کوڈ</h4>
<div className="underline-class"></div>

- • دوبارہ استعمال کے قابل ماحول کے اجزاء
- • کارکردگی کی بہتری
- • متعدد ٹیسٹنگ منظرنامے

<h4 className="fourth-heading">ٹیسٹ کمانڈز</h4>
<div className="underline-class"></div>
```bash
gz sdf -k model://room_module/model.sdf
gazebo modular_environment.world
```

<h4 className="fourth-heading">اشارے</h4>
<div className="underline-class"></div>

- • معیاری کنکشن پوائنٹس
- • مسلسل کوآرڈینیٹ سسٹم

</details>

<div className="border-line"></div>

<h2 className="second-heading">ٹربل شوٹنگ</h2>
<div className="underline-class"></div>

<details>
<summary>عام مسائل</summary>

<h3 className="third-heading">ٹربل شوٹنگ</h3>
<div className="underline-class"></div>

<h4 className="fourth-heading">سست سیمولیشن</h4>
<div className="underline-class"></div>

**علامات**: کم ریل ٹائم فیکٹر، زیادہ CPU/GPU استعمال

**حل**:
```xml
<visual><geometry><box><size>2 2 0.1</size></box></geometry></visual>
<model static="true"><!-- content --></model>
<physics><max_step_size>0.01</max_step_size></physics>
```

<h4 className="fourth-heading">اشیاء سطحوں کے ذریعے گرتی ہیں</h4>
<div className="underline-class"></div>

**حل**:
```xml
<collision><surface><contact><ode>
  <kp>1e+13</kp><kd>1</kd>
</ode></contact></surface></collision>
```

<h4 className="fourth-heading">غلط لائٹنگ</h4>
<div className="underline-class"></div>

**حل**:
```xml
<light type="point">
  <pose>0 0 3 0 0 0</pose>
  <diffuse>0.9 0.9 0.9 1</diffuse>
</light>
```

<h4 className="fourth-heading">تیرتی/ڈوبتی ہوئی اشیاء</h4>
<div className="underline-class"></div>

**حل**:
```xml
<model><pose>2.0 1.0 0.5 0 0 0</pose></model>
<collision><pose>0 0 0 0 0 0</pose></collision>
```

</details>

<div className="border-line"></div>

<h2 className="second-heading">ماحول کی ڈیزائن</h2>
<div className="underline-class"></div>

<h3 className="third-heading">ریلزم بمقابلہ کارکردگی</h3>
<div className="underline-class"></div>

- • **پولی گان کاؤنٹ**: میشز کو سادہ رکھیں
- • **کولیژن جیومیٹری**: سادہ شکلیں استعمال کریں
- • **فزکس**: ریلزم کو کارکردگی کے ساتھ توازن دیں
- • **ٹیکسچرز**: مناسب ریزولوشن

<h3 className="third-heading">ماحول کی اقسام</h3>
<div className="underline-class"></div>

<h4 className="fourth-heading">انڈور</h4>
<div className="underline-class"></div>
```xml
<world name="indoor_lab">
  <include><uri>model://ground_plane</uri></include>
  <light type="point"><pose>0 0 3 0 0 0</pose></light>
  <model name="wall"><pose>-5 0 1.5 0 0 0</pose></model>
</world>
```

<h4 className="fourth-heading">آؤٹ ڈور</h4>
<div className="underline-class"></div>
```xml
<world name="outdoor_park">
  <model name="terrain">
    <link><collision><geometry>
      <heightmap><uri>heightmap.png</uri></heightmap>
    </geometry></collision></link>
  </model>
</world>
```

<div className="border-line"></div>

<h2 className="second-heading">فزکس کنفیگریشن</h2>
<div className="underline-class"></div>

<h3 className="third-heading">مواد کی خصوصیات</h3>
<div className="underline-class"></div>
```xml
<physics type="ode">
  <max_step_size>0.001</max_step_size>
  <gravity>0 0 -9.8</gravity>
</physics>
<surface><friction><ode><mu>0.8</mu></ode></friction></surface>
```

<h3 className="third-heading">فرکشن ماڈلز</h3>
<div className="underline-class"></div>
```xml
<!-- High friction (rubber) -->
<surface><friction><ode><mu>1.0</mu></ode></friction></surface>
<!-- Low friction (ice) -->
<surface><friction><ode><mu>0.1</mu></ode></friction></surface>
```

<h2 className="second-heading">ڈائنامک عناصر</h2>
<div className="underline-class"></div>

<h3 className="third-heading">قابل حرکت اشیاء</h3>
<div className="underline-class"></div>
```xml
<model name="moving_platform">
  <joint type="prismatic">
    <axis><xyz>0 1 0</xyz></axis>
  </joint>
</model>
```

<h3 className="third-heading">انٹرایکٹو اشیاء</h3>
<div className="underline-class"></div>
```xml
<model name="cup">
  <link><collision><geometry>
    <cylinder><radius>0.05</radius></cylinder>
  </geometry></collision></link>
</model>
```

<h2 className="second-heading">لائٹنگ</h2>
<div className="underline-class"></div>
```xml
<!-- Directional (sun) -->
<light type="directional"><direction>-0.5 0.1 -0.9</direction></light>
<!-- Point light -->
<light type="point"><pose>0 0 3 0 0 0</pose></light>
```

<h2 className="second-heading">بہترین مشقیں</h2>
<div className="underline-class"></div>

- • اسٹیٹک اشیاء کو اسٹیٹک کے طور پر نشان زد کریں
- • کولیژن جیومیٹری کو سادہ بنائیں
- • ڈائنامک عناصر کو محدود کریں
- • میش کی پیچیدگی کو بہتر بنائیں
- • ماڈیولر ڈیزائن استعمال کریں

<h3 className="third-heading">ماڈیولر ڈیزائن</h3>
<div className="underline-class"></div>
```xml
<world name="modular_environment">
  <include><uri>model://room_module_1</uri></include>
  <include><uri>model://room_module_2</uri></include>
</world>
```

<h3 className="third-heading">ورلڈ جنریشن</h3>
<div className="underline-class"></div>
```python
import xml.etree.ElementTree as ET

def create_world_with_obstacles(world_name, obstacles):
    sdf = ET.Element('sdf', version='1.7')
    world = ET.SubElement(sdf, 'world', name=world_name)
    # Add obstacles
    tree = ET.ElementTree(sdf)
    tree.write(f'{world_name}.world')
```

<h2 className="second-heading">خلاصہ</h2>
<div className="underline-class"></div>

سیمولیشن ماحول بناتے وقت ریلزم کو کارکردگی کے ساتھ توازن دیں۔ دوبارہ استعمال کے لیے ماڈیولر ڈیزائن استعمال کریں اور مؤثر ہیومنوائڈ روبوٹ ٹیسٹنگ کے لیے مناسب فزکس کنفیگریشن برقرار رکھیں۔

import ReadingTime from '@site/src/components/ReadingTime';

<ReadingTime minutes={6} />