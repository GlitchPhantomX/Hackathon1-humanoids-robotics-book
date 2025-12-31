---
sidebar_position: 5
title: 'वर्ल्ड बिल्डिंग: जटिल सिमुलेशन वातावरण बनाना'
description: 'मानवरूपी रोबोटिक्स के लिए जटिल सिमुलेशन वातावरण को डिज़ाइन और कार्यान्वित करना'
---

import ReadingTime from '@site/src/components/ReadingTime';

<ReadingTime minutes={6} />

<h1 className="main-heading">वर्ल्ड बिल्डिंग: जटिल सिमुलेशन वातावरण बनाना</h1>
<div className="underline-class"></div>

मानवरूपी रोबोट परीक्षण के लिए वास्तविक सिमुलेशन वातावरण बनाएं - आंतरिक स्थानों से लेकर बाहरी भूभाग तक।

<div className="border-line"></div>

<h2 className="second-heading">सीखने के उद्देश्य</h2>
<div className="underline-class"></div>

- • भौतिकी गुणों के साथ सिमुलेशन वातावरण डिज़ाइन करें
- • आंतरिक/बाहरी वातावरण बनाएं
- • गतिशील और इंटरैक्टिव ऑब्जेक्ट कार्यान्वित करें
- • प्रकाश और दृश्य प्रभाव कॉन्फ़िगर करें
- • सिमुलेशन प्रदर्शन अनुकूलित करें

<div className="border-line"></div>

<h2 className="second-heading">अभ्यास</h2>
<div className="underline-class"></div>

<details>
<summary>अभ्यास 2.4.1: मूल आंतरिक वातावरण (⭐, ~30 मिनट)</summary>

<h3 className="third-heading">अभ्यास 2.4.1: मूल आंतरिक वातावरण</h3>
<div className="underline-class"></div>

**कठिनाई**: ⭐ | **समय**: 30 मिनट | **आवश्यकताएँ**: गेज़बो, एक्सएमएल ज्ञान

<h4 className="fourth-heading">स्टार्टर कोड</h4>
<div className="underline-class"></div>

- • कमरा संरचना (दीवारें, फर्श, छत)
- • फर्नीचर (मेज, कुर्सी)
- • प्रकाश और भौतिकी

<h4 className="fourth-heading">सफलता मानदंड</h4>
<div className="underline-class"></div>

- [ ] कमरा गेज़बो में लोड होता है
- [ ] संघर्ष/दृश्य तत्व काम करते हैं
- [ ] प्रकाश कार्यात्मक है
- [ ] भौतिकी स्थिर है

<h4 className="fourth-heading">परीक्षण कमांड</h4>
<div className="underline-class"></div>
```bash
gz sdf -k your_indoor.world
gazebo your_indoor.world
```

<h4 className="fourth-heading">संकेत</h4>
<div className="underline-class"></div>

- • अचल वस्तुओं के लिए स्थैतिक फ्लैग का उपयोग करें
- • परीक्षण से पहले एसडीएफ की पुष्टि करें

</details>

<details>
<summary>अभ्यास 2.4.2: उन्नत वातावरण (⭐⭐, ~45 मिनट)</summary>

<h3 className="third-heading">अभ्यास 2.4.2: उन्नत वातावरण</h3>
<div className="underline-class"></div>

**कठिनाई**: ⭐⭐ | **समय**: 45 मिनट

<h4 className="fourth-heading">स्टार्टर कोड</h4>
<div className="underline-class"></div>

- • आंतरिक/बाहरी संक्रमण
- • गतिशील गतिशील वस्तुएँ
- • कई प्रकाश परिदृश्य
- • अलग-अलग सतह सामग्री

<h4 className="fourth-heading">सफलता मानदंड</h4>
<div className="underline-class"></div>

- [ ] स्थैतिक और गतिशील तत्व काम करते हैं
- [ ] सतह सामग्री कॉन्फ़िगर की गई है
- [ ] अच्छा प्रदर्शन

<h4 className="fourth-heading">परीक्षण कमांड</h4>
<div className="underline-class"></div>
```bash
gazebo your_advanced.world
gz model -m moving_platform -x 1 -y 0 -z 1
```

</details>

<details>
<summary>अभ्यास 2.4.3: मॉड्यूलर वर्ल्ड सिस्टम (⭐⭐⭐, ~60 मिनट)</summary>

<h3 className="third-heading">अभ्यास 2.4.3: मॉड्यूलर वर्ल्ड सिस्टम</h3>
<div className="underline-class"></div>

**कठिनाई**: ⭐⭐⭐ | **समय**: 60 मिनट

<h4 className="fourth-heading">स्टार्टर कोड</h4>
<div className="underline-class"></div>

- • पुन: उपयोग योग्य वातावरण घटक
- • प्रदर्शन अनुकूलन
- • कई परीक्षण परिदृश्य

<h4 className="fourth-heading">परीक्षण कमांड</h4>
<div className="underline-class"></div>
```bash
gz sdf -k model://room_module/model.sdf
gazebo modular_environment.world
```

<h4 className="fourth-heading">संकेत</h4>
<div className="underline-class"></div>

- • मानक कनेक्शन बिंदु
- • सुसंगत निर्देशांक प्रणालियाँ

</details>

<div className="border-line"></div>

<h2 className="second-heading">समस्या निवारण</h2>
<div className="underline-class"></div>

<details>
<summary>सामान्य समस्याएँ</summary>

<h3 className="third-heading">समस्या निवारण</h3>
<div className="underline-class"></div>

<h4 className="fourth-heading">धीमा सिमुलेशन</h4>
<div className="underline-class"></div>

**लक्षण**: कम रीयल-टाइम फैक्टर, उच्च सीपीयू/जीपीयू उपयोग

**समाधान**:
```xml
<visual><geometry><box><size>2 2 0.1</size></box></geometry></visual>
<model static="true"><!-- content --></model>
<physics><max_step_size>0.01</max_step_size></physics>
```

<h4 className="fourth-heading">ऑब्जेक्ट सतहों के माध्यम से गिर जाते हैं</h4>
<div className="underline-class"></div>

**समाधान**:
```xml
<collision><surface><contact><ode>
  <kp>1e+13</kp><kd>1</kd>
</ode></contact></surface></collision>
```

<h4 className="fourth-heading">गलत प्रकाश</h4>
<div className="underline-class"></div>

**समाधान**:
```xml
<light type="point">
  <pose>0 0 3 0 0 0</pose>
  <diffuse>0.9 0.9 0.9 1</diffuse>
</light>
```

<h4 className="fourth-heading">तैरती/डूबी हुई वस्तुएँ</h4>
<div className="underline-class"></div>

**समाधान**:
```xml
<model><pose>2.0 1.0 0.5 0 0 0</pose></model>
<collision><pose>0 0 0 0 0 0</pose></collision>
```

</details>

<div className="border-line"></div>

<h2 className="second-heading">वातावरण डिज़ाइन</h2>
<div className="underline-class"></div>

<h3 className="third-heading">वास्तविकता बनाम प्रदर्शन</h3>
<div className="underline-class"></div>

- • **बहुभुज गिनती**: मेश को सरल रखें
- • **संघर्ष ज्यामिति**: सरल आकृतियों का उपयोग करें
- • **भौतिकी**: दक्षता के साथ वास्तविकता को संतुलित करें
- • **टेक्सचर**: उचित रिज़ॉल्यूशन

<h3 className="third-heading">वातावरण प्रकार</h3>
<div className="underline-class"></div>

<h4 className="fourth-heading">आंतरिक</h4>
<div className="underline-class"></div>
```xml
<world name="indoor_lab">
  <include><uri>model://ground_plane</uri></include>
  <light type="point"><pose>0 0 3 0 0 0</pose></light>
  <model name="wall"><pose>-5 0 1.5 0 0 0</pose></model>
</world>
```

<h4 className="fourth-heading">बाहरी</h4>
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

<h2 className="second-heading">भौतिकी कॉन्फ़िगरेशन</h2>
<div className="underline-class"></div>

<h3 className="third-heading">सामग्री गुण</h3>
<div className="underline-class"></div>
```xml
<physics type="ode">
  <max_step_size>0.001</max_step_size>
  <gravity>0 0 -9.8</gravity>
</physics>
<surface><friction><ode><mu>0.8</mu></ode></friction></surface>
```

<h3 className="third-heading">घर्षण मॉडल</h3>
<div className="underline-class"></div>
```xml
<!-- उच्च घर्षण (रबर) -->
<surface><friction><ode><mu>1.0</mu></ode></friction></surface>
<!-- कम घर्षण (बर्फ) -->
<surface><friction><ode><mu>0.1</mu></ode></friction></surface>
```

<h2 className="second-heading">गतिशील तत्व</h2>
<div className="underline-class"></div>

<h3 className="third-heading">गतिशील वस्तुएँ</h3>
<div className="underline-class"></div>
```xml
<model name="moving_platform">
  <joint type="prismatic">
    <axis><xyz>0 1 0</xyz></axis>
  </joint>
</model>
```

<h3 className="third-heading">इंटरैक्टिव वस्तुएँ</h3>
<div className="underline-class"></div>
```xml
<model name="cup">
  <link><collision><geometry>
    <cylinder><radius>0.05</radius></cylinder>
  </geometry></collision></link>
</model>
```

<h2 className="second-heading">प्रकाश</h2>
<div className="underline-class"></div>
```xml
<!-- दिशात्मक (सूर्य) -->
<light type="directional"><direction>-0.5 0.1 -0.9</direction></light>
<!-- पॉइंट लाइट -->
<light type="point"><pose>0 0 3 0 0 0</pose></light>
```

<h2 className="second-heading">सर्वोत्तम प्रथाएँ</h2>
<div className="underline-class"></div>

- • स्थैतिक वस्तुओं को स्थैतिक के रूप में चिह्नित करें
- • संघर्ष ज्यामिति को सरल बनाएं
- • गतिशील तत्वों को सीमित रखें
- • मेश जटिलता को अनुकूलित करें
- • मॉड्यूलर डिज़ाइन का उपयोग करें

<h3 className="third-heading">मॉड्यूलर डिज़ाइन</h3>
<div className="underline-class"></div>
```xml
<world name="modular_environment">
  <include><uri>model://room_module_1</uri></include>
  <include><uri>model://room_module_2</uri></include>
</world>
```

<h3 className="third-heading">वर्ल्ड जनरेशन</h3>
<div className="underline-class"></div>
```python
import xml.etree.ElementTree as ET

def create_world_with_obstacles(world_name, obstacles):
    sdf = ET.Element('sdf', version='1.7')
    world = ET.SubElement(sdf, 'world', name=world_name)
    # बाधाएँ जोड़ें
    tree = ET.ElementTree(sdf)
    tree.write(f'{world_name}.world')
```

<h2 className="second-heading">सारांश</h2>
<div className="underline-class"></div>

सिमुलेशन वातावरण बनाते समय वास्तविकता को प्रदर्शन के साथ संतुलित करें। पुन: उपयोग के लिए मॉड्यूलर डिज़ाइन का उपयोग करें और प्रभावी मानवरूपी रोबोट परीक्षण के लिए उचित भौतिकी कॉन्फ़िगरेशन बनाए रखें।