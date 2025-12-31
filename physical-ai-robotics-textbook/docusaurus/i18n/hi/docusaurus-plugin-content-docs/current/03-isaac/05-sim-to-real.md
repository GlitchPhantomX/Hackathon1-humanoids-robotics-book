---
sidebar_position: 5
title: "सिमुलेशन से वास्तविकता में स्थानांतरण"
description: "Isaac पारिस्थितिकी तंत्र का उपयोग करके सिमुलेशन से वास्तविक दुनिया में रोबोट प्रणालियों को स्थानांतरित करना"
---

import ReadingTime from '@site/src/components/ReadingTime';
import ViewToggle from '@site/src/components/ViewToggle';

<ReadingTime minutes={18} />


<h1 className="main-heading">सिमुलेशन से वास्तविकता में स्थानांतरण</h1>
<div className="underline-class"></div>

<div className="border-line"></div>

<h2 className="second-heading">सीखने के उद्देश्य</h2>
<div className="underline-class"></div>

- • सिम-टू-रियल स्थानांतरण चुनौतियों और समाधानों को समझना
- • डोमेन रैंडमाइजेशन तकनीकें लागू करना
- • सिमुलेशन-प्रशिक्षित मॉडल को वास्तविक हार्डवेयर पर तैनात करना
- • रोबोट प्रणालियों को कैलिब्रेट और मान्य करना
- • भौतिक तैनाती के लिए पर्सेप्शन और नियंत्रण को अनुकूलित करना

<div className="border-line"></div>

<h2 className="second-heading">परिचय</h2>
<div className="underline-class"></div>

सिम-टू-रियल स्थानांतरण सिमुलेशन में विकसित प्रणालियों को सफलतापूर्वक वास्तविक रोबोट पर तैनात करता है। Isaac पारिस्थितिकी तंत्र डोमेन रैंडमाइजेशन, सिंथेटिक डेटा उत्पादन और हार्डवेयर-इन-द-लूप परीक्षण के माध्यम से इस अंतर को पाटने के लिए उपकरण प्रदान करता है।

<div className="border-line"></div>

<h2 className="second-heading">वास्तविकता अंतर को समझना</h2>
<div className="underline-class"></div>

<h3 className="third-heading">- भौतिकी और गतिशीलता अंतर</h3>
<div className="underline-class"></div>

वास्तविकता अंतर में भौतिकी अनुमान, गतिशीलता मॉडलिंग अशुद्धियां, एक्चुएटर व्यवहार अंतर और सेंसर विशेषता भिन्नताएं शामिल हैं।

```python
class PhysicsCalibrator:
    def __init__(self):
        self.sim_params = {'gravity': 9.81, 'friction': 0.5, 'damping': 0.1}
        self.real_params = {'gravity': 9.81, 'friction': 0.45, 'damping': 0.12}

    def get_adjusted_params(self):
        adjusted = self.sim_params.copy()
        adjusted['friction'] *= self.real_params['friction'] / self.sim_params['friction']
        return adjusted
```

<div className="border-line"></div>

<h2 className="second-heading">डोमेन रैंडमाइजेशन</h2>
<div className="underline-class"></div>

<h3 className="third-heading">- दृश्य रैंडमाइजेशन</h3>
<div className="underline-class"></div>

```python
class DomainRandomizer:
    def __init__(self):
        self.lighting_range = {'intensity': (0.5, 2.0), 'color_temp': (3000, 8000)}
        self.material_range = {'roughness': (0.1, 0.9), 'metallic': (0.0, 1.0)}

    def randomize_lighting(self, light_actor):
        intensity = random.uniform(*self.lighting_range['intensity'])
        light_actor.set_attribute('intensity', intensity)

    def randomize_materials(self, material):
        material.set_roughness(random.uniform(*self.material_range['roughness']))
        material.set_metallic(random.uniform(*self.material_range['metallic']))
```

<div className="border-line"></div>

<h3 className="third-heading">- भौतिकी रैंडमाइजेशन</h3>
<div className="underline-class"></div>

```python
class PhysicsDomainRandomizer:
    def __init__(self):
        self.param_ranges = {'mass_mult': (0.8, 1.2), 'friction_mult': (0.7, 1.3)}

    def randomize_robot_dynamics(self, robot):
        for link in robot.links:
            link.mass *= random.uniform(*self.param_ranges['mass_mult'])
            link.friction *= random.uniform(*self.param_ranges['friction_mult'])
```

<div className="border-line"></div>

<h2 className="second-heading">Isaac Sim रैंडमाइजेशन</h2>
<div className="underline-class"></div>

<h3 className="third-heading">- अंतर्निहित उपकरण</h3>
<div className="underline-class"></div>

```python
class IsaacSimDomainRandomizer:
    def setup_domain_randomization(self):
        for i in range(10):
            material = VisualMaterial(
                prim_path=f"/World/Looks/Material_{i}",
                diffuse_color=(random.random(), random.random(), random.random()),
                metallic=random.uniform(0.0, 1.0)
            )
            self.materials.append(material)
```

<div className="border-line"></div>

<h3 className="third-heading">- सिंथेटिक डेटा उत्पादन</h3>
<div className="underline-class"></div>

```python
class SyntheticDataGenerator:
    def capture_synthetic_data(self, num_samples=1000):
        for i in range(num_samples):
            self.randomizer.randomize_materials()
            rgb = self.capture_rgb_image()
            depth = self.capture_depth_image()
            self.save_synthetic_sample(rgb, depth, sample_id=i)
```

<div className="border-line"></div>

<h2 className="second-heading">Isaac ROS एकीकरण</h2>
<div className="underline-class"></div>

<h3 className="third-heading">- हार्डवेयर-इन-द-लूप परीक्षण</h3>
<div className="underline-class"></div>

```python
class HardwareInLoopNode(Node):
    def __init__(self):
        super().__init__('hil_node')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.rgb_sub = self.create_subscription(Image, '/camera/rgb', self.rgb_callback, 10)
        self.sim_interface = IsaacSimInterface()

    def control_loop(self):
        sim_state = self.sim_interface.get_robot_state()
        control_cmd = self.apply_control_algorithm(sim_state)
        self.cmd_vel_pub.publish(control_cmd)
```

<div className="border-line"></div>

<h3 className="third-heading">- पर्सेप्शन पाइपलाइन तैनाती</h3>
<div className="underline-class"></div>

```python
class RealWorldPerceptionPipeline:
    def __init__(self, model_path):
        self.model = torch.load(model_path).cuda().eval()
        self.trt_model = torch2trt(self.model, [torch.randn(1, 3, 640, 480).cuda()], fp16_mode=True)

    def process_frame(self, image):
        input_tensor = self.preprocess_image(image)
        with torch.no_grad():
            output = self.trt_model(input_tensor)
        return self.postprocess_output(output)
```

<div className="border-line"></div>

<h2 className="second-heading">कैलिब्रेशन और मान्यकरण</h2>
<div className="underline-class"></div>

<h3 className="third-heading">- सेंसर कैलिब्रेशन</h3>
<div className="underline-class"></div>

```python
class MultiSensorCalibrator:
    def calibrate_camera(self, calibration_images, checkerboard_size=(9, 6)):
        obj_points, img_points = [], []
        objp = np.zeros((checkerboard_size[0] * checkerboard_size[1], 3), np.float32)

        for img in calibration_images:
            ret, corners = cv2.findChessboardCorners(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), checkerboard_size)
            if ret:
                obj_points.append(objp)
                img_points.append(corners)

        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, img.shape[:2], None, None)
        return ret, mtx, dist
```

<div className="border-line"></div>

<h3 className="third-heading">- प्रदर्शन मान्यकरण</h3>
<div className="underline-class"></div>

```python
class PerformanceValidator:
    def validate_perception(self, ground_truth, predictions):
        accuracy = self.calculate_accuracy(ground_truth, predictions)
        precision = self.calculate_precision(ground_truth, predictions)
        recall = self.calculate_recall(ground_truth, predictions)
        return {'accuracy': accuracy, 'precision': precision, 'recall': recall}

    def assess_sim_real_gap(self):
        gap_analysis = {}
        for metric, current_value in self.metrics.items():
            baseline = self.baseline_performance[metric]
            gap_analysis[metric] = {'gap': abs(current_value - baseline) / baseline}
        return gap_analysis
```

<div className="border-line"></div>

<h2 className="second-heading">तैनाती रणनीतियां</h2>
<div className="underline-class"></div>

<h3 className="third-heading">- प्रगतिशील तैनाती</h3>
<div className="underline-class"></div>

```python
class ProgressiveDeployer:
    def __init__(self):
        self.deployment_stages = ['simulation_only', 'hardware_in_loop', 'limited_real', 'full_real']
        self.current_stage = 0
        self.thresholds = {'simulation_only': 0.95, 'hardware_in_loop': 0.90}

    def advance_deployment_stage(self, performance_score):
        threshold = self.thresholds[self.deployment_stages[self.current_stage]]
        if performance_score >= threshold and self.current_stage < len(self.deployment_stages) - 1:
            self.current_stage += 1
            return True
        return False
```

<div className="border-line"></div>

<h3 className="third-heading">- सुरक्षा तंत्र</h3>
<div className="underline-class"></div>

```python
class SafetyFallbackSystem:
    def __init__(self):
        self.safety_limits = {'velocity': 1.0, 'acceleration': 2.0, 'temperature': 80.0}

    def monitor_safety(self, robot_state):
        violations = []
        if robot_state.velocity > self.safety_limits['velocity']:
            violations.append('velocity_exceeded')
        if violations:
            self.trigger_safety_action(violations)
        return violations
```

<div className="border-line"></div>

<h2 className="second-heading">सर्वोत्तम प्रथाएं</h2>
<div className="underline-class"></div>

<h3 className="third-heading">- स्थानांतरण अनुकूलन</h3>
<div className="underline-class"></div>

- • न्यूनतम रैंडमाइजेशन के साथ शुरू करें, धीरे-धीरे बढ़ाएं
- • सरल से जटिल तक पाठ्यक्रम सीखना उपयोग करें
- • विविध सेंसर मॉडलिटी के साथ प्रशिक्षण लें
- • कई स्थितियों में मान्यकरण करें

<div className="border-line"></div>

<h3 className="third-heading">- प्रदर्शन निगरानी</h3>
<div className="underline-class"></div>

- • संचालन के दौरान रीयल-टाइम मेट्रिक्स की निगरानी करें
- • ड्रिफ्ट का पता लगाएं और क्षतिपूर्ति करें
- • अनुकूली कैलिब्रेशन लागू करें
- • अनुभव से निरंतर सीखना सक्षम करें

<div className="border-line"></div>

<h2 className="second-heading">समस्या निवारण</h2>
<div className="underline-class"></div>

**डोमेन रैंडमाइजेशन अस्थिरता**: पैरामीटर सीमाएं सीमित करें, भौतिकी की पुष्टि करें, धीरे-धीरे परिवर्तन लागू करें

**कैलिब्रेशन विफलता**: लक्ष्य गुणवत्ता अनुकूलित करें, उत्तेजना सुनिश्चित करें, सिंक्रनाइज़ेशन में सुधार करें, वातावरण का ध्यान रखें

**HIL उच्च विलंबता**: नेटवर्क का अनुकूलन करें, रीयल-टाइम अनुसूचना लागू करें, कुशल डेटा प्रसंस्करण

**बड़ा सिम-रियल अंतर**: बेहतर रैंडमाइजेशन, सिस्टम पहचान, वास्तविकता अंतर मात्रात्मक निर्धारण

**संसाधन उपभोग**: रेंडरिंग का अनुकूलन करें, कुशल संसाधन प्रबंधन, समानांतर प्रसंस्करण

<div className="border-line"></div>

<h2 className="second-heading">सारांश</h2>
<div className="underline-class"></div>

सिम-टू-रियल स्थानांतरण डोमेन रैंडमाइजेशन, सिंथेटिक डेटा और HIL परीक्षण के माध्यम से सिमुलेशन और वास्तविकता को जोड़ता है। सफलता के लिए भौतिकी मॉडलिंग, सेंसर कैलिब्रेशन, सुरक्षा तंत्र के साथ प्रगतिशील तैनाती की आवश्यकता होती है। Isaac पारिस्थितिकी तंत्र वास्तविकता अंतर को प्रभावी ढंग से कम करने के लिए उपकरण प्रदान करता है।

<div className="border-line"></div>

<h2 className="second-heading">अभ्यास</h2>
<div className="underline-class"></div>

1. • Isaac Sim में डोमेन रैंडमाइजेशन लागू करें
2. • कैमरा-LiDAR प्रणाली कैलिब्रेट करें
3. • प्रगतिशील तैनाती रणनीति डिज़ाइन करें
4. • पर्सेप्शन प्रणाली प्रदर्शन मान्य करें
5. • सुरक्षा फॉलबैक प्रणाली बनाएं

<div className="border-line"></div>

<h2 className="second-heading">आगे की पढ़ाई</h2>
<div className="underline-class"></div>

- • "डोमेन रैंडमाइजेशन फॉर ट्रांसफेरिंग डीप न्यूरल नेटवर्क्स" टोबिन एट अल. द्वारा
- • "सिम-टू-रियल: लर्निंग एगल लोकोमोशन" हीस एट अल. द्वारा
- • NVIDIA Isaac डोमेन रैंडमाइजेशन दस्तावेज़
- • "घने स्टीरियो संगति एल्गोरिदम का वर्गीकरण" मान्यकरण के लिए