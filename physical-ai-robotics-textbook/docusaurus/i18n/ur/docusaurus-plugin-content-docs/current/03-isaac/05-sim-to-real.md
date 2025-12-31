---
sidebar_position: 5
title: "سیمولیشن سے حقیقت منتقلی"
description: "Isaac ایکو سسٹم کا استعمال کرتے ہوئے سیمولیشن سے حقیقی دنیا کے ڈیپلائمنٹ پر روبوٹ سسٹم منتقل کرنا"
---

import ReadingTime from '@site/src/components/ReadingTime';
import ViewToggle from '@site/src/components/ViewToggle';


<ReadingTime minutes={18} />

<h1 className="main-heading">سیمولیشن سے حقیقت منتقلی</h1>
<div className="underline-class"></div>

<div className="border-line"></div>

<h2 className="second-heading">سیکھنے کے اہداف</h2>
<div className="underline-class"></div>

- • سیمولیشن سے حقیقت منتقلی کے چیلنجز اور حل سمجھنا
- • ڈومین رینڈمائزیشن کی تکنیکیں نافذ کرنا
- • حقیقی ہارڈ ویئر پر سیمولیشن-ٹرینڈ ماڈلز ڈپلائی کرنا
- • روبوٹ سسٹم کیلیبریٹ اور توثیق کرنا
- • جسمانی ڈیپلائمنٹ کے لیے ادراک اور کنٹرول کو بہتر بنانا

<div className="border-line"></div>

<h2 className="second-heading">تعارف</h2>
<div className="underline-class"></div>

سیمولیشن سے حقیقت کے منتقلی سیمولیشن میں تیار کردہ سسٹم لیتا ہے اور انہیں جسمانی روبوٹس پر کامیابی سے ڈیپلائی کرتا ہے۔ Isaac ایکو سسٹم ڈومین رینڈمائزیشن، مصنوعی ڈیٹا جنریشن، اور ہارڈ ویئر-ان-لوپ ٹیسٹنگ کے ذریعے اس خل کو پانے کے لیے ٹولز فراہم کرتا ہے۔

<div className="border-line"></div>

<h2 className="second-heading">ریلٹی گیپ کو سمجھنا</h2>
<div className="underline-class"></div>

<h3 className="third-heading">- فزکس اور ڈائی نامکس کے فرق</h3>
<div className="underline-class"></div>

ریلٹی گیپ میں فزکس تقربات، ڈائی نامکس ماڈلنگ کی نادرستگی، ایکچوایٹر برتاؤ کے فرق، اور سینسر خصوصیات کی متغیرات شامل ہیں۔

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

<h2 className="second-heading">ڈومین رینڈمائزیشن</h2>
<div className="underline-class"></div>

<h3 className="third-heading">- وژوئل رینڈمائزیشن</h3>
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

<h3 className="third-heading">- فزکس رینڈمائزیشن</h3>
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

<h2 className="second-heading">Isaac Sim رینڈمائزیشن</h2>
<div className="underline-class"></div>

<h3 className="third-heading">- بلٹ ان ٹولز</h3>
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

<h3 className="third-heading">- مصنوعی ڈیٹا جنریشن</h3>
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

<h2 className="second-heading">Isaac ROS انضمام</h2>
<div className="underline-class"></div>

<h3 className="third-heading">- ہارڈ ویئر-ان-دی-لوپ ٹیسٹنگ</h3>
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

<h3 className="third-heading">- ادراک پائپ لائن ڈیپلائمنٹ</h3>
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

<h2 className="second-heading">کیلیبریشن اور توثیق</h2>
<div className="underline-class"></div>

<h3 className="third-heading">- سینسر کیلیبریشن</h3>
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

<h3 className="third-heading">- کارکردگی کی توثیق</h3>
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

<h2 className="second-heading">ڈیپلائمنٹ کی حکمت عملیاں</h2>
<div className="underline-class"></div>

<h3 className="third-heading">- تدریجی ڈیپلائمنٹ</h3>
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

<h3 className="third-heading">- حفاظتی مکانزم</h3>
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

<h2 className="second-heading">بہترین مشقیں</h2>
<div className="underline-class"></div>

<h3 className="third-heading">- منتقلی کی بہتری</h3>
<div className="underline-class"></div>

- • کم رینڈمائزیشن کے ساتھ شروع کریں، تدریج سے بڑھائیں
- • سادہ سے پیچیدہ تک منصوبہ بندی سیکھنا استعمال کریں
- • مختلف سینسر ماڈلیٹیز کے ساتھ تربیت کریں
- • متعدد حالت کے تحت توثیق کریں

<div className="border-line"></div>

<h3 className="third-heading">- کارکردگی کی نگرانی</h3>
<div className="underline-class"></div>

- • آپریشن کے دوران ریل ٹائم میٹرکس مانیٹر کریں
- • ڈرائیفٹ کو ڈیٹیکٹ اور معاوضہ دیں
- • ایڈاپٹو کیلیبریشن نافذ کریں
- • تجربے سے جاریہ لرننگ کو فعال کریں

<div className="border-line"></div>

<h2 className="second-heading">ٹربل شوٹنگ</h2>
<div className="underline-class"></div>

**ڈومین رینڈمائزیشن انسٹیبلیٹی**: پیرامیٹر رینج کو محدود کریں، فزکس کی توثیق کریں، تدریجی تبدیلیاں نافذ کریں

**کیلیبریشن ناکامیاں**: ہدف کی معیار کو بہتر بنائیں، ایکسیٹیشن کو یقینی بنائیں، ہم وقت سازی کو بہتر بنائیں، ماحول کو مدنظر رکھیں

**HIL زیادہ لیٹنسی**: نیٹ ورک کو بہتر بنائیں، ریل ٹائم شیڈولنگ نافذ کریں، کارآمد ڈیٹا پروسیسنگ

**بڑا سیم-ریل گیپ**: بہتر رینڈمائزیشن، سسٹم آئیڈنٹیفکیشن، حقیقت گیپ کمی کریں

**ریسورس کنسمپشن**: رینڈرنگ کو بہتر بنائیں، کارآمد ریسورس مینجمنٹ، متوازی پروسیسنگ

<div className="border-line"></div>

<h2 className="second-heading">خلاصہ</h2>
<div className="underline-class"></div>

سیمولیشن سے حقیقت کے منتقلی ڈومین رینڈمائزیشن، مصنوعی ڈیٹا، اور HIL ٹیسٹنگ کے ذریعے سیمولیشن اور حقیقت کو جوڑتا ہے۔ کامیابی کے لیے فزکس ماڈلنگ، سینسر کیلیبریشن، حفاظتی مکانزم کے ساتھ تدریجی ڈیپلائمنٹ کی ضرورت ہوتی ہے۔ Isaac ایکو سسٹم ریلٹی گیپ کو مؤثر طریقے سے کم کرنے کے لیے ٹولز فراہم کرتا ہے۔

<div className="border-line"></div>

<h2 className="second-heading">مشقیں</h2>
<div className="underline-class"></div>

1. • Isaac Sim میں ڈومین رینڈمائزیشن نافذ کریں
2. • کیمرہ-لیڈار سسٹم کیلیبریٹ کریں
3. • تدریجی ڈیپلائمنٹ کی حکمت عملی ڈیزائن کریں
4. • ادراک سسٹم کی کارکردگی کی توثیق کریں
5. • حفاظتی فیل بیک سسٹم تخلیق کریں

<div className="border-line"></div>

<h2 className="second-heading">مزید پڑھائی</h2>
<div className="underline-class"></div>

- • "Domain Randomization for Transferring Deep Neural Networks" by Tobin et al.
- • "Sim-to-Real: Learning Agile Locomotion" by Heess et al.
- • NVIDIA Isaac domain randomization docs
- • "Taxonomy of Dense Stereo Correspondence Algorithms" for validation