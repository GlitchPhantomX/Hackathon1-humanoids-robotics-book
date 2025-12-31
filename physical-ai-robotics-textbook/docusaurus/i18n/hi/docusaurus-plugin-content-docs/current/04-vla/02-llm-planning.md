---
sidebar_position: 2
title: "बड़ा भाषा मॉडल योजना"
description: "उच्च-स्तरीय रोबोटिक कार्य योजना और निर्णय लेने के लिए LLMs का उपयोग करना"
---

import ReadingTime from '@site/src/components/ReadingTime';

# बड़ा भाषा मॉडल योजना

<div className="main-heading"></div>
<div className="underline-class"></div>


<ReadingTime minutes={25} />

<div className="border-line"></div>

## सीखने के उद्देश्य

<div className="second-heading"></div>
<div className="underline-class"></div>

- • रोबोटिक कार्य योजना के लिए LLMs को समझना
- • LLM-आधारित योजना प्रणाली लागू करना
- • प्रॉम्प्ट इंजीनियरिंग रणनीतियां डिज़ाइन करना
- • निष्पादन प्रणालियों के साथ LLM योजनाकार एकीकृत करना
- • LLM-आधारित योजना प्रभावशीलता का मूल्यांकन करना

<div className="border-line"></div>

## परिचय

<div className="second-heading"></div>
<div className="underline-class"></div>

LLMs रोबोटिक योजना के लिए सामान्य-ज्ञान तर्क, प्रक्रियात्मक ज्ञान और प्राकृतिक भाषा समझ प्रदान करते हैं। क्लासिकल योजना के विपरीत, LLMs विश्व ज्ञान का उपयोग करके प्राकृतिक भाषा निर्देशों से परिष्कृत कार्य योजनाएं उत्पन्न करने के लिए।

<div className="border-line"></div>

## LLM मौलिक सिद्धांत

<div className="second-heading"></div>
<div className="underline-class"></div>

### - LLM क्षमताएं

<div className="third-heading"></div>
<div className="underline-class"></div>

- • दुनिया के संचालन के बारे में सामान्य-ज्ञान तर्क
- • कार्य अनुक्रम के लिए प्रक्रियात्मक ज्ञान
- • प्राकृतिक भाषा समझ
- • सांदर्भिक तर्क और अनुकूलन

<div className="border-line"></div>

### - कार्य विभाजन

<div className="third-heading"></div>
<div className="underline-class"></div>

```python
class LLMTaskDecomposer:
    def __init__(self, model_name="gpt-3.5-turbo"):
        self.model_name = model_name
        openai.api_key = os.getenv("OPENAI_API_KEY")

    def decompose_task(self, task_description, robot_capabilities, environment_context):
        prompt = f"""
        Task: {task_description}
        Capabilities: {', '.join(robot_capabilities)}
        Context: {json.dumps(environment_context)}

        Return JSON with steps, actions, and parameters.
        """

        response = openai.ChatCompletion.create(
            model=self.model_name,
            messages=[{"role": "user", "content": prompt}],
            temperature=0.3
        )
        return json.loads(response.choices[0].message.content)
```

<div className="border-line"></div>

## योजना वास्तुकला

<div className="second-heading"></div>
<div className="underline-class"></div>

### - पदानुक्रमित योजना

<div className="third-heading"></div>
<div className="underline-class"></div>

```python
class HierarchicalLLMPlanner:
    def __init__(self, llm_planner, low_level_planner):
        self.llm_planner = llm_planner
        self.low_level_planner = low_level_planner

    def generate_plan(self, task, context):
        high_level_plan = self.llm_planner.decompose_task(task, self.get_capabilities(), context)

        refined_plan = {"high_level_steps": [], "execution_sequence": []}
        for step in high_level_plan["decomposed_steps"]:
            refined_step = self.refine_step(step, context)
            refined_plan["high_level_steps"].append(refined_step)
            if "low_level_actions" in refined_step:
                refined_plan["execution_sequence"].extend(refined_step["low_level_actions"])

        return refined_plan
```

<div className="border-line"></div>

### - प्रतिक्रियाशील योजना

<div className="third-heading"></div>
<div className="underline-class"></div>

```python
class ReactiveLLMPlanner:
    def __init__(self, llm_planner):
        self.llm_planner = llm_planner
        self.current_plan = None
        self.plan_index = 0

    def execute_with_monitoring(self, task, initial_context):
        self.current_plan = self.llm_planner.decompose_task(task, self.get_capabilities(), initial_context)

        while self.plan_index < len(self.current_plan["decomposed_steps"]):
            step = self.current_plan["decomposed_steps"][self.plan_index]
            success, new_context = self.execute_step(step)

            if success:
                self.plan_index += 1
            else:
                self.handle_failure(step, new_context)
```

<div className="border-line"></div>

## Isaac एकीकरण

<div className="second-heading"></div>
<div className="underline-class"></div>

### - Isaac LLM घटक

<div className="third-heading"></div>
<div className="underline-class"></div>

```python
class IsaacLLMPlannerNode(Node):
    def __init__(self):
        super().__init__('llm_planner_node')
        self.llm_planner = LLMTaskDecomposer()
        self.task_sub = self.create_subscription(String, '/robot_tasks', self.task_callback, 10)
        self.plan_pub = self.create_publisher(String, '/generated_plans', 10)

    def task_callback(self, msg):
        task_data = json.loads(msg.data)
        context = {'map_data': self.current_map, 'objects': self.detected_objects}
        plan = self.llm_planner.decompose_task(task_data['task'], context)
        self.plan_pub.publish(String(data=json.dumps(plan)))
```

<div className="border-line"></div>

### - धारणा एकीकरण

<div className="third-heading"></div>
<div className="underline-class"></div>

```python
class PerceptionEnhancedLLMPlanner:
    def __init__(self, llm_planner):
        self.llm_planner = llm_planner
        self.perception_system = PerceptionSystem()

    def plan_with_perception(self, task):
        context = {
            'objects': self.perception_system.get_object_poses(),
            'map': self.perception_system.get_environment_map(),
            'robot_position': self.get_robot_position()
        }
        return self.llm_planner.decompose_task(task, self.get_capabilities(), context)
```

<div className="border-line"></div>

## प्रॉम्प्ट इंजीनियरिंग

<div className="second-heading"></div>
<div className="underline-class"></div>

### - प्रभावी रणनीतियां

<div className="third-heading"></div>
<div className="underline-class"></div>

```python
class AdvancedPromptEngineer:
    def create_context_aware_prompt(self, task, context, robot_spec):
        return f"""
        Robot: {robot_spec['description']}
        Actions: {', '.join(robot_spec['actions'])}

        Context:
        - Objects: {', '.join(context.get('detected_objects', []))}
        - Location: {context.get('robot_position')}

        Task: {task}

        Return JSON with plan_id, steps (action, parameters, verification), confidence.
        """
```

<div className="border-line"></div>

### - सुरक्षा प्रॉम्प्ट

<div className="third-heading"></div>
<div className="underline-class"></div>

```python
class SafetyPromptEngineer:
    def create_safe_planning_prompt(self, task, context, safety_constraints):
        return f"""
        SAFETY-CRITICAL PLANNING

        Constraints: {', '.join(safety_constraints)}
        Humans: {context.get('humans_nearby')}

        Task: {task}

        Each step must include: safety_checks, risk_level, safety_annotations.
        """
```

<div className="border-line"></div>

## मूल्यांकन

<div className="second-heading"></div>
<div className="underline-class"></div>

### - गुणवत्ता मेट्रिक्स

<div className="third-heading"></div>
<div className="underline-class"></div>

```python
class LLMPlanningEvaluator:
    def evaluate_plan_quality(self, plan, task_requirements):
        return {
            'completeness': self.evaluate_completeness(plan, task_requirements),
            'feasibility': self.evaluate_feasibility(plan),
            'optimality': self.evaluate_optimality(plan),
            'safety': self.evaluate_safety(plan),
            'robustness': self.evaluate_robustness(plan)
        }
```

<div className="border-line"></div>

## सर्वोत्तम प्रथाएं

<div className="second-heading"></div>
<div className="underline-class"></div>

### - कार्यान्वयन

<div className="third-heading"></div>
<div className="underline-class"></div>

- • सुसंगत प्रॉम्प्ट प्रारूप का उपयोग करें
- • संदर्भ को ध्यान से प्रबंधित करें
- • मजबूत त्रुटि निपटान लागू करें
- • सुरक्षा को प्राथमिकता दें
- • निष्पादन से पहले आउटपुट को मान्य करें

<div className="border-line"></div>

### - प्रदर्शन

<div className="third-heading"></div>
<div className="underline-class"></div>

- • पुनरावर्ती योजनाओं को कैश करें
- • अनुरोधों को समानांतर प्रक्रिया करें
- • उपयुक्त मॉडल आकार चुनें
- • संदर्भ विंडोज़ को कुशलता से प्रबंधित करें

<div className="border-line"></div>

## समस्या निवारण

<div className="second-heading"></div>
<div className="underline-class"></div>

**API समस्याएं**: क्रेडेंशियल सत्यापित करें, पुन: प्रयास लागू करें, फ़ॉलबैक का उपयोग करें, दर सीमा अनुकूलित करें

**अमान्य योजनाएं**: संरचित प्रॉम्प्ट का उपयोग करें, आउटपुट मान्य करें, सुरक्षा बाधाएं लागू करें, कार्य स्थान परिभाषित करें

**संदर्भ समस्याएं**: उचित राज्य प्रतिनिधित्व, सिंक्रनाइज़ अद्यतन, अवलोकन आवृत्ति अनुकूलित करें

**प्रदर्शन**: योजनाओं को कैश करें, प्रॉम्प्ट अनुकूलित करें, बैच प्रक्रिया, लागत मॉनिटर करें

<div className="border-line"></div>

## सारांश

<div className="second-heading"></div>
<div className="underline-class"></div>

LLMs प्राकृतिक भाषा को समझने और सामान्य-ज्ञान तर्क का उपयोग करके परिष्कृत योजनाएं उत्पन्न करने के लिए रोबोट को सक्षम बनाते हैं। सफलता के लिए सावधान प्रॉम्प्ट इंजीनियरिंग, पदानुक्रमित वास्तुकला, सुरक्षा मान्यकरण और मॉडल क्षमताओं और सीमाओं को समझने की आवश्यकता होती है।

<div className="border-line"></div>

## अभ्यास

<div className="second-heading"></div>
<div className="underline-class"></div>

1. • मोबाइल रोबोट के लिए LLM योजनाकार लागू करें
2. • हेरफेर के लिए प्रॉम्प्ट रणनीतियां डिज़ाइन करें
3. • पदानुक्रमित योजना प्रणाली बनाएं
4. • सुरक्षा मान्यकरण प्रणाली विकसित करें
5. • मूल्यांकन ढांचा बनाएं

<div className="border-line"></div>

<h2 className="second-heading">आगे की पढ़ाई</h2>
<div className="underline-class"></div>

- • "Language Models as Zero-Shot Planners" चेन एट अल. द्वारा
- • "Inner Monologue: Embodied Reasoning" हा एट अल. द्वारा
- • NVIDIA Isaac AI एकीकरण दस्तावेज़
- • "Robot Learning from Language" मिश्रा एट अल. द्वारा