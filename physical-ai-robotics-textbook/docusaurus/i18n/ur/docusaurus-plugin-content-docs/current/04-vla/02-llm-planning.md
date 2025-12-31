---
sidebar_position: 2
title: "بڑے لینگویج ماڈل پلاننگ"
description: "ہائی لیول روبوٹک ٹاسک پلاننگ اور فیصلہ سازی کے لیے LLMs کا استعمال"
---

import ReadingTime from '@site/src/components/ReadingTime';
import ViewToggle from '@site/src/components/ViewToggle';


<ReadingTime minutes={25} />

<div className="border-line"></div>

<h2 className="second-heading">سیکھنے کے اہداف</h2>
<div className="underline-class"></div>

- • روبوٹک ٹاسک پلاننگ کے لیے LLMs کو سمجھنا
- • LLM-بیسڈ پلاننگ سسٹمز کو نافذ کرنا
- • پروموٹ انجینئرنگ کی حکمت عملیاں ڈیزائن کرنا
- • ایگزیکیوشن سسٹمز کے ساتھ LLM پلینرز کو یکجا کرنا
- • LLM-بیسڈ پلاننگ کی مؤثرتا کا جائزہ لینا

<div className="border-line"></div>

<h2 className="second-heading">تعارف</h2>
<div className="underline-class"></div>

LLMs عام سینس کی تشریح، طریقہ کار کا علم، اور روبوٹک پلاننگ کے لیے قدرتی لینگویج کی سمجھ کی پیش کش کرتے ہیں۔ کلاسیکل پلاننگ کے برعکس، LLMs دنیا کے علم کو قدرتی لینگویج ہدایات سے جامع ٹاسک پلان تیار کرنے کے لیے استعمال کرتے ہیں۔

<div className="border-line"></div>

<h2 className="second-heading">LLM بنیادیات</h2>
<div className="underline-class"></div>

<h3 className="third-heading">- LLM صلاحیات</h3>
<div className="underline-class"></div>

- • دنیا کے کاموں کے بارے میں عام سینس کی تشریح
- • ٹاسک ترتیبات کے لیے طریقہ کار کا علم
- • قدرتی لینگویج کی سمجھ
- • متن کے مطابق تشریح اور ایڈاپٹیشن

<div className="border-line"></div>

<h3 className="third-heading">- ٹاسک کی تقسیم</h3>
<div className="underline-class"></div>

```python
class TaskDecomposer:
    def __init__(self, llm_model):
        self.llm = llm_model
        self.decomposition_prompt = """
        Task: {task_description}
        Decompose this task into subtasks:
        1. [Subtask 1]
        2. [Subtask 2]
        3. [Subtask 3]
        ...
        """

    def decompose_task(self, task_description):
        prompt = self.decomposition_prompt.format(task_description=task_description)
        response = self.llm.generate(prompt)
        return self.parse_subtasks(response)
```

<div className="border-line"></div>

<h3 className="third-heading">- منطق کی تشریح</h3>
<div className="underline-class"></div>

```python
class LogicalReasoner:
    def __init__(self, llm_model):
        self.llm = llm_model

    def reason_about_task(self, task_context, constraints):
        prompt = f"""
        Given the task context: {task_context}
        With constraints: {constraints}
        What is the logical sequence of actions?
        """
        response = self.llm.generate(prompt)
        return self.extract_logical_sequence(response)
```

<div className="border-line"></div>

<h2 className="second-heading">LLM پلاننگ کے طریقے</h2>
<div className="underline-class"></div>

<h3 className="third-heading">- پروموٹ انجینئرنگ</h3>
<div className="underline-class"></div>

```python
class PromptEngineer:
    def __init__(self):
        self.templates = {
            'planning': "Given the goal: {goal}\nContext: {context}\nConstraints: {constraints}\nPlan the sequence of actions.",
            'reasoning': "Explain why {action} is needed in this context.",
            'validation': "Check if {plan} satisfies {goal} with {constraints}."
        }

    def create_planning_prompt(self, goal, context, constraints):
        return self.templates['planning'].format(
            goal=goal,
            context=context,
            constraints=constraints
        )
```

<div className="border-line"></div>

<h3 className="third-heading">- کنٹرول کی تقسیم</h3>
<div className="underline-class"></div>

```python
class HierarchicalPlanner:
    def __init__(self, llm_model):
        self.llm = llm_model
        self.high_level_planner = HighLevelPlanner(llm_model)
        self.low_level_planner = LowLevelPlanner()

    def plan_hierarchical(self, high_level_goal):
        high_level_plan = self.high_level_planner.plan(high_level_goal)
        detailed_plan = []
        for high_level_action in high_level_plan:
            detailed_subplan = self.low_level_planner.plan(high_level_action)
            detailed_plan.extend(detailed_subplan)
        return detailed_plan
```

<div className="border-line"></div>

<h2 className="second-heading">LLM-روبوٹک یکجہتی</h2>
<div className="underline-class"></div>

<h3 className="third-heading">- ایکشن سیکوئنسنگ</h3>
<div className="underline-class"></div>

```python
class ActionSequencer:
    def __init__(self, llm_model):
        self.llm = llm_model

    def sequence_actions(self, plan, robot_capabilities):
        prompt = f"""
        Given plan: {plan}
        Robot capabilities: {robot_capabilities}
        Sequence actions considering dependencies and capabilities.
        """
        response = self.llm.generate(prompt)
        return self.parse_action_sequence(response)
```

<div className="border-line"></div>

<h3 className="third-heading">- ایکشن والیڈیشن</h3>
<div className="underline-class"></div>

```python
class ActionValidator:
    def __init__(self, llm_model):
        self.llm = llm_model

    def validate_action(self, action, context, constraints):
        prompt = f"""
        Action: {action}
        Context: {context}
        Constraints: {constraints}
        Is this action valid? Why or why not?
        """
        response = self.llm.generate(prompt)
        return self.parse_validation_result(response)
```

<div className="border-line"></div>

<h2 className="second-heading">LLM پلاننگ کے چیلنج</h2>
<div className="underline-class"></div>

<h3 className="third-heading">- درستگی کی چیلنج</h3>
<div className="underline-class"></div>

- • LLMs کے جوابات کی توثیق کرنا
- • غلط منطق کو روکنا
- • کنٹرول کی اقسام کی توثیق کرنا

<div className="border-line"></div>

<h3 className="third-heading">- کارکردگی کی چیلنج</h3>
<div className="underline-class"></div>

- • جلد جواب کے لیے پروسیسنگ کو تیز کرنا
- • ڈیٹا کے اخراج کو کم کرنا
- • میموری کے استعمال کو کم کرنا

<div className="border-line"></div>

<h2 className="second-heading">LLM پلاننگ کی مثالیں</h2>
<div className="underline-class"></div>

<h3 className="third-heading">- ہوم اسسٹنٹ پلاننگ</h3>
<div className="underline-class"></div>

```python
class HomeAssistantPlanner:
    def __init__(self, llm_model):
        self.llm = llm_model

    def plan_daily_routine(self, user_preferences, schedule, constraints):
        prompt = f"""
        User preferences: {user_preferences}
        Daily schedule: {schedule}
        Constraints: {constraints}
        Plan a daily routine considering preferences and constraints.
        """
        response = self.llm.generate(prompt)
        return self.parse_daily_plan(response)
```

<div className="border-line"></div>

<h3 className="third-heading">- روبوٹ نیویگیشن پلاننگ</h3>
<div className="underline-class"></div>

```python
class NavigationPlanner:
    def __init__(self, llm_model):
        self.llm = llm_model

    def plan_navigation(self, start_location, destination, obstacles, preferences):
        prompt = f"""
        Start: {start_location}
        Destination: {destination}
        Obstacles: {obstacles}
        Preferences: {preferences}
        Plan navigation route and actions.
        """
        response = self.llm.generate(prompt)
        return self.parse_navigation_plan(response)
```

<div className="border-line"></div>

<h2 className="second-heading">جائزہ</h2>
<div className="underline-class"></div>

<h3 className="third-heading">- سسٹم جائزہ</h3>
<div className="underline-class"></div>

```python
class LLMPlannerEvaluator:
    def evaluate_planner(self, planner, test_scenarios):
        results = []
        for scenario in test_scenarios:
            plan = planner.plan(scenario.goal, scenario.context)
            success = self.execute_and_evaluate_plan(plan, scenario)
            results.append({
                'success': success,
                'plan_quality': self.assess_plan_quality(plan),
                'execution_time': self.measure_execution_time(plan)
            })
        return self.calculate_overall_metrics(results)
```

<div className="border-line"></div>

<h2 className="second-heading">بہترین طریقے</h2>
<div className="underline-class"></div>

<h3 className="third-heading">- ڈیزائن اصول</h3>
<div className="underline-class"></div>

- • LLM پلاننگ کو ہارڈ کوڈڈ لاجک کے ساتھ جوڑیں
- • ہر پلان کی توثیق کے لیے متعدد چیکس استعمال کریں
- • ڈومین کے علم کو LLM کے ساتھ یکجا کریں
- • ایکشن کی ناکامی کے لیے بازیافت کے طریقے نافذ کریں

<div className="border-line"></div>

<h3 className="third-heading">- کارکردگی کی بہتری</h3>
<div className="underline-class"></div>

- • LLM کالز کو کم کرنے کے لیے کیش کا استعمال کریں
- • کم اہمیت کے ٹاسکس کے لیے چھوٹے ماڈلز استعمال کریں
- • کیچرل اسٹوریج کا استعمال کریں تاکہ ایک جیسے ٹاسکس کے لیے دوبارہ پلاننگ نہ کی جائے

<div className="border-line"></div>

<h2 className="second-heading">خلاصہ</h2>
<div className="underline-class"></div>

LLM پلاننگ روبوٹکس کے لیے طاقتور اوزار ہے جو عام سینس کی تشریح، طریقہ کار کا علم، اور قدرتی لینگویج کی سمجھ کو جمع کرتا ہے۔ LLMs کے ساتھ کام کرتے ہوئے چیلنجوں کو سمجھنا اور منظم کرنا اہم ہے تاکہ قابل اعتماد اور مؤثر پلاننگ سسٹمز نافذ کیے جا سکیں۔

<div className="border-line"></div>

<h2 className="second-heading">ورقے</h2>
<div className="underline-class"></div>

1. • LLM-بیسڈ پلاننگ سسٹم نافذ کریں
2. • پروموٹ انجینئرنگ کے ٹیکنیکس کو ٹیسٹ کریں
3. • ہوم اسسٹنٹ سکینریو کے لیے پلان تیار کریں
4. • نیویگیشن پلاننگ کے لیے جائزہ فریم ورک تیار کریں
5. • LLM پلاننگ کی کارکردگی کا جائزہ لیں

<div className="border-line"></div>

<h2 className="second-heading">مزید پڑھائی</h2>
<div className="underline-class"></div>

- • "Language Models for Robotics" by Brohan et al.
- • "LLM-based Task Planning" by Say et al.
- • "Hierarchical Planning with LLMs" by Chen et al.
- • "Grounded Language Understanding for Robotics" by Misra et al.