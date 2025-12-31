---
sidebar_position: 3
title: "वॉइस कमांड और इंटरैक्शन सिस्टम"
description: "ह्यूमनॉइड रोबोट के लिए वॉइस कमांड प्रोसेसिंग लागू करना"
---

import ReadingTime from '@site/src/components/ReadingTime';

<ReadingTime minutes={30} />


<h1 className="main-heading">वॉइस कमांड और इंटरैक्शन सिस्टम</h1>
<div className="underline-class"></div>

<div className="full-content">

<div className="border-line"></div>

<h2 className="second-heading">सीखने के उद्देश्य</h2>
<div className="underline-class"></div>

- व्यापक वॉइस कमांड प्रोसेसिंग सिस्टम डिज़ाइन करना
- NLU के साथ भाषा पहचान एकीकृत करना
- संदर्भ-जागरूक संवाद प्रबंधन बनाना
- वॉइस-आधारित कार्य योजना लागू करना
- वास्तविक दुनिया के वातावरण में मजबूत वॉइस इंटरैक्शन सुनिश्चित करना

<div className="border-line"></div>

<h2 className="second-heading">परिचय</h2>
<div className="underline-class"></div>

वॉइस कमांड सिस्टम मानव और ह्यूमनॉइड रोबोट के बीच प्राकृतिक संचार को सक्षम बनाता है। सिस्टम ऑडियो कैप्चर, भाषा-टू-टेक्स्ट रूपांतरण, इरादा समझ, संवाद प्रबंधन और क्रिया निष्पादन को संभालता है। विजन-भाषा-एक्शन पैराडिम का उपयोग करके, यह बोली गई भाषा को रोबोटिक क्रियाओं से जोड़ता है।

<div className="border-line"></div>

<h2 className="second-heading">भाषा पहचान</h2>
<div className="underline-class"></div>

<h3 className="third-heading">ऑडियो कैप्चर</h3>

```python
import pyaudio
import numpy as np
import webrtcvad

class AudioCaptureSystem:
    def __init__(self, sample_rate=16000, chunk_size=1024):
        self.sample_rate = sample_rate
        self.chunk_size = chunk_size
        self.audio = pyaudio.PyAudio()
        self.vad = webrtcvad.Vad(1)  # वॉइस एक्टिविटी डिटेक्शन
        self.audio_buffer = []

    def start_capture(self):
        self.stream = self.audio.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=self.sample_rate,
            input=True,
            frames_per_buffer=self.chunk_size
        )
        self.is_capturing = True

    def is_speech_detected(self, audio_chunk):
        energy = np.sqrt(np.mean(audio_chunk ** 2))
        if energy < 0.01:
            return False

        audio_int16 = (audio_chunk * 32767).astype(np.int16)
        return self.vad.is_speech(audio_int16.tobytes(), self.sample_rate)
```

<div className="border-line"></div>

<h3 className="third-heading">भाषा-टू-टेक्स्ट</h3>

```python
import speech_recognition as sr
import whisper

class SpeechRecognitionSystem:
    def __init__(self, model_type="whisper"):
        self.model_type = model_type
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

        if model_type == "whisper":
            self.whisper_model = whisper.load_model("base")

    def listen_for_speech(self, timeout=5.0):
        try:
            with self.microphone as source:
                audio = self.recognizer.listen(source, timeout=timeout)

            if self.model_type == "whisper":
                return self.recognize_with_whisper(audio)
            else:
                return self.recognizer.recognize_google(audio)

        except sr.WaitTimeoutError:
            return ""
        except Exception as e:
            return ""

    def recognize_with_whisper(self, audio):
        audio_data = np.frombuffer(audio.get_raw_data(), dtype=np.int16).astype(np.float32) / 32768.0
        result = self.whisper_model.transcribe(audio_data)
        return result["text"].strip()
```

<div className="border-line"></div>

<h2 className="second-heading">प्राकृतिक भाषा समझ</h2>
<div className="underline-class"></div>

<h3 className="third-heading">इरादा पहचान</h3>

```python
import spacy
import re

class NaturalLanguageUnderstandingSystem:
    def __init__(self):
        self.nlp = spacy.load("en_core_web_sm")

        self.intent_patterns = {
            'navigation': [r'go to (.+)', r'move to (.+)', r'navigate to (.+)'],
            'manipulation': [r'pick up (.+)', r'grab (.+)', r'get (.+)'],
            'information': [r'what is (.+)', r'where is (.+)'],
            'action': [r'open (.+)', r'close (.+)', r'turn (.+)']
        }

        self.entity_patterns = {
            'location': ['kitchen', 'bedroom', 'living room', 'office'],
            'object': ['cup', 'bottle', 'book', 'phone', 'keys']
        }

    def understand_command(self, text, context=None):
        doc = self.nlp(text) if self.nlp else None

        # इरादा निकालें
        intent_result = self.extract_intent(text)

        # एंटिटी निकालें
        entities = self.extract_entities(text, doc)

        return {
            'original_text': text,
            'intent': intent_result['label'],
            'intent_confidence': intent_result['confidence'],
            'entities': entities,
            'timestamp': time.time()
        }

    def extract_intent(self, text):
        text_lower = text.lower()

        for intent, patterns in self.intent_patterns.items():
            for pattern in patterns:
                if re.search(pattern, text_lower):
                    return {'label': intent, 'confidence': 0.9}

        return {'label': 'unknown', 'confidence': 0.1}

    def extract_entities(self, text, doc=None):
        entities = []
        text_lower = text.lower()

        for entity_type, patterns in self.entity_patterns.items():
            for pattern in patterns:
                if pattern in text_lower:
                    entities.append({
                        'text': pattern,
                        'label': entity_type.upper(),
                        'confidence': 0.7
                    })

        return entities
```

<div className="border-line"></div>

<h3 className="third-heading">संवाद प्रबंधन</h3>

```python
from enum import Enum
from dataclasses import dataclass

class DialogueState(Enum):
    IDLE = "idle"
    LISTENING = "listening"
    PROCESSING = "processing"
    EXECUTING = "executing"

@dataclass
class ConversationContext:
    current_task: dict = None
    task_history: list = None
    conversation_history: list = None

class ContextAwareDialogueManager:
    def __init__(self):
        self.current_state = DialogueState.IDLE
        self.context = ConversationContext(
            task_history=[],
            conversation_history=[]
        )
        self.nlu = NaturalLanguageUnderstandingSystem()

    def process_user_input(self, user_input):
        # इतिहास में जोड़ें
        self.context.conversation_history.append({
            'speaker': 'user',
            'text': user_input,
            'timestamp': time.time()
        })

        # कमांड समझें
        nlu_result = self.nlu.understand_command(user_input)

        # मान्यता दें
        if not self.validate_command(nlu_result):
            response = "मैंने समझा नहीं। क्या आप फिर से कह सकते हैं?"
        else:
            response = self.start_new_task(nlu_result)

        # प्रतिक्रिया को इतिहास में जोड़ें
        self.context.conversation_history.append({
            'speaker': 'robot',
            'text': response,
            'timestamp': time.time()
        })

        return {'text': response, 'intent': nlu_result['intent']}

    def start_new_task(self, nlu_result):
        task = {
            'id': f"task_{len(self.context.task_history) + 1}",
            'intent': nlu_result['intent'],
            'entities': nlu_result['entities'],
            'status': 'pending'
        }

        self.context.current_task = task
        self.context.task_history.append(task)

        entities = nlu_result['entities']
        if entities:
            return f"ठीक है, मैं {nlu_result['intent']} {entities[0]['text']} करूंगा।"
        return f"{nlu_result['intent']} कार्य शुरू कर रहा है।"
```

<div className="border-line"></div>

<h2 className="second-heading">Isaac एकीकरण</h2>
<div className="underline-class"></div>

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class IsaacVoiceProcessingNode(Node):
    def __init__(self):
        super().__init__('voice_processing_node')

        # पब्लिशर
        self.text_pub = self.create_publisher(String, '/speech_to_text', 10)
        self.command_pub = self.create_publisher(String, '/robot_commands', 10)

        # सब्सक्राइबर
        self.audio_sub = self.create_subscription(AudioData, '/audio_input', self.audio_cb, 10)

        # सिस्टम शुरू करें
        self.speech_recognizer = SpeechRecognitionSystem()
        self.nlu = NaturalLanguageUnderstandingSystem()
        self.dialogue = ContextAwareDialogueManager()

    def audio_cb(self, msg):
        try:
            # ऑडियो को एरे में बदलें
            audio_array = np.frombuffer(msg.data, dtype=np.int16).astype(np.float32) / 32768.0

            # भाषा पहचानें
            text = self.speech_recognizer.recognize_with_whisper(audio_array)

            if text:
                # पहचाने गए टेक्स्ट को पब्लिश करें
                text_msg = String()
                text_msg.data = json.dumps({'text': text})
                self.text_pub.publish(text_msg)

                # NLU के साथ प्रक्रिया करें
                nlu_result = self.nlu.understand_command(text)

                # रोबोट कमांड उत्पन्न करें
                if nlu_result['intent'] in ['navigation', 'manipulation']:
                    cmd = self.generate_robot_command(nlu_result)
                    cmd_msg = String()
                    cmd_msg.data = json.dumps(cmd)
                    self.command_pub.publish(cmd_msg)

        except Exception as e:
            self.get_logger().error(f'त्रुटि: {e}')

    def generate_robot_command(self, nlu_result):
        intent = nlu_result['intent']
        entities = nlu_result['entities']

        if intent == 'navigation' and entities:
            return {'command': 'navigate', 'target': entities[0]['text']}
        elif intent == 'manipulation' and entities:
            return {'command': 'manipulate', 'object': entities[0]['text']}

        return None
```

<div className="border-line"></div>

<h2 className="second-heading">कार्य योजना एकीकरण</h2>
<div className="underline-class"></div>

```python
class VoiceCommandPlanner:
    def __init__(self):
        self.planning_system = self.init_planner()

    def plan_from_voice(self, nlu_result, environment):
        intent = nlu_result['intent']
        entities = nlu_result['entities']

        if intent == 'navigation':
            return self.plan_navigation(entities, environment)
        elif intent == 'manipulation':
            return self.plan_manipulation(entities, environment)

        return {'success': False, 'error': f'अज्ञात इरादा: {intent}'}

    def plan_navigation(self, entities, environment):
        location = entities[0]['text'] if entities else None
        if not location:
            return {'success': False, 'error': 'कोई स्थान निर्दिष्ट नहीं'}

        known_locations = environment.get('locations', [])
        if location not in known_locations:
            return {'success': False, 'error': f'अज्ञात स्थान: {location}'}

        return {
            'success': True,
            'intent': 'navigation',
            'target': location,
            'plan': [
                {'action': 'plan_path'},
                {'action': 'execute_navigation'},
                {'action': 'verify_arrival'}
            ]
        }

    def plan_manipulation(self, entities, environment):
        object_name = entities[0]['text'] if entities else None
        if not object_name:
            return {'success': False, 'error': 'कोई ऑब्जेक्ट निर्दिष्ट नहीं'}

        return {
            'success': True,
            'intent': 'manipulation',
            'target': object_name,
            'plan': [
                {'action': 'locate_object'},
                {'action': 'approach_object'},
                {'action': 'grasp_object'}
            ]
        }
```

<div className="border-line"></div>

<h2 className="second-heading">त्रुटि निपटान</h2>
<div className="underline-class"></div>

```python
class RobustVoiceInteractionSystem:
    def __init__(self):
        self.speech_recognizer = SpeechRecognitionSystem()
        self.nlu = NaturalLanguageUnderstandingSystem()
        self.max_attempts = 3
        self.confidence_threshold = 0.7

    def process_with_error_handling(self, audio_input):
        attempts = 0

        while attempts < self.max_attempts:
            try:
                # भाषा पहचानें
                text = self.speech_recognizer.recognize_with_whisper(audio_input)

                # आत्मविश्वास की जांच करें (सरलीकृत)
                if len(text) < 3:
                    raise ValueError("पहचान आत्मविश्वास बहुत कम")

                # NLU के साथ प्रक्रिया करें
                nlu_result = self.nlu.understand_command(text)

                # मान्यता दें
                if nlu_result['intent'] == 'unknown':
                    raise ValueError("इरादा समझ नहीं आया")

                return {'success': True, 'nlu': nlu_result, 'attempts': attempts + 1}

            except ValueError as e:
                attempts += 1
                if attempts >= self.max_attempts:
                    break
                time.sleep(1.0)

        return {'success': False, 'error': 'अधिकतम प्रयास पहुंच गए', 'attempts': attempts}

    def provide_feedback(self, error, attempt):
        if attempt < self.max_attempts:
            if "low confidence" in error.lower():
                return "क्या आप अधिक स्पष्ट रूप से बोल सकते हैं?"
            return "क्या आप यह दोहरा सकते हैं?"
        return "क्षमा करें, मुझे समझने में परेशानी हो रही है।"
```

<div className="border-line"></div>

<h2 className="second-heading">मूल्यांकन</h2>
<div className="underline-class"></div>

```python
class VoiceSystemEvaluator:
    def __init__(self):
        self.metrics = {
            'recognition_accuracy': [],
            'understanding_accuracy': [],
            'task_success_rate': []
        }

    def evaluate_system(self, voice_system, test_scenarios):
        results = []

        for scenario in test_scenarios:
            output = voice_system.process_with_error_handling(scenario['audio'])

            result = {
                'recognition_accuracy': self.eval_recognition(output, scenario),
                'understanding_accuracy': self.eval_understanding(output, scenario),
                'task_success': self.eval_task_success(output, scenario)
            }
            results.append(result)

        return {
            'recognition_accuracy': np.mean([r['recognition_accuracy'] for r in results]),
            'understanding_accuracy': np.mean([r['understanding_accuracy'] for r in results]),
            'task_success_rate': sum(1 for r in results if r['task_success']) / len(results)
        }

    def eval_recognition(self, output, scenario):
        if not output['success']:
            return 0.0

        recognized = output.get('nlu', {}).get('original_text', '')
        expected = scenario['expected_text']

        import difflib
        return difflib.SequenceMatcher(None, recognized.lower(), expected.lower()).ratio()
```

<div className="border-line"></div>

<h2 className="second-heading">सारांश</h2>
<div className="underline-class"></div>

**मुख्य घटक:**

1. **ऑडियो प्रोसेसिंग**: शोर संभाल के लिए VAD के साथ मजबूत कैप्चर
2. **भाषा पहचान**: उच्च-सटीकता टेक्स्ट रूपांतरण के लिए Whisper/Wav2Vec2
3. **NLU**: सांदर्भिक समाधान के साथ इरादा और एंटिटी निष्कर्षण
4. **संवाद प्रबंधन**: संदर्भ-जागरूक बहु-कतार इंटरैक्शन संभाल
5. **Isaac एकीकरण**: विशिष्ट वॉइस प्रोसेसिंग घटक
6. **कार्य योजना**: वॉइस कमांड से रोबोट क्रिया एकीकरण
7. **त्रुटि निपटान**: व्यापक प्रतिक्रिया तंत्र

**सफलता कारक**: प्रतिक्रिया के साथ सटीकता को संतुलित करें, प्राकृतिक बातचीत प्रवाह बनाए रखें, वास्तविक दुनिया की स्थितियों में सुरक्षा और विश्वसनीयता सुनिश्चित करें।

<div className="border-line"></div>

<h2 className="second-heading">अभ्यास</h2>
<div className="underline-class"></div>

1. Whisper का उपयोग करके भाषा पहचान लागू करें
2. डोमेन-विशिष्ट कमांड के लिए NLU सिस्टम बनाएं
3. संदर्भ-जागरूक संवाद प्रबंधक डिज़ाइन करें
4. वॉइस सिस्टम के लिए मूल्यांकन ढांचा बनाएं
5. रोबोट योजना सिस्टम के साथ एकीकृत करें

<div className="border-line"></div>

<h2 className="second-heading">आगे की पढ़ाई</h2>
<div className="underline-class"></div>

- "Spoken Language Processing" रबिनर और जुआंग द्वारा
- "Natural Language Understanding" जेम्स एलन द्वारा
- "Dialog Systems" वॉकर एट अल. द्वारा
- भाषा प्रोसेसिंग पर NVIDIA Isaac दस्तावेज़ीकरण

</div>