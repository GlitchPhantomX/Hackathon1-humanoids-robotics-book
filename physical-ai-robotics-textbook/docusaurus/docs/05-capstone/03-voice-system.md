---
sidebar_position: 3
title: "Voice Command and Interaction System"
description: "Implementing voice command processing for humanoid robots"
---

import ReadingTime from '@site/src/components/ReadingTime';

<ReadingTime minutes={30} />

<h1 className="main-heading">Voice Command and Interaction System</h1>
<div className="underline-class"></div>

<div className="full-content">

<div className="border-line"></div>

<h2 className="second-heading">Learning Objectives</h2>
<div className="underline-class"></div>

- Design comprehensive voice command processing system
- Integrate speech recognition with NLU
- Create context-aware dialogue management
- Implement voice-based task planning
- Ensure robust voice interaction in real-world environments

<div className="border-line"></div>

<h2 className="second-heading">Introduction</h2>
<div className="underline-class"></div>

Voice command systems enable natural communication between humans and humanoid robots. The system handles audio capture, speech-to-text conversion, intent understanding, dialogue management, and action execution. Using the Vision-Language-Action paradigm, it connects spoken language to robotic actions.

<div className="border-line"></div>

<h2 className="second-heading">Speech Recognition</h2>
<div className="underline-class"></div>

<h3 className="third-heading">Audio Capture</h3>

```python
import pyaudio
import numpy as np
import webrtcvad

class AudioCaptureSystem:
    def __init__(self, sample_rate=16000, chunk_size=1024):
        self.sample_rate = sample_rate
        self.chunk_size = chunk_size
        self.audio = pyaudio.PyAudio()
        self.vad = webrtcvad.Vad(1)  # Voice activity detection
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

<h3 className="third-heading">Speech-to-Text</h3>

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

<h2 className="second-heading">Natural Language Understanding</h2>
<div className="underline-class"></div>

<h3 className="third-heading">Intent Recognition</h3>

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
        
        # Extract intent
        intent_result = self.extract_intent(text)
        
        # Extract entities
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

<h3 className="third-heading">Dialogue Management</h3>

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
        # Add to history
        self.context.conversation_history.append({
            'speaker': 'user',
            'text': user_input,
            'timestamp': time.time()
        })
        
        # Understand command
        nlu_result = self.nlu.understand_command(user_input)
        
        # Validate
        if not self.validate_command(nlu_result):
            response = "I didn't understand. Could you rephrase?"
        else:
            response = self.start_new_task(nlu_result)
        
        # Add response to history
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
            return f"Okay, I'll {nlu_result['intent']} {entities[0]['text']}."
        return f"Starting {nlu_result['intent']} task."
```

<div className="border-line"></div>

<h2 className="second-heading">Isaac Integration</h2>
<div className="underline-class"></div>

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class IsaacVoiceProcessingNode(Node):
    def __init__(self):
        super().__init__('voice_processing_node')
        
        # Publishers
        self.text_pub = self.create_publisher(String, '/speech_to_text', 10)
        self.command_pub = self.create_publisher(String, '/robot_commands', 10)
        
        # Subscribers
        self.audio_sub = self.create_subscription(AudioData, '/audio_input', self.audio_cb, 10)
        
        # Initialize systems
        self.speech_recognizer = SpeechRecognitionSystem()
        self.nlu = NaturalLanguageUnderstandingSystem()
        self.dialogue = ContextAwareDialogueManager()
    
    def audio_cb(self, msg):
        try:
            # Convert audio to array
            audio_array = np.frombuffer(msg.data, dtype=np.int16).astype(np.float32) / 32768.0
            
            # Recognize speech
            text = self.speech_recognizer.recognize_with_whisper(audio_array)
            
            if text:
                # Publish recognized text
                text_msg = String()
                text_msg.data = json.dumps({'text': text})
                self.text_pub.publish(text_msg)
                
                # Process with NLU
                nlu_result = self.nlu.understand_command(text)
                
                # Generate robot command
                if nlu_result['intent'] in ['navigation', 'manipulation']:
                    cmd = self.generate_robot_command(nlu_result)
                    cmd_msg = String()
                    cmd_msg.data = json.dumps(cmd)
                    self.command_pub.publish(cmd_msg)
        
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
    
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

<h2 className="second-heading">Task Planning Integration</h2>
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
        
        return {'success': False, 'error': f'Unknown intent: {intent}'}
    
    def plan_navigation(self, entities, environment):
        location = entities[0]['text'] if entities else None
        if not location:
            return {'success': False, 'error': 'No location specified'}
        
        known_locations = environment.get('locations', [])
        if location not in known_locations:
            return {'success': False, 'error': f'Unknown location: {location}'}
        
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
            return {'success': False, 'error': 'No object specified'}
        
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

<h2 className="second-heading">Error Handling</h2>
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
                # Recognize speech
                text = self.speech_recognizer.recognize_with_whisper(audio_input)
                
                # Check confidence (simplified)
                if len(text) < 3:
                    raise ValueError("Recognition confidence too low")
                
                # Process with NLU
                nlu_result = self.nlu.understand_command(text)
                
                # Validate
                if nlu_result['intent'] == 'unknown':
                    raise ValueError("Could not understand intent")
                
                return {'success': True, 'nlu': nlu_result, 'attempts': attempts + 1}
            
            except ValueError as e:
                attempts += 1
                if attempts >= self.max_attempts:
                    break
                time.sleep(1.0)
        
        return {'success': False, 'error': 'Max attempts reached', 'attempts': attempts}
    
    def provide_feedback(self, error, attempt):
        if attempt < self.max_attempts:
            if "low confidence" in error.lower():
                return "Could you speak more clearly?"
            return "Could you repeat that?"
        return "Sorry, I'm having trouble understanding."
```

<div className="border-line"></div>

<h2 className="second-heading">Evaluation</h2>
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

<h2 className="second-heading">Summary</h2>
<div className="underline-class"></div>

**Key Components:**

1. **Audio Processing**: Robust capture with VAD for noise handling
2. **Speech Recognition**: Whisper/Wav2Vec2 for high-accuracy text conversion
3. **NLU**: Intent and entity extraction with contextual resolution
4. **Dialogue Management**: Context-aware multi-turn interaction handling
5. **Isaac Integration**: Specialized voice processing components
6. **Task Planning**: Voice command to robot action integration
7. **Error Handling**: Comprehensive feedback mechanisms

**Success Factors**: Balance accuracy with responsiveness, maintain natural conversation flow, ensure safety and reliability in real-world conditions.

<div className="border-line"></div>

<h2 className="second-heading">Exercises</h2>
<div className="underline-class"></div>

1. Implement speech recognition using Whisper
2. Create NLU system for domain-specific commands
3. Design context-aware dialogue manager
4. Build evaluation framework for voice system
5. Integrate with robot planning system

<div className="border-line"></div>

<h2 className="second-heading">Further Reading</h2>
<div className="underline-class"></div>

- "Spoken Language Processing" by Rabiner and Juang
- "Natural Language Understanding" by James Allen
- "Dialog Systems" by Walker et al.
- NVIDIA Isaac documentation on voice processing

</div>