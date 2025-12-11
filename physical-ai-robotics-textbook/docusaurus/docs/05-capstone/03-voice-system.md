---
sidebar_position: 3
title: "Voice Command and Interaction System"
description: "Implementing voice command processing and natural language interaction for humanoid robots"
---

import ReadingTime from '@site/src/components/ReadingTime';
import ViewToggle from '@site/src/components/ViewToggle';

<ReadingTime minutes={153} />

<ViewToggle />

<h1 className="main-heading">Voice Command and Interaction System</h1>
<div className="underline-class"></div>

<div className="full-content">

<div className="border-line"></div>
---

<h2 className="second-heading">
 Learning Objectives
</h2>
<div className="underline-class"></div>

After completing this chapter, you will be able to:
- • Design and implement a comprehensive voice command processing system
- • Integrate speech recognition with natural language understanding
- • Create context-aware dialogue management for human-robot interaction
- • Implement voice-based task planning and execution
- • Ensure robust voice interaction in real-world environments

<div className="border-line"></div>
---

<h2 className="second-heading">
 Introduction to Voice Systems in Humanoid Robotics
</h2>
<div className="underline-class"></div>

Voice command and interaction systems represent a crucial interface between humans and humanoid robots, enabling natural and intuitive communication. Unlike traditional command-line interfaces or button-based controls, voice systems allow users to communicate with robots using natural language, making robotic systems more accessible and user-friendly.

The voice system in a humanoid robot must handle multiple complex tasks: capturing and processing audio input, converting speech to text, understanding the meaning and intent behind the spoken words, planning appropriate responses or actions, and potentially generating spoken responses. This requires tight integration between audio processing, natural language understanding, dialogue management, and action execution systems.

This chapter explores the implementation of a comprehensive voice command and interaction system for humanoid robots, focusing on the Vision-Language-Action paradigm that connects spoken language to robotic action execution. The system must be robust enough to handle real-world acoustic conditions while maintaining the natural, conversational quality that users expect.

<div className="border-line"></div>
---

<h2 className="second-heading">
 Speech Recognition and Audio Processing
</h2>
<div className="underline-class"></div>

<h3 className="third-heading">
- Audio Capture and Preprocessing
</h3>
<div className="underline-class"></div>

The foundation of any voice system is the ability to capture high-quality audio in various environmental conditions:

```python
# Example: Audio capture and preprocessing system
import pyaudio
import numpy as np
import webrtcvad
import collections
import threading
import time
from typing import List, Dict, Any, Optional
import logging

class AudioCaptureSystem:
    def __init__(self, sample_rate: int = 16000, chunk_size: int = 1024, channels: int = 1):
        self.sample_rate = sample_rate
        self.chunk_size = chunk_size
        self.channels = channels
        self.audio_format = pyaudio.paInt16
        self.audio = pyaudio.PyAudio()

        # Voice activity detection
        self.vad = webrtcvad.Vad()
        self.vad.set_mode(1)  # Aggressiveness mode: 0-3

        # Audio buffers
        self.audio_buffer = collections.deque(maxlen=100)  # Store last 100 chunks
        self.active_speech_buffer = collections.deque(maxlen=50)  # Active speech chunks

        # Audio processing parameters
        self.noise_threshold = 0.01
        self.speech_threshold = 0.1
        self.min_speech_duration = 0.5  # seconds

        # Threading for continuous capture
        self.capture_thread = None
        self.is_capturing = False
        self.logger = logging.getLogger("AudioCaptureSystem")

    def start_capture(self):
        """Start continuous audio capture"""
        if self.is_capturing:
            return False

        # Open audio stream
        self.stream = self.audio.open(
            format=self.audio_format,
            channels=self.channels,
            rate=self.sample_rate,
            input=True,
            frames_per_buffer=self.chunk_size
        )

        self.is_capturing = True
        self.capture_thread = threading.Thread(target=self._capture_loop)
        self.capture_thread.daemon = True
        self.capture_thread.start()

        self.logger.info("Audio capture started")
        return True

    def stop_capture(self):
        """Stop audio capture"""
        if not self.is_capturing:
            return True

        self.is_capturing = False

        if hasattr(self, 'stream'):
            self.stream.stop_stream()
            self.stream.close()

        if self.capture_thread:
            self.capture_thread.join()

        self.logger.info("Audio capture stopped")
        return True

    def _capture_loop(self):
        """Main capture loop running in separate thread"""
        while self.is_capturing:
            try:
                # Read audio chunk
                data = self.stream.read(self.chunk_size, exception_on_overflow=False)
                audio_chunk = np.frombuffer(data, dtype=np.int16).astype(np.float32) / 32768.0

                # Add to buffer
                self.audio_buffer.append(audio_chunk)

                # Check for voice activity
                if self.is_speech_detected(audio_chunk):
                    self.active_speech_buffer.append(audio_chunk)

            except Exception as e:
                self.logger.error(f"Error in audio capture loop: {e}")
                time.sleep(0.01)  # Brief pause to prevent busy loop

    def is_speech_detected(self, audio_chunk: np.ndarray) -> bool:
        """Detect if speech is present in audio chunk"""
        # Calculate energy
        energy = np.sqrt(np.mean(audio_chunk ** 2))

        # Check energy threshold
        if energy < self.noise_threshold:
            return False

        # Use WebRTC VAD for more sophisticated detection
        # Convert to appropriate format for WebRTC VAD
        audio_int16 = (audio_chunk * 32767).astype(np.int16)
        audio_bytes = audio_int16.tobytes()

        try:
            # WebRTC VAD requires 10, 20, or 30ms frames
            frame_duration = len(audio_bytes) * 1000 / (self.sample_rate * 2)  # in ms
            if frame_duration in [10, 20, 30]:
                is_speech = self.vad.is_speech(audio_bytes, self.sample_rate)
                return is_speech
            else:
                # If frame size doesn't match VAD requirements, use energy-based detection
                return energy > self.speech_threshold
        except:
            # Fallback to energy-based detection
            return energy > self.speech_threshold

    def get_recent_audio(self, duration: float = 1.0) -> np.ndarray:
        """Get recent audio data for processing"""
        required_chunks = int(duration * self.sample_rate / self.chunk_size)
        chunks_to_get = min(required_chunks, len(self.audio_buffer))

        if chunks_to_get == 0:
            return np.array([])

        recent_chunks = list(self.audio_buffer)[-chunks_to_get:]
        return np.concatenate(recent_chunks)

    def get_active_speech(self) -> np.ndarray:
        """Get accumulated active speech data"""
        if not self.active_speech_buffer:
            return np.array([])

        return np.concatenate(list(self.active_speech_buffer))

    def clear_active_speech(self):
        """Clear the active speech buffer"""
        self.active_speech_buffer.clear()

class AudioPreprocessingSystem:
    """Advanced audio preprocessing for speech recognition"""
    def __init__(self):
        self.sample_rate = 16000
        self.frame_length = 25  # ms
        self.frame_step = 10    # ms
        self.fft_length = 512
        self.preemphasis_coefficient = 0.97

        # Noise reduction parameters
        self.noise_reduction_factor = 0.1
        self.spectral_subtraction = True

    def preprocess_audio(self, audio_data: np.ndarray) -> np.ndarray:
        """Apply preprocessing steps to audio data"""
        # Normalize audio
        audio_data = self.normalize_audio(audio_data)

        # Apply pre-emphasis filter
        audio_data = self.preemphasis_filter(audio_data)

        # Apply noise reduction
        audio_data = self.reduce_noise(audio_data)

        # Apply voice activity detection
        audio_data = self.apply_vad(audio_data)

        return audio_data

    def normalize_audio(self, audio_data: np.ndarray) -> np.ndarray:
        """Normalize audio to consistent amplitude range"""
        if len(audio_data) == 0:
            return audio_data

        # Calculate RMS and normalize
        rms = np.sqrt(np.mean(audio_data ** 2))
        if rms > 0:
            target_rms = 0.1  # Target RMS value
            gain = target_rms / rms
            audio_data = audio_data * gain

        return np.clip(audio_data, -1.0, 1.0)

    def preemphasis_filter(self, audio_data: np.ndarray) -> np.ndarray:
        """Apply pre-emphasis filter to enhance high frequencies"""
        return np.append(
            audio_data[0],
            audio_data[1:] - self.preemphasis_coefficient * audio_data[:-1]
        )

    def reduce_noise(self, audio_data: np.ndarray) -> np.ndarray:
        """Apply basic noise reduction"""
        if not self.spectral_subtraction or len(audio_data) < self.fft_length:
            return audio_data

        # Simple spectral subtraction approach
        window = np.hanning(self.fft_length)
        frames = self.audio_to_frames(audio_data, self.fft_length, self.fft_length // 2)

        enhanced_frames = []
        for frame in frames:
            # Apply window
            windowed_frame = frame * window

            # FFT
            fft_frame = np.fft.fft(windowed_frame, self.fft_length)

            # Calculate magnitude spectrum
            magnitude = np.abs(fft_frame)

            # Estimate noise (simple approach: use minimum values over time)
            # In practice, you'd estimate noise during silence periods
            noise_floor = np.mean(magnitude) * self.noise_reduction_factor

            # Apply spectral subtraction
            enhanced_magnitude = np.maximum(magnitude - noise_floor, 0.1 * magnitude)

            # Combine with phase
            phase = np.angle(fft_frame)
            enhanced_fft = enhanced_magnitude * np.exp(1j * phase)

            # IFFT
            enhanced_frame = np.real(np.fft.ifft(enhanced_fft, self.fft_length))

            # Apply window again for overlap-add
            enhanced_frame = enhanced_frame * window
            enhanced_frames.append(enhanced_frame)

        # Overlap-add reconstruction
        result_length = len(audio_data)
        result = np.zeros(result_length)
        frame_step = self.fft_length // 2
        for i, frame in enumerate(enhanced_frames):
            start_idx = i * frame_step
            end_idx = min(start_idx + len(frame), result_length)
            frame_end = min(len(frame), result_length - start_idx)
            result[start_idx:end_idx] += frame[:frame_end]

        return result[:len(audio_data)]

    def audio_to_frames(self, audio_data: np.ndarray, frame_length: int, frame_step: int) -> List[np.ndarray]:
        """Convert audio data to overlapping frames"""
        if len(audio_data) < frame_length:
            return [audio_data]

        frames = []
        for i in range(0, len(audio_data) - frame_length + 1, frame_step):
            frames.append(audio_data[i:i + frame_length])

        return frames

    def apply_vad(self, audio_data: np.ndarray) -> np.ndarray:
        """Apply voice activity detection to focus on speech segments"""
        # This would implement more sophisticated VAD
        # For this example, we'll return the original data
        return audio_data
```

### Speech-to-Text Integration

The speech recognition component converts audio to text for further processing:

```python
# Example: Speech recognition system integration
import speech_recognition as sr
import whisper
import torch
from transformers import pipeline
import asyncio
import threading
from queue import Queue

class SpeechRecognitionSystem:
    def __init__(self, model_type: str = "whisper"):
        self.model_type = model_type
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

        # Adjust for ambient noise
        self.recognizer.energy_threshold = 400
        self.recognizer.dynamic_energy_threshold = True
        self.recognizer.pause_threshold = 0.8  # seconds of silence before phrase is considered complete

        # Initialize models based on type
        if model_type == "whisper":
            self.whisper_model = whisper.load_model("base")
        elif model_type == "transformers":
            self.asr_pipeline = pipeline(
                "automatic-speech-recognition",
                model="facebook/wav2vec2-large-960h",
                device=0 if torch.cuda.is_available() else -1
            )

        # Audio processing system
        self.audio_capture = AudioCaptureSystem()
        self.audio_preprocessor = AudioPreprocessingSystem()

        # Recognition queue for async processing
        self.recognition_queue = Queue()
        self.result_callback = None

    def initialize_microphone(self):
        """Initialize microphone with proper settings"""
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source, duration=1.0)
            self.logger.info(f"Adjusted for ambient noise, energy threshold: {self.recognizer.energy_threshold}")

    def listen_for_speech(self, timeout: float = 5.0, phrase_time_limit: float = 10.0) -> str:
        """Listen for speech and return recognized text"""
        try:
            with self.microphone as source:
                self.logger.info("Listening for speech...")
                audio = self.recognizer.listen(
                    source,
                    timeout=timeout,
                    phrase_time_limit=phrase_time_limit
                )

            # Convert to text using selected model
            if self.model_type == "whisper":
                return self.recognize_with_whisper(audio)
            elif self.model_type == "transformers":
                return self.recognize_with_transformers(audio)
            else:
                return self.recognize_with_google(audio)

        except sr.WaitTimeoutError:
            self.logger.warning("No speech detected within timeout")
            return ""
        except sr.UnknownValueError:
            self.logger.warning("Could not understand audio")
            return ""
        except sr.RequestError as e:
            self.logger.error(f"Recognition service error: {e}")
            return ""

    def recognize_with_whisper(self, audio) -> str:
        """Recognize speech using Whisper model"""
        try:
            # Convert audio to raw data
            audio_data = audio.get_raw_data()
            audio_array = np.frombuffer(audio_data, dtype=np.int16).astype(np.float32)

            # Normalize audio
            audio_array = audio_array / 32768.0

            # Process with Whisper
            result = self.whisper_model.transcribe(audio_array)
            return result["text"].strip()
        except Exception as e:
            self.logger.error(f"Whisper recognition error: {e}")
            return ""

    def recognize_with_transformers(self, audio) -> str:
        """Recognize speech using Transformers pipeline"""
        try:
            # Convert audio to raw data
            audio_data = audio.get_raw_data()
            audio_array = np.frombuffer(audio_data, dtype=np.int16).astype(np.float32)

            # Normalize audio
            audio_array = audio_array / 32768.0

            # Process with pipeline
            result = self.asr_pipeline(audio_array)
            return result["text"].strip()
        except Exception as e:
            self.logger.error(f"Transformers recognition error: {e}")
            return ""

    def recognize_with_google(self, audio) -> str:
        """Recognize speech using Google Speech Recognition"""
        try:
            return self.recognizer.recognize_google(audio).strip()
        except Exception as e:
            self.logger.error(f"Google recognition error: {e}")
            return ""

    def continuous_listening(self, callback_func=None):
        """Start continuous listening for voice commands"""
        def callback(recognizer, audio):
            try:
                text = self.recognize_with_google(audio)
                if text and callback_func:
                    callback_func(text)
            except sr.UnknownValueError:
                pass  # Ignore unrecognized audio
            except sr.RequestError as e:
                self.logger.error(f"Recognition service error: {e}")

        # Start listening in background
        self.stop_listening = self.recognizer.listen_in_background(
            self.microphone, callback
        )
        self.logger.info("Started continuous listening")

    def stop_continuous_listening(self):
        """Stop continuous listening"""
        if hasattr(self, 'stop_listening'):
            self.stop_listening()
            self.logger.info("Stopped continuous listening")

class AdvancedSpeechRecognitionSystem:
    """Advanced speech recognition with context and error handling"""
    def __init__(self):
        self.speech_recognizer = SpeechRecognitionSystem()
        self.language_models = {}
        self.context_history = []
        self.error_correction_enabled = True

        # Initialize language models for error correction
        self.initialize_language_models()

    def initialize_language_models(self):
        """Initialize language models for error correction and context"""
        try:
            self.language_models['correction'] = pipeline(
                "fill-mask",
                model="bert-base-uncased",
                device=0 if torch.cuda.is_available() else -1
            )
        except:
            self.logger.warning("Could not initialize language model, error correction disabled")
            self.error_correction_enabled = False

    def recognize_with_context(self, audio, context: Dict[str, Any] = None) -> Dict[str, Any]:
        """Recognize speech with contextual information"""
        # Perform initial recognition
        raw_text = self.speech_recognizer.recognize_with_google(audio)

        # Apply error correction based on context
        if self.error_correction_enabled and context:
            corrected_text = self.apply_contextual_correction(raw_text, context)
        else:
            corrected_text = raw_text

        # Add to context history
        self.context_history.append({
            'raw_text': raw_text,
            'corrected_text': corrected_text,
            'timestamp': time.time(),
            'context': context or {}
        })

        # Keep only recent history
        if len(self.context_history) > 50:
            self.context_history = self.context_history[-50:]

        return {
            'raw_text': raw_text,
            'corrected_text': corrected_text,
            'confidence': self.estimate_confidence(corrected_text, raw_text),
            'timestamp': time.time()
        }

    def apply_contextual_correction(self, text: str, context: Dict[str, Any]) -> str:
        """Apply contextual correction to recognized text"""
        if not text or not context:
            return text

        # Example: Replace common recognition errors based on context
        corrections = context.get('expected_words', [])
        words = text.split()

        corrected_words = []
        for word in words:
            # Check if word is likely a recognition error based on context
            corrected_word = self.correct_word_in_context(word, corrections)
            corrected_words.append(corrected_word)

        return ' '.join(corrected_words)

    def correct_word_in_context(self, word: str, expected_words: List[str]) -> str:
        """Correct a word based on expected words in context"""
        if word.lower() in [w.lower() for w in expected_words]:
            return word  # Already correct

        # Simple similarity-based correction
        import difflib
        best_match = difflib.get_close_matches(
            word.lower(),
            [w.lower() for w in expected_words],
            n=1,
            cutoff=0.6
        )

        if best_match:
            # Return the original case from expected_words
            for expected_word in expected_words:
                if expected_word.lower() == best_match[0]:
                    return expected_word

        return word

    def estimate_confidence(self, corrected_text: str, raw_text: str) -> float:
        """Estimate confidence in the recognition result"""
        if not corrected_text:
            return 0.0

        # Simple confidence estimation based on text similarity
        import difflib
        similarity = difflib.SequenceMatcher(None, corrected_text.lower(), raw_text.lower()).ratio()

        # Additional factors could include:
        # - Length of recognized text
        # - Presence of common command words
        # - Contextual appropriateness

        base_confidence = 0.7  # Base confidence
        similarity_factor = 0.3 * similarity  # Up to 30% from similarity

        return min(1.0, base_confidence + similarity_factor)
```

## Natural Language Understanding

### Intent Recognition and Entity Extraction

The natural language understanding component interprets the meaning behind recognized text:

```python
# Example: Natural language understanding system
import spacy
import nltk
from transformers import pipeline, AutoTokenizer, AutoModelForTokenClassification
from typing import List, Dict, Any, Tuple
import re

class NaturalLanguageUnderstandingSystem:
    def __init__(self):
        # Load NLP models
        try:
            self.nlp = spacy.load("en_core_web_sm")
        except OSError:
            self.logger.error("Please install spaCy English model: python -m spacy download en_core_web_sm")
            self.nlp = None

        # Initialize intent classification pipeline
        self.intent_classifier = pipeline(
            "text-classification",
            model="microsoft/DialoGPT-medium",
            return_all_scores=True
        )

        # Initialize named entity recognition
        self.ner_pipeline = pipeline(
            "ner",
            model="dbmdz/bert-large-cased-finetuned-conll03-english",
            aggregation_strategy="simple"
        )

        # Define command patterns and intents
        self.intent_patterns = {
            'navigation': [
                r'go to (.+)',
                r'move to (.+)',
                r'go (.+)',
                r'walk to (.+)',
                r'navigate to (.+)',
                r'take me to (.+)',
                r'bring me to (.+)'
            ],
            'manipulation': [
                r'pick up (.+)',
                r'grab (.+)',
                r'take (.+)',
                r'get (.+)',
                r'fetch (.+)',
                r'bring me (.+)',
                r'give me (.+)'
            ],
            'information': [
                r'what is (.+)',
                r'where is (.+)',
                r'how many (.+)',
                r'tell me about (.+)'
            ],
            'action': [
                r'open (.+)',
                r'close (.+)',
                r'turn (.+)',
                r'switch (.+)',
                r'start (.+)',
                r'stop (.+)'
            ]
        }

        # Define entity types and patterns
        self.entity_patterns = {
            'location': ['kitchen', 'bedroom', 'living room', 'office', 'bathroom', 'hallway', 'dining room'],
            'object': ['cup', 'bottle', 'book', 'phone', 'keys', 'glasses', 'apple', 'water'],
            'action': ['pick up', 'grab', 'take', 'go to', 'navigate', 'open', 'close'],
            'color': ['red', 'blue', 'green', 'yellow', 'orange', 'purple', 'pink', 'brown', 'black', 'white', 'gray']
        }

    def understand_command(self, text: str, context: Dict[str, Any] = None) -> Dict[str, Any]:
        """Understand a command in natural language"""
        if not text:
            return {'intent': 'unknown', 'entities': [], 'confidence': 0.0}

        # Process with spaCy if available
        doc = self.nlp(text) if self.nlp else None

        # Extract intent
        intent_result = self.extract_intent(text)
        intent = intent_result['label']
        intent_confidence = intent_result['confidence']

        # Extract entities
        entities = self.extract_entities(text, doc)

        # Resolve references in context
        if context:
            entities = self.resolve_contextual_references(entities, context)

        # Generate structured result
        result = {
            'original_text': text,
            'intent': intent,
            'intent_confidence': intent_confidence,
            'entities': entities,
            'tokens': [token.text for token in doc] if doc else text.split(),
            'pos_tags': [(token.text, token.pos_) for token in doc] if doc else [],
            'dependency_tree': [(token.text, token.dep_, token.head.text) for token in doc] if doc else [],
            'timestamp': time.time()
        }

        return result

    def extract_intent(self, text: str) -> Dict[str, Any]:
        """Extract intent from text using pattern matching and classification"""
        text_lower = text.lower()

        # Pattern-based intent recognition
        for intent, patterns in self.intent_patterns.items():
            for pattern in patterns:
                if re.search(pattern, text_lower):
                    return {
                        'label': intent,
                        'confidence': 0.9  # High confidence for pattern match
                    }

        # If no pattern matches, use classifier
        try:
            classification_results = self.intent_classifier(text)
            # Take the highest scoring result
            top_result = max(classification_results[0], key=lambda x: x['score'])
            return {
                'label': top_result['label'],
                'confidence': top_result['score']
            }
        except Exception as e:
            self.logger.error(f"Intent classification error: {e}")
            return {
                'label': 'unknown',
                'confidence': 0.1
            }

    def extract_entities(self, text: str, doc=None) -> List[Dict[str, Any]]:
        """Extract named entities from text"""
        entities = []

        if doc:
            # Use spaCy NER
            for ent in doc.ents:
                entities.append({
                    'text': ent.text,
                    'label': ent.label_,
                    'start': ent.start_char,
                    'end': ent.end_char,
                    'confidence': 1.0  # spaCy provides high confidence for NER
                })

        # Fallback: pattern-based entity extraction
        text_lower = text.lower()
        for entity_type, patterns in self.entity_patterns.items():
            for pattern in patterns:
                if pattern in text_lower:
                    entities.append({
                        'text': pattern,
                        'label': entity_type.upper(),
                        'start': text_lower.find(pattern),
                        'end': text_lower.find(pattern) + len(pattern),
                        'confidence': 0.7
                    })

        # Remove duplicates and sort by position
        unique_entities = []
        seen = set()
        for entity in sorted(entities, key=lambda x: x['start']):
            entity_key = (entity['text'], entity['start'])
            if entity_key not in seen:
                unique_entities.append(entity)
                seen.add(entity_key)

        return unique_entities

    def resolve_contextual_references(self, entities: List[Dict[str, Any]],
                                   context: Dict[str, Any]) -> List[Dict[str, Any]]:
        """Resolve contextual references like 'it', 'there', 'this'"""
        if not context:
            return entities

        resolved_entities = []
        for entity in entities:
            # Check if entity refers to something in context
            if entity['text'].lower() in ['it', 'this', 'that']:
                # Resolve to most recently mentioned object
                if 'last_mentioned_object' in context:
                    resolved_entity = entity.copy()
                    resolved_entity['resolved_text'] = context['last_mentioned_object']
                    resolved_entity['original_text'] = entity['text']
                    resolved_entities.append(resolved_entity)
                else:
                    resolved_entities.append(entity)
            elif entity['text'].lower() in ['there', 'here']:
                # Resolve to current location or target location
                if 'current_location' in context:
                    resolved_entity = entity.copy()
                    resolved_entity['resolved_text'] = context['current_location']
                    resolved_entity['original_text'] = entity['text']
                    resolved_entities.append(resolved_entity)
                else:
                    resolved_entities.append(entity)
            else:
                resolved_entities.append(entity)

        return resolved_entities

    def validate_command(self, nlu_result: Dict[str, Any]) -> Dict[str, Any]:
        """Validate that the understood command is executable"""
        validation_result = {
            'is_valid': True,
            'errors': [],
            'suggestions': []
        }

        intent = nlu_result['intent']
        entities = nlu_result['entities']

        # Check if intent has required entities
        required_entities = self.get_required_entities_for_intent(intent)
        missing_entities = []

        for required_type in required_entities:
            if not any(ent['label'].lower() == required_type.lower() for ent in entities):
                missing_entities.append(required_type)

        if missing_entities:
            validation_result['is_valid'] = False
            validation_result['errors'].append(f"Missing required entities: {missing_entities}")

            # Provide suggestions for missing entities
            for missing in missing_entities:
                suggestions = self.get_entity_suggestions(missing)
                if suggestions:
                    validation_result['suggestions'].append({
                        'entity_type': missing,
                        'suggestions': suggestions
                    })

        # Check intent confidence
        if nlu_result['intent_confidence'] < 0.5:
            validation_result['is_valid'] = False
            validation_result['errors'].append("Low confidence in intent recognition")

        return validation_result

    def get_required_entities_for_intent(self, intent: str) -> List[str]:
        """Get required entity types for a given intent"""
        requirements = {
            'navigation': ['location'],
            'manipulation': ['object'],
            'information': ['object', 'location'],  # One of these is typically required
            'action': ['object']
        }
        return requirements.get(intent, [])

    def get_entity_suggestions(self, entity_type: str) -> List[str]:
        """Get suggestions for missing entity types"""
        suggestions = {
            'location': ['kitchen', 'bedroom', 'living room', 'office'],
            'object': ['cup', 'bottle', 'book', 'phone'],
            'action': ['pick up', 'go to', 'open', 'close']
        }
        return suggestions.get(entity_type, [])
```

### Context-Aware Dialogue Management

Advanced dialogue management maintains conversation context and handles multi-turn interactions:

```python
# Example: Context-aware dialogue manager
from enum import Enum
from dataclasses import dataclass
from typing import Optional, List, Dict, Any

class DialogueState(Enum):
    IDLE = "idle"
    LISTENING = "listening"
    PROCESSING = "processing"
    CONFIRMING = "confirming"
    EXECUTING = "executing"
    COMPLETING = "completing"
    ERROR = "error"

@dataclass
class ConversationContext:
    """Data structure for conversation context"""
    current_task: Optional[Dict[str, Any]] = None
    task_history: List[Dict[str, Any]] = None
    user_preferences: Dict[str, Any] = None
    environment_state: Dict[str, Any] = None
    robot_state: Dict[str, Any] = None
    conversation_history: List[Dict[str, str]] = None
    current_intent: Optional[str] = None
    pending_confirmation: Optional[Dict[str, Any]] = None

class ContextAwareDialogueManager:
    def __init__(self):
        self.current_state = DialogueState.IDLE
        self.context = ConversationContext(
            task_history=[],
            user_preferences={},
            environment_state={},
            robot_state={},
            conversation_history=[]
        )
        self.nlu_system = NaturalLanguageUnderstandingSystem()
        self.response_generator = self.initialize_response_generator()

    def initialize_response_generator(self):
        """Initialize response generation system"""
        class MockResponseGenerator:
            def generate_response(self, intent: str, entities: List[Dict], context: ConversationContext):
                responses = {
                    'navigation': [
                        f"I can help you go to {entities[0]['text'] if entities else 'the specified location'}. Is that correct?",
                        f"Okay, I'll navigate to {entities[0]['text'] if entities else 'the location'}."
                    ],
                    'manipulation': [
                        f"I can help you get {entities[0]['text'] if entities else 'the object'}.",
                        f"Okay, I'll fetch {entities[0]['text'] if entities else 'the item'} for you."
                    ],
                    'unknown': [
                        "I'm not sure I understood that correctly. Could you please rephrase?",
                        "I didn't catch that. Could you repeat your request?"
                    ]
                }
                return responses.get(intent, responses['unknown'])[0]

        return MockResponseGenerator()

    def process_user_input(self, user_input: str) -> Dict[str, Any]:
        """Process user input and generate appropriate response"""
        # Add to conversation history
        self.context.conversation_history.append({
            'speaker': 'user',
            'text': user_input,
            'timestamp': time.time()
        })

        # Update state
        self.current_state = DialogueState.PROCESSING

        # Understand the command
        nlu_result = self.nlu_system.understand_command(user_input, self.get_context_for_nlu())

        # Validate the command
        validation_result = self.nlu_system.validate_command(nlu_result)

        if not validation_result['is_valid']:
            response = self.handle_invalid_command(validation_result, nlu_result)
            return self.generate_response(response, 'clarification')

        # Determine appropriate action based on intent
        intent = nlu_result['intent']
        entities = nlu_result['entities']

        if self.is_continuation_command(user_input):
            response = self.continue_current_task(nlu_result)
        else:
            response = self.start_new_task(nlu_result)

        # Add to conversation history
        self.context.conversation_history.append({
            'speaker': 'robot',
            'text': response,
            'timestamp': time.time(),
            'intent': intent
        })

        return self.generate_response(response, intent)

    def get_context_for_nlu(self) -> Dict[str, Any]:
        """Extract relevant context for NLU processing"""
        return {
            'current_location': self.context.robot_state.get('location', 'unknown'),
            'available_objects': self.context.environment_state.get('objects', []),
            'last_mentioned_object': self.get_last_mentioned_object(),
            'user_preferences': self.context.user_preferences
        }

    def get_last_mentioned_object(self) -> str:
        """Get the last object mentioned in the conversation"""
        for item in reversed(self.context.conversation_history):
            if item['speaker'] == 'user':
                # Simple extraction - in practice, this would use NLU
                for entity in self.nlu_system.extract_entities(item['text']):
                    if entity['label'] in ['OBJECT', 'PRODUCT']:
                        return entity['text']
        return ''

    def is_continuation_command(self, user_input: str) -> bool:
        """Determine if user input continues the current task"""
        continuation_indicators = [
            'more', 'continue', 'keep', 'then', 'next', 'after', 'also', 'too'
        ]
        return any(indicator in user_input.lower() for indicator in continuation_indicators)

    def continue_current_task(self, nlu_result: Dict[str, Any]) -> str:
        """Continue execution of current task based on new input"""
        if not self.context.current_task:
            return "I'm not currently working on a task. What would you like me to do?"

        # Update current task with new information
        updated_task = self.update_task_with_new_info(self.context.current_task, nlu_result)
        self.context.current_task = updated_task

        return f"I'll continue with the task: {updated_task.get('description', 'current task')}"

    def start_new_task(self, nlu_result: Dict[str, Any]) -> str:
        """Start a new task based on NLU result"""
        intent = nlu_result['intent']
        entities = nlu_result['entities']

        # Create task structure
        task = {
            'id': f"task_{len(self.context.task_history) + 1}",
            'intent': intent,
            'entities': entities,
            'status': 'pending',
            'created_at': time.time(),
            'description': self.generate_task_description(nlu_result)
        }

        # Store as current task
        self.context.current_task = task

        # Generate response based on intent
        response = self.response_generator.generate_response(intent, entities, self.context)

        # Add to history
        self.context.task_history.append(task)

        return response

    def update_task_with_new_info(self, current_task: Dict[str, Any],
                                 nlu_result: Dict[str, Any]) -> Dict[str, Any]:
        """Update existing task with new information from NLU result"""
        updated_task = current_task.copy()

        # Merge new entities
        existing_entities = {ent['text']: ent for ent in updated_task.get('entities', [])}
        new_entities = {ent['text']: ent for ent in nlu_result['entities']}

        # Combine entities
        all_entities = {**existing_entities, **new_entities}
        updated_task['entities'] = list(all_entities.values())

        # Update description
        updated_task['description'] = self.generate_task_description(nlu_result)

        return updated_task

    def generate_task_description(self, nlu_result: Dict[str, Any]) -> str:
        """Generate human-readable task description"""
        intent = nlu_result['intent']
        entities = nlu_result['entities']

        if entities:
            entity_names = [ent['text'] for ent in entities]
            return f"{intent} {', '.join(entity_names)}"
        else:
            return f"{intent} command"

    def handle_invalid_command(self, validation_result: Dict[str, Any],
                             nlu_result: Dict[str, Any]) -> str:
        """Handle invalid command by requesting clarification"""
        errors = validation_result['errors']
        suggestions = validation_result['suggestions']

        if suggestions:
            suggestion_text = []
            for suggestion in suggestions:
                entity_type = suggestion['entity_type']
                options = suggestion['suggestions']
                suggestion_text.append(f"Did you mean {entity_type} like: {', '.join(options)}?")

            return f"I need more information. {'. '.join(suggestion_text)} Please clarify."

        return f"I couldn't understand your request: {'; '.join(errors)}. Could you rephrase?"

    def generate_response(self, text: str, intent: str) -> Dict[str, Any]:
        """Generate structured response"""
        return {
            'text': text,
            'intent': intent,
            'state': self.current_state.value,
            'timestamp': time.time(),
            'context': {
                'current_task_id': self.context.current_task['id'] if self.context.current_task else None,
                'task_count': len(self.context.task_history)
            }
        }

    def update_environment_state(self, new_state: Dict[str, Any]):
        """Update environment state in context"""
        self.context.environment_state.update(new_state)

    def update_robot_state(self, new_state: Dict[str, Any]):
        """Update robot state in context"""
        self.context.robot_state.update(new_state)

    def get_conversation_context(self) -> ConversationContext:
        """Get current conversation context"""
        return self.context
```

## Isaac Integration for Voice Systems

### Isaac Voice Processing Components

The Isaac ecosystem provides specialized components for voice processing and natural language understanding:

```python
# Example: Isaac integration for voice processing
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import AudioData
from geometry_msgs.msg import PoseStamped
from dialogflow_ros_msgs.msg import DialogflowQuery, DialogflowResponse
import json

class IsaacVoiceProcessingNode(Node):
    def __init__(self):
        super().__init__('voice_processing_node')

        # Publishers
        self.speech_to_text_pub = self.create_publisher(
            String,
            '/speech_to_text',
            10
        )

        self.nlu_result_pub = self.create_publisher(
            String,  # In practice, this would be a custom NLU result message
            '/nlu_results',
            10
        )

        self.dialogue_response_pub = self.create_publisher(
            String,
            '/dialogue_responses',
            10
        )

        self.robot_command_pub = self.create_publisher(
            String,  # Custom robot command message
            '/robot_commands',
            10
        )

        # Subscribers
        self.audio_sub = self.create_subscription(
            AudioData,
            '/audio_input',
            self.audio_callback,
            10
        )

        self.text_command_sub = self.create_subscription(
            String,
            '/text_commands',
            self.text_command_callback,
            10
        )

        # Initialize Isaac voice components
        self.speech_recognizer = self.initialize_speech_recognition()
        self.nlu_system = self.initialize_nlu_system()
        self.dialogue_manager = self.initialize_dialogue_manager()

        # Store conversation context
        self.conversation_context = {}

        self.get_logger().info('Isaac Voice Processing node initialized')

    def initialize_speech_recognition(self):
        """Initialize Isaac speech recognition components"""
        # This would interface with Isaac's speech recognition capabilities
        # For this example, we'll use the system we defined earlier
        return AdvancedSpeechRecognitionSystem()

    def initialize_nlu_system(self):
        """Initialize Isaac NLU components"""
        # This would interface with Isaac's NLU capabilities
        # For this example, we'll use the system we defined earlier
        return NaturalLanguageUnderstandingSystem()

    def initialize_dialogue_manager(self):
        """Initialize Isaac dialogue management"""
        # This would interface with Isaac's dialogue management
        # For this example, we'll use the system we defined earlier
        return ContextAwareDialogueManager()

    def audio_callback(self, msg: AudioData):
        """Process incoming audio data"""
        try:
            # Convert audio data to format for processing
            audio_array = np.frombuffer(msg.data, dtype=np.int16).astype(np.float32) / 32768.0

            # Create audio object for speech recognition
            # In practice, this would use Isaac's audio handling
            class MockAudio:
                def __init__(self, data):
                    self.raw_data = (data * 32768).astype(np.int16).tobytes()
                    self.sample_rate = 16000  # Standard rate

            audio_obj = MockAudio(audio_array)

            # Perform speech recognition
            recognition_result = self.speech_recognizer.recognize_with_context(
                audio_obj,
                context=self.conversation_context
            )

            if recognition_result['corrected_text']:
                # Publish recognized text
                text_msg = String()
                text_msg.data = json.dumps(recognition_result)
                self.speech_to_text_pub.publish(text_msg)

                # Process with NLU
                self.process_recognized_text(recognition_result['corrected_text'])

        except Exception as e:
            self.get_logger().error(f'Error processing audio: {e}')

    def text_command_callback(self, msg: String):
        """Process incoming text commands"""
        try:
            # Handle text commands directly
            text = msg.data
            self.process_recognized_text(text)
        except Exception as e:
            self.get_logger().error(f'Error processing text command: {e}')

    def process_recognized_text(self, text: str):
        """Process recognized text through NLU and dialogue systems"""
        try:
            # Process with NLU system
            nlu_result = self.nlu_system.understand_command(
                text,
                context=self.conversation_context
            )

            # Publish NLU results
            nlu_msg = String()
            nlu_msg.data = json.dumps(nlu_result)
            self.nlu_result_pub.publish(nlu_msg)

            # Process with dialogue manager
            dialogue_response = self.dialogue_manager.process_user_input(text)

            # Publish dialogue response
            response_msg = String()
            response_msg.data = json.dumps(dialogue_response)
            self.dialogue_response_pub.publish(response_msg)

            # If the command requires robot action, generate command
            if self.should_generate_robot_command(nlu_result):
                robot_command = self.generate_robot_command(nlu_result)
                if robot_command:
                    cmd_msg = String()
                    cmd_msg.data = json.dumps(robot_command)
                    self.robot_command_pub.publish(cmd_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing recognized text: {e}')

    def should_generate_robot_command(self, nlu_result: Dict[str, Any]) -> bool:
        """Determine if NLU result should generate a robot command"""
        actionable_intents = ['navigation', 'manipulation', 'action']
        return nlu_result.get('intent') in actionable_intents

    def generate_robot_command(self, nlu_result: Dict[str, Any]) -> Dict[str, Any]:
        """Generate robot command from NLU result"""
        intent = nlu_result['intent']
        entities = nlu_result['entities']

        if intent == 'navigation':
            # Find location entity
            location_entity = next((ent for ent in entities if ent['label'] in ['LOCATION', 'FAC', 'GPE']), None)
            if location_entity:
                return {
                    'command': 'navigate',
                    'target_location': location_entity['text'],
                    'nlu_result': nlu_result
                }

        elif intent == 'manipulation':
            # Find object entity
            object_entity = next((ent for ent in entities if ent['label'] in ['OBJECT', 'PRODUCT']), None)
            if object_entity:
                return {
                    'command': 'manipulate',
                    'target_object': object_entity['text'],
                    'nlu_result': nlu_result
                }

        elif intent == 'action':
            # Handle general actions
            return {
                'command': 'execute_action',
                'action_type': 'general',
                'nlu_result': nlu_result
            }

        return None
```

### Voice Command Planning Integration

Integrating voice commands with the robot's planning system:

```python
# Example: Voice command to action planning
class VoiceCommandPlanner:
    def __init__(self):
        self.planning_system = self.initialize_planning_system()
        self.task_decomposer = self.initialize_task_decomposer()

    def initialize_planning_system(self):
        """Initialize the robot planning system"""
        class MockPlanningSystem:
            def plan_navigation(self, target_location):
                return {
                    'success': True,
                    'plan': [
                        {'action': 'move_to_waypoint', 'params': {'x': 1.0, 'y': 1.0}},
                        {'action': 'move_to_waypoint', 'params': {'x': 2.0, 'y': 2.0}},
                        {'action': 'reach_target', 'params': {'location': target_location}}
                    ]
                }

            def plan_manipulation(self, target_object):
                return {
                    'success': True,
                    'plan': [
                        {'action': 'navigate_to_object', 'params': {'object': target_object}},
                        {'action': 'approach_object', 'params': {'object': target_object}},
                        {'action': 'grasp_object', 'params': {'object': target_object}},
                        {'action': 'lift_object', 'params': {'object': target_object}}
                    ]
                }

        return MockPlanningSystem()

    def initialize_task_decomposer(self):
        """Initialize task decomposition system"""
        class MockTaskDecomposer:
            def decompose_navigation_task(self, location, context):
                return [
                    {'action': 'check_map', 'description': 'Verify location exists in map'},
                    {'action': 'plan_path', 'description': 'Calculate navigation path'},
                    {'action': 'execute_navigation', 'description': 'Move to location'},
                    {'action': 'verify_arrival', 'description': 'Confirm arrival at destination'}
                ]

            def decompose_manipulation_task(self, object_name, context):
                return [
                    {'action': 'locate_object', 'description': f'Find {object_name} in environment'},
                    {'action': 'plan_approach', 'description': f'Plan approach to {object_name}'},
                    {'action': 'execute_grasp', 'description': f'Grasp {object_name}'},
                    {'action': 'verify_grasp', 'description': f'Confirm successful grasp of {object_name}'}
                ]

        return MockTaskDecomposer()

    def plan_from_voice_command(self, nlu_result: Dict[str, Any],
                               environment_context: Dict[str, Any]) -> Dict[str, Any]:
        """Generate action plan from voice command NLU result"""
        intent = nlu_result['intent']
        entities = nlu_result['entities']

        if intent == 'navigation':
            return self.plan_navigation_task(nlu_result, environment_context)
        elif intent == 'manipulation':
            return self.plan_manipulation_task(nlu_result, environment_context)
        elif intent == 'action':
            return self.plan_action_task(nlu_result, environment_context)
        else:
            return {
                'success': False,
                'error': f'Unknown intent: {intent}',
                'plan': []
            }

    def plan_navigation_task(self, nlu_result: Dict[str, Any],
                           environment_context: Dict[str, Any]) -> Dict[str, Any]:
        """Plan navigation task from NLU result"""
        entities = nlu_result['entities']

        # Find location entity
        location_entity = next((ent for ent in entities if ent['label'] in ['LOCATION', 'FAC', 'GPE']), None)

        if not location_entity:
            return {
                'success': False,
                'error': 'No location specified in navigation command',
                'plan': []
            }

        target_location = location_entity['text']

        # Check if location is known in environment
        known_locations = environment_context.get('map', {}).get('locations', [])
        if target_location not in known_locations:
            # Try to resolve to known location
            resolved_location = self.resolve_location(target_location, known_locations)
            if not resolved_location:
                return {
                    'success': False,
                    'error': f'Unknown location: {target_location}',
                    'suggestions': known_locations[:5],  # Provide suggestions
                    'plan': []
                }
            target_location = resolved_location

        # Generate navigation plan
        plan_result = self.planning_system.plan_navigation(target_location)

        if plan_result['success']:
            # Add task decomposition
            task_decomposition = self.task_decomposer.decompose_navigation_task(
                target_location, environment_context
            )

            return {
                'success': True,
                'intent': 'navigation',
                'target': target_location,
                'plan': plan_result['plan'],
                'task_decomposition': task_decomposition,
                'original_nlu': nlu_result
            }
        else:
            return {
                'success': False,
                'error': 'Failed to generate navigation plan',
                'plan': []
            }

    def plan_manipulation_task(self, nlu_result: Dict[str, Any],
                             environment_context: Dict[str, Any]) -> Dict[str, Any]:
        """Plan manipulation task from NLU result"""
        entities = nlu_result['entities']

        # Find object entity
        object_entity = next((ent for ent in entities if ent['label'] in ['OBJECT', 'PRODUCT']), None)

        if not object_entity:
            return {
                'success': False,
                'error': 'No object specified in manipulation command',
                'plan': []
            }

        target_object = object_entity['text']

        # Check if object is in environment
        known_objects = environment_context.get('objects', [])
        if target_object not in [obj.get('name', '') for obj in known_objects]:
            # Try to resolve to known object
            resolved_object = self.resolve_object(target_object, known_objects)
            if not resolved_object:
                return {
                    'success': False,
                    'error': f'Object not found: {target_object}',
                    'suggestions': [obj.get('name', '') for obj in known_objects[:5]],
                    'plan': []
                }
            target_object = resolved_object

        # Generate manipulation plan
        plan_result = self.planning_system.plan_manipulation(target_object)

        if plan_result['success']:
            # Add task decomposition
            task_decomposition = self.task_decomposer.decompose_manipulation_task(
                target_object, environment_context
            )

            return {
                'success': True,
                'intent': 'manipulation',
                'target': target_object,
                'plan': plan_result['plan'],
                'task_decomposition': task_decomposition,
                'original_nlu': nlu_result
            }
        else:
            return {
                'success': False,
                'error': 'Failed to generate manipulation plan',
                'plan': []
            }

    def resolve_location(self, requested_location: str, known_locations: List[str]) -> Optional[str]:
        """Resolve requested location to known location using similarity"""
        if requested_location in known_locations:
            return requested_location

        # Use string similarity to find best match
        import difflib
        matches = difflib.get_close_matches(
            requested_location.lower(),
            [loc.lower() for loc in known_locations],
            n=1,
            cutoff=0.6
        )

        if matches:
            # Find the original case from known_locations
            for loc in known_locations:
                if loc.lower() == matches[0]:
                    return loc

        return None

    def resolve_object(self, requested_object: str, known_objects: List[Dict[str, Any]]) -> Optional[str]:
        """Resolve requested object to known object using similarity"""
        known_object_names = [obj.get('name', '') for obj in known_objects if obj.get('name')]

        if requested_object in known_object_names:
            return requested_object

        # Use string similarity to find best match
        import difflib
        matches = difflib.get_close_matches(
            requested_object.lower(),
            [name.lower() for name in known_object_names],
            n=1,
            cutoff=0.6
        )

        if matches:
            # Find the original case from known_object_names
            for obj_name in known_object_names:
                if obj_name.lower() == matches[0]:
                    return obj_name

        return None

    def plan_action_task(self, nlu_result: Dict[str, Any],
                        environment_context: Dict[str, Any]) -> Dict[str, Any]:
        """Plan general action task from NLU result"""
        # For general actions, we might need more specific entity extraction
        entities = nlu_result['entities']

        # This is a simplified implementation
        # In practice, this would handle various action types
        action_description = nlu_result['original_text']

        return {
            'success': True,
            'intent': 'action',
            'target': action_description,
            'plan': [
                {'action': 'interpret_command', 'params': {'command': action_description}},
                {'action': 'execute_interpreted_action', 'params': {'command': action_description}}
            ],
            'task_decomposition': [
                {'action': 'parse_action', 'description': f'Parse action: {action_description}'},
                {'action': 'validate_action', 'description': f'Validate action feasibility'},
                {'action': 'execute_action', 'description': f'Execute action: {action_description}'}
            ],
            'original_nlu': nlu_result
        }
```

## Voice Interaction Best Practices

### Robustness and Error Handling

Creating robust voice interaction systems requires careful attention to error handling and user experience:

```python
# Example: Voice interaction best practices and error handling
class RobustVoiceInteractionSystem:
    def __init__(self):
        self.speech_recognizer = AdvancedSpeechRecognitionSystem()
        self.nlu_system = NaturalLanguageUnderstandingSystem()
        self.dialogue_manager = ContextAwareDialogueManager()
        self.command_planner = VoiceCommandPlanner()

        # Error handling configuration
        self.max_recognition_attempts = 3
        self.confidence_threshold = 0.7
        self.confirmation_required_threshold = 0.8

        # User feedback mechanisms
        self.audio_feedback_enabled = True
        self.visual_feedback_enabled = True

    def process_voice_command_with_error_handling(self, audio_input) -> Dict[str, Any]:
        """Process voice command with comprehensive error handling"""
        attempt_count = 0
        last_error = None

        while attempt_count < self.max_recognition_attempts:
            try:
                # Recognize speech
                recognition_result = self.speech_recognizer.recognize_with_context(
                    audio_input,
                    context=self.get_interaction_context()
                )

                # Check confidence
                if recognition_result['confidence'] < self.confidence_threshold:
                    raise ValueError(f"Low confidence: {recognition_result['confidence']}")

                # Process with NLU
                nlu_result = self.nlu_system.understand_command(
                    recognition_result['corrected_text'],
                    context=self.get_interaction_context()
                )

                # Validate command
                validation_result = self.nlu_system.validate_command(nlu_result)
                if not validation_result['is_valid']:
                    raise ValueError(f"Invalid command: {validation_result['errors']}")

                # Plan action
                plan_result = self.command_planner.plan_from_voice_command(
                    nlu_result,
                    environment_context=self.get_environment_context()
                )

                if not plan_result['success']:
                    raise ValueError(f"Planning failed: {plan_result.get('error', 'Unknown error')}")

                # Successful processing
                return {
                    'success': True,
                    'recognition': recognition_result,
                    'nlu': nlu_result,
                    'plan': plan_result,
                    'attempts': attempt_count + 1
                }

            except ValueError as e:
                last_error = str(e)
                attempt_count += 1
                self.provide_error_feedback(str(e), attempt_count)

                if attempt_count >= self.max_recognition_attempts:
                    break

                # Wait before retry
                time.sleep(1.0)

            except Exception as e:
                self.logger.error(f"Unexpected error in voice processing: {e}")
                last_error = str(e)
                attempt_count += 1

        # All attempts failed
        return {
            'success': False,
            'error': last_error,
            'attempts': attempt_count
        }

    def provide_error_feedback(self, error: str, attempt: int):
        """Provide user feedback about recognition errors"""
        if self.audio_feedback_enabled:
            feedback_message = self.generate_error_feedback_message(error, attempt)
            self.play_audio_feedback(feedback_message)

        if self.visual_feedback_enabled:
            self.show_visual_feedback(error, attempt)

    def generate_error_feedback_message(self, error: str, attempt: int) -> str:
        """Generate appropriate error feedback message"""
        if attempt < self.max_recognition_attempts:
            if "low confidence" in error.lower():
                return f"I didn't catch that well. Could you please speak more clearly?"
            elif "invalid command" in error.lower():
                return f"I didn't understand that command. Could you rephrase it?"
            else:
                return f"I encountered an issue. Could you repeat your request?"
        else:
            return f"Sorry, I'm having trouble understanding. Maybe we can try a simpler command."

    def play_audio_feedback(self, message: str):
        """Play audio feedback to user"""
        # This would interface with text-to-speech system
        print(f"Audio feedback: {message}")  # Placeholder

    def show_visual_feedback(self, error: str, attempt: int):
        """Show visual feedback on robot display"""
        # This would interface with robot's visual display
        print(f"Visual feedback: Error on attempt {attempt}: {error}")  # Placeholder

    def get_interaction_context(self) -> Dict[str, Any]:
        """Get current interaction context"""
        return {
            'robot_state': self.dialogue_manager.context.robot_state,
            'environment_state': self.dialogue_manager.context.environment_state,
            'user_preferences': self.dialogue_manager.context.user_preferences,
            'conversation_history': self.dialogue_manager.context.conversation_history[-5:]  # Last 5 exchanges
        }

    def get_environment_context(self) -> Dict[str, Any]:
        """Get current environment context"""
        return self.dialogue_manager.context.environment_state

    def handle_confirmation_required(self, plan_result: Dict[str, Any]) -> bool:
        """Handle cases where confirmation is required before execution"""
        confidence = plan_result.get('plan', {}).get('confidence', 1.0)

        if confidence < self.confirmation_required_threshold:
            # Ask for confirmation
            target = plan_result.get('target', 'unknown')
            intent = plan_result.get('intent', 'unknown')

            confirmation_question = f"I plan to {intent} {target}. Should I proceed?"

            # In a real system, this would wait for user confirmation
            # For this example, we'll return True to continue
            return True

        return True

    def adapt_to_user_preferences(self, user_id: str) -> Dict[str, Any]:
        """Adapt interaction style based on user preferences"""
        # This would load user-specific preferences
        # For this example, return default preferences
        return {
            'preferred_commands': [],
            'voice_recognition_model': 'default',
            'interaction_style': 'polite',
            'error_tolerance': 'medium'
        }
```

## Evaluation and Performance Metrics

### Voice System Evaluation

Evaluating voice systems requires specialized metrics that account for the multi-modal nature of human-robot interaction:

```python
# Example: Voice system evaluation framework
class VoiceSystemEvaluator:
    def __init__(self):
        self.metrics = {
            'recognition_accuracy': [],
            'understanding_accuracy': [],
            'task_success_rate': [],
            'response_time': [],
            'user_satisfaction': [],
            'dialogue_coherence': []
        }
        self.test_scenarios = []
        self.evaluation_results = []

    def add_test_scenario(self, name: str, audio_input: Any, expected_output: Dict[str, Any],
                         environment_context: Dict[str, Any] = None) -> bool:
        """Add a test scenario for evaluation"""
        scenario = {
            'name': name,
            'audio_input': audio_input,
            'expected_output': expected_output,
            'environment_context': environment_context or {},
            'timestamp': time.time()
        }
        self.test_scenarios.append(scenario)
        return True

    def evaluate_system(self, voice_system: 'RobustVoiceInteractionSystem') -> Dict[str, Any]:
        """Evaluate the voice system against all test scenarios"""
        results = []

        for scenario in self.test_scenarios:
            result = self.evaluate_single_scenario(voice_system, scenario)
            results.append(result)

        # Calculate aggregate metrics
        aggregate_metrics = self.calculate_aggregate_metrics(results)

        evaluation_summary = {
            'individual_results': results,
            'aggregate_metrics': aggregate_metrics,
            'total_scenarios': len(results),
            'timestamp': time.time()
        }

        self.evaluation_results.append(evaluation_summary)
        return evaluation_summary

    def evaluate_single_scenario(self, voice_system: 'RobustVoiceInteractionSystem',
                               scenario: Dict[str, Any]) -> Dict[str, Any]:
        """Evaluate system performance on a single scenario"""
        # Process the scenario
        system_output = voice_system.process_voice_command_with_error_handling(
            scenario['audio_input']
        )

        # Compare with expected output
        recognition_accuracy = self.evaluate_recognition_accuracy(
            system_output, scenario['expected_output']
        )

        understanding_accuracy = self.evaluate_understanding_accuracy(
            system_output, scenario['expected_output']
        )

        task_success = self.evaluate_task_success(
            system_output, scenario['expected_output']
        )

        response_time = system_output.get('processing_time', 0)

        return {
            'scenario_name': scenario['name'],
            'recognition_accuracy': recognition_accuracy,
            'understanding_accuracy': understanding_accuracy,
            'task_success': task_success,
            'response_time': response_time,
            'system_output': system_output,
            'expected_output': scenario['expected_output'],
            'environment_context': scenario['environment_context']
        }

    def evaluate_recognition_accuracy(self, system_output: Dict[str, Any],
                                    expected_output: Dict[str, Any]) -> float:
        """Evaluate speech recognition accuracy"""
        if not system_output['success']:
            return 0.0

        recognized_text = system_output.get('recognition', {}).get('corrected_text', '')
        expected_text = expected_output.get('expected_text', '')

        if not recognized_text or not expected_text:
            return 0.0

        # Calculate similarity using sequence matching
        import difflib
        similarity = difflib.SequenceMatcher(
            None,
            recognized_text.lower(),
            expected_text.lower()
        ).ratio()

        return similarity

    def evaluate_understanding_accuracy(self, system_output: Dict[str, Any],
                                      expected_output: Dict[str, Any]) -> float:
        """Evaluate natural language understanding accuracy"""
        if not system_output['success']:
            return 0.0

        system_intent = system_output.get('nlu', {}).get('intent', 'unknown')
        expected_intent = expected_output.get('expected_intent', 'unknown')

        # Intent matching
        intent_match = 1.0 if system_intent == expected_intent else 0.0

        # Entity matching
        system_entities = {ent['text']: ent['label'] for ent in system_output.get('nlu', {}).get('entities', [])}
        expected_entities = expected_output.get('expected_entities', {})

        entity_matches = 0
        total_entities = len(expected_entities)

        for expected_text, expected_label in expected_entities.items():
            if expected_text in system_entities:
                if system_entities[expected_text] == expected_label:
                    entity_matches += 1

        entity_accuracy = entity_matches / total_entities if total_entities > 0 else 1.0

        # Weighted combination
        return 0.7 * intent_match + 0.3 * entity_accuracy

    def evaluate_task_success(self, system_output: Dict[str, Any],
                            expected_output: Dict[str, Any]) -> bool:
        """Evaluate if the system successfully completed the intended task"""
        if not system_output['success']:
            return False

        # Check if the planned action matches expectations
        planned_action = system_output.get('plan', {}).get('intent')
        expected_action = expected_output.get('expected_intent')

        return planned_action == expected_action

    def calculate_aggregate_metrics(self, results: List[Dict[str, Any]]) -> Dict[str, float]:
        """Calculate aggregate evaluation metrics"""
        if not results:
            return {}

        # Calculate averages
        recognition_accuracy = np.mean([r['recognition_accuracy'] for r in results])
        understanding_accuracy = np.mean([r['understanding_accuracy'] for r in results])
        task_success_rate = sum(1 for r in results if r['task_success']) / len(results)
        avg_response_time = np.mean([r['response_time'] for r in results])

        return {
            'recognition_accuracy': recognition_accuracy,
            'understanding_accuracy': understanding_accuracy,
            'task_success_rate': task_success_rate,
            'avg_response_time': avg_response_time,
            'total_evaluations': len(results)
        }

    def generate_evaluation_report(self, evaluation_summary: Dict[str, Any]) -> str:
        """Generate a comprehensive evaluation report"""
        report = []
        report.append("Voice System Evaluation Report")
        report.append("=" * 40)
        report.append(f"Evaluation Time: {time.ctime(evaluation_summary['timestamp'])}")
        report.append(f"Total Scenarios: {evaluation_summary['total_scenarios']}")
        report.append("")

        metrics = evaluation_summary['aggregate_metrics']
        report.append("Aggregate Metrics:")
        report.append(f"  Recognition Accuracy: {metrics['recognition_accuracy']:.3f}")
        report.append(f"  Understanding Accuracy: {metrics['understanding_accuracy']:.3f}")
        report.append(f"  Task Success Rate: {metrics['task_success_rate']:.3f}")
        report.append(f"  Average Response Time: {metrics['avg_response_time']:.3f}s")
        report.append("")

        # Scenario-by-scenario results
        report.append("Detailed Results:")
        for result in evaluation_summary['individual_results']:
            report.append(f"  {result['scenario_name']}:")
            report.append(f"    Recognition: {result['recognition_accuracy']:.3f}")
            report.append(f"    Understanding: {result['understanding_accuracy']:.3f}")
            report.append(f"    Success: {result['task_success']}")
            report.append(f"    Time: {result['response_time']:.3f}s")

        return "\n".join(report)
```

## Summary

The voice command and interaction system represents a critical component of humanoid robotics, enabling natural and intuitive communication between humans and robots. The system integrates multiple sophisticated technologies including speech recognition, natural language understanding, dialogue management, and action planning to create a seamless user experience.

Key components of the voice system include:

1. **Audio Processing**: Robust audio capture and preprocessing that handles real-world acoustic conditions and noise.

2. **Speech Recognition**: Advanced recognition systems that convert speech to text with high accuracy, often using models like Whisper or Wav2Vec2.

3. **Natural Language Understanding**: Sophisticated NLU systems that extract intent and entities from recognized text, resolving contextual references and handling ambiguous commands.

4. **Dialogue Management**: Context-aware dialogue systems that maintain conversation state and handle multi-turn interactions.

5. **Isaac Integration**: Specialized components that leverage the Isaac ecosystem for enhanced voice processing capabilities.

6. **Task Planning**: Integration between voice commands and robot action planning systems to execute user requests.

7. **Error Handling**: Comprehensive error handling and user feedback mechanisms that ensure robust operation in real-world conditions.

The success of the voice system depends on careful attention to real-world challenges including acoustic noise, ambiguous commands, and the need for natural, conversational interaction. The system must balance accuracy with responsiveness while maintaining safety and reliability.

## Exercises

1. Implement a speech recognition system using the Whisper model for your robot platform
2. Create a natural language understanding system that can handle domain-specific commands
3. Design a dialogue manager that maintains context across multiple interactions
4. Build an evaluation framework to test voice system performance
5. Integrate the voice system with your robot's planning and execution systems

## Further Reading

- "Spoken Language Processing" by Rabiner and Juang
- "Natural Language Understanding" by James Allen
- "Dialog Systems and Natural Language Understanding" by Walker et al.
- "Speech and Audio Signal Processing" by Rabiner and Schafer
- NVIDIA Isaac documentation on voice processing

</div>