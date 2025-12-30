---
title: "Chapter 02: Voice-to-Action Integration"
description: "Implementation of voice recognition and speech-to-action pipelines using OpenAI Whisper for robotic systems"
sidebar_label: "Chapter 02: Voice-to-Action Integration"
---

# Chapter 02: Voice-to-Action Integration

This chapter covers the implementation of voice recognition and speech-to-action pipelines using OpenAI Whisper for robotic systems.

## OpenAI Whisper Implementation

OpenAI Whisper is a state-of-the-art speech recognition model that can be leveraged for voice-controlled robotic systems. This section covers its implementation and integration.

### Introduction to OpenAI Whisper

OpenAI Whisper is a robust speech-to-text model that demonstrates high accuracy across multiple languages and diverse audio conditions. It is particularly well-suited for robotic applications due to its ability to handle noisy environments and varying audio quality.

### Whisper Model Variants

Whisper comes in several model sizes, each offering different trade-offs between accuracy and computational requirements:

- **tiny**: Fastest, least accurate (75M parameters)
- **base**: Good balance (145M parameters)
- **small**: Better accuracy (440M parameters)
- **medium**: High accuracy (769M parameters)
- **large**: Highest accuracy (1550M parameters)

For robotic applications, the choice depends on computational constraints and real-time requirements.

### Installation and Setup

```bash
pip install openai-whisper
# Or for GPU acceleration
pip install openai-whisper[cuda]
```

### Basic Whisper Implementation for Robotics

```python
import whisper
import torch
import numpy as np
from typing import Optional, Dict, Any

class WhisperSpeechRecognizer:
    def __init__(self, model_size: str = "base"):
        """
        Initialize Whisper speech recognizer
        """
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.model = whisper.load_model(model_size).to(self.device)

    def transcribe_audio(self, audio_path: str) -> Dict[str, Any]:
        """
        Transcribe audio file to text
        """
        result = self.model.transcribe(audio_path)
        return {
            "text": result["text"],
            "language": result["language"],
            "segments": result["segments"]
        }

    def transcribe_audio_buffer(self, audio_buffer: np.ndarray,
                               sample_rate: int = 16000) -> str:
        """
        Transcribe raw audio buffer to text
        """
        # Convert audio buffer to appropriate format
        audio_tensor = torch.from_numpy(audio_buffer).to(self.device)

        # Process audio
        mel = whisper.log_mel_spectrogram(audio_tensor)
        options = whisper.DecodingOptions()
        result = whisper.decode(self.model, mel, options)

        return result.text
```

### Whisper in Real-Time Robotic Systems

For real-time applications, Whisper can be integrated with audio streaming libraries:

```python
import pyaudio
import wave
import threading
import queue
from datetime import datetime

class RealTimeWhisperProcessor:
    def __init__(self, model_size: str = "base"):
        self.recognizer = WhisperSpeechRecognizer(model_size)
        self.audio_queue = queue.Queue()
        self.is_listening = False

    def start_listening(self):
        """
        Start real-time audio capture and processing
        """
        self.is_listening = True
        # Start audio capture thread
        capture_thread = threading.Thread(target=self._capture_audio)
        capture_thread.start()

        # Process audio in main thread
        while self.is_listening:
            try:
                audio_data = self.audio_queue.get(timeout=1)
                if audio_data:
                    transcription = self.recognizer.transcribe_audio_buffer(audio_data)
                    self._process_command(transcription)
            except queue.Empty:
                continue

    def _capture_audio(self):
        """
        Capture audio from microphone
        """
        chunk = 1024
        format = pyaudio.paInt16
        channels = 1
        rate = 16000

        p = pyaudio.PyAudio()

        stream = p.open(
            format=format,
            channels=channels,
            rate=rate,
            input=True,
            frames_per_buffer=chunk
        )

        print("Listening...")

        while self.is_listening:
            data = stream.read(chunk)
            audio_array = np.frombuffer(data, dtype=np.int16)
            self.audio_queue.put(audio_array)

        stream.stop_stream()
        stream.close()
        p.terminate()

    def _process_command(self, transcription: str):
        """
        Process the transcribed command
        """
        if transcription.strip():
            print(f"Recognized: {transcription}")
            # Here you would integrate with your robot's command system
            self._send_to_robot_command_system(transcription)

    def _send_to_robot_command_system(self, command: str):
        """
        Send recognized command to robot control system
        """
        # Implementation would depend on your specific robot system
        pass
```

### Optimizing Whisper for Robotic Applications

For robotic applications, consider these optimizations:

1. **Model Quantization**: Reduce model size for faster inference
2. **Audio Preprocessing**: Apply noise reduction and audio enhancement
3. **Trigger Word Detection**: Use lightweight models to detect wake words before running Whisper
4. **Caching**: Cache common phrases for faster response
5. **VAD Integration**: Use Voice Activity Detection to only process when speech is detected

## Speech-to-Action Pipeline

Creating an effective speech-to-action pipeline involves processing spoken commands and translating them into actionable robot behaviors.

### Pipeline Architecture

A comprehensive speech-to-action pipeline consists of multiple stages:

1. **Audio Capture**: Recording spoken commands from the environment
2. **Preprocessing**: Noise reduction, normalization, and audio enhancement
3. **Speech Recognition**: Converting speech to text using models like Whisper
4. **Natural Language Understanding**: Extracting intent and parameters from text
5. **Command Validation**: Ensuring commands are safe and appropriate
6. **Action Planning**: Generating sequences of robot actions
7. **Execution**: Executing planned actions on the robot
8. **Feedback**: Providing confirmation to the user

### Implementation of Speech-to-Action Pipeline

```python
from dataclasses import dataclass
from typing import List, Dict, Any, Optional
import re

@dataclass
class RobotCommand:
    action_type: str
    parameters: Dict[str, Any]
    confidence: float
    priority: int = 1

class SpeechToActionPipeline:
    def __init__(self, whisper_processor, llm_processor):
        self.whisper_processor = whisper_processor
        self.llm_processor = llm_processor
        self.command_history = []

    def process_speech_command(self, audio_input) -> Optional[RobotCommand]:
        """
        Complete pipeline from audio to robot command
        """
        # Step 1: Speech Recognition
        text = self.whisper_processor.transcribe_audio_buffer(audio_input)

        if not text.strip():
            return None

        # Step 2: Natural Language Understanding
        command = self._extract_command(text)

        if not command:
            return None

        # Step 3: Command Validation
        if not self._validate_command(command):
            print(f"Invalid command: {command.action_type}")
            return None

        # Step 4: Add to history and return
        self.command_history.append(command)
        return command

    def _extract_command(self, text: str) -> Optional[RobotCommand]:
        """
        Extract structured command from natural language text
        """
        # Use LLM to parse the command
        parsed_command = self.llm_processor.process_command(text, {
            "available_actions": self._get_available_actions(),
            "environment_context": self._get_environment_context()
        })

        if parsed_command:
            return RobotCommand(
                action_type=parsed_command["action_type"],
                parameters=parsed_command["parameters"],
                confidence=parsed_command["confidence"]
            )

        # Fallback: Rule-based parsing
        return self._rule_based_parse(text)

    def _rule_based_parse(self, text: str) -> Optional[RobotCommand]:
        """
        Fallback rule-based command parsing
        """
        text_lower = text.lower().strip()

        # Movement commands
        if "move" in text_lower or "go" in text_lower or "walk" in text_lower:
            direction = self._extract_direction(text_lower)
            distance = self._extract_distance(text_lower)
            return RobotCommand(
                action_type="move",
                parameters={"direction": direction, "distance": distance},
                confidence=0.7
            )

        # Object interaction commands
        if "pick" in text_lower or "take" in text_lower or "grasp" in text_lower:
            object_name = self._extract_object(text_lower)
            return RobotCommand(
                action_type="pick_object",
                parameters={"object": object_name},
                confidence=0.7
            )

        # Navigation commands
        if "navigate" in text_lower or "go to" in text_lower:
            location = self._extract_location(text_lower)
            return RobotCommand(
                action_type="navigate",
                parameters={"location": location},
                confidence=0.7
            )

        return None

    def _extract_direction(self, text: str) -> str:
        """
        Extract movement direction from text
        """
        if "forward" in text or "ahead" in text:
            return "forward"
        elif "backward" in text or "back" in text:
            return "backward"
        elif "left" in text:
            return "left"
        elif "right" in text:
            return "right"
        else:
            return "forward"  # default

    def _extract_distance(self, text: str) -> float:
        """
        Extract movement distance from text
        """
        # Look for number patterns followed by distance units
        pattern = r"(\d+(?:\.\d+)?)\s*(meters?|m|feet|ft|steps?)"
        match = re.search(pattern, text)

        if match:
            value = float(match.group(1))
            unit = match.group(2)

            # Convert to meters if needed
            if unit in ["feet", "ft"]:
                return value * 0.3048
            elif unit in ["steps"]:
                # Assume average step is 0.75 meters
                return value * 0.75
            else:
                return value

        return 1.0  # default distance

    def _extract_object(self, text: str) -> str:
        """
        Extract object name from text
        """
        # Simple extraction - in practice, this would use more sophisticated NLP
        # Look for words after "the", "a", "an" or after action words
        words = text.split()
        for i, word in enumerate(words):
            if word in ["the", "a", "an"] and i + 1 < len(words):
                return words[i + 1]

        # If no article found, look for potential object names
        potential_objects = ["cup", "book", "ball", "box", "table", "chair"]
        for word in words:
            if word in potential_objects:
                return word

        return "object"  # default

    def _extract_location(self, text: str) -> str:
        """
        Extract location from text
        """
        # Look for location indicators
        location_patterns = [
            r"to the (\w+)",
            r"to (\w+)",
            r"go to (\w+)",
            r"navigate to (\w+)"
        ]

        for pattern in location_patterns:
            match = re.search(pattern, text)
            if match:
                return match.group(1)

        return "unknown"

    def _validate_command(self, command: RobotCommand) -> bool:
        """
        Validate that a command is safe and appropriate
        """
        # Check if action type is supported
        if command.action_type not in self._get_available_actions():
            return False

        # Check confidence threshold
        if command.confidence < 0.5:
            return False

        # Additional safety checks could go here
        # For example, checking if navigation target is valid
        # Or ensuring manipulation targets are safe

        return True

    def _get_available_actions(self) -> List[str]:
        """
        Return list of available robot actions
        """
        return [
            "move",
            "navigate",
            "pick_object",
            "place_object",
            "greet",
            "speak",
            "turn",
            "stop"
        ]

    def _get_environment_context(self) -> Dict[str, Any]:
        """
        Get current environment context
        """
        # This would integrate with perception systems
        # to provide current environment information
        return {
            "robot_position": {"x": 0, "y": 0, "z": 0},
            "available_objects": [],
            "navigable_locations": []
        }
```

### Pipeline Optimization Strategies

To optimize the speech-to-action pipeline for robotic applications:

1. **Caching**: Cache frequently used commands and their interpretations
2. **Context Awareness**: Use environment context to disambiguate commands
3. **Confidence Thresholding**: Only execute commands above a certain confidence level
4. **Error Recovery**: Implement fallback strategies for failed commands
5. **Parallel Processing**: Process audio and plan actions in parallel where possible
6. **Command Chaining**: Allow multiple commands to be executed in sequence

## Voice Command Integration

Integrating voice commands with robotic systems requires careful consideration of command parsing, intent recognition, and action execution.

### Integration Architecture

The integration between voice processing and robotic systems typically involves:

1. **Command Interface Layer**: A standardized interface for receiving and validating commands
2. **Action Mapping**: Translating high-level commands to specific robot actions
3. **Execution Manager**: Coordinating the execution of robot actions
4. **Feedback System**: Providing status updates and acknowledgments
5. **Safety Monitor**: Ensuring commands result in safe behaviors

### ROS 2 Integration Example

Here's how to integrate the speech-to-action pipeline with ROS 2:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration

class VoiceCommandNode(Node):
    def __init__(self):
        super().__init__('voice_command_node')

        # Publishers for different robot actions
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.joint_command_publisher = self.create_publisher(JointState, '/joint_commands', 10)

        # Subscriber for receiving voice commands
        self.command_subscription = self.create_subscription(
            String,
            'voice_commands',
            self.command_callback,
            10
        )

        # Initialize speech-to-action pipeline
        self.pipeline = SpeechToActionPipeline(whisper_processor, llm_processor)

        self.get_logger().info('Voice Command Node initialized')

    def command_callback(self, msg):
        """
        Process incoming voice command
        """
        command_text = msg.data
        self.get_logger().info(f'Received command: {command_text}')

        # Process through speech-to-action pipeline
        robot_command = self.pipeline.process_speech_command(command_text)

        if robot_command:
            self.execute_robot_command(robot_command)
        else:
            self.get_logger().warn(f'Could not parse command: {command_text}')

    def execute_robot_command(self, command):
        """
        Execute a parsed robot command
        """
        if command.action_type == "move":
            self.execute_move_command(command.parameters)
        elif command.action_type == "navigate":
            self.execute_navigate_command(command.parameters)
        elif command.action_type == "pick_object":
            self.execute_pick_command(command.parameters)
        elif command.action_type == "turn":
            self.execute_turn_command(command.parameters)
        else:
            self.get_logger().warn(f'Unknown command type: {command.action_type}')

    def execute_move_command(self, params):
        """
        Execute movement command
        """
        twist_msg = Twist()

        direction = params.get('direction', 'forward')
        distance = params.get('distance', 1.0)  # meters

        # Convert distance and direction to velocity commands
        # This is a simplified example - in practice, you'd use path planning
        if direction == 'forward':
            twist_msg.linear.x = 0.5  # m/s
        elif direction == 'backward':
            twist_msg.linear.x = -0.5
        elif direction == 'left':
            twist_msg.angular.z = 0.5  # rad/s
        elif direction == 'right':
            twist_msg.angular.z = -0.5

        # Publish command for specified duration
        duration = distance / 0.5  # assuming 0.5 m/s speed
        self.publish_for_duration(twist_msg, duration)

    def execute_turn_command(self, params):
        """
        Execute turn command
        """
        angle = params.get('angle', 90.0)  # degrees
        twist_msg = Twist()

        # Convert angle to angular velocity command
        twist_msg.angular.z = 0.5  # rad/s
        duration = (angle * 3.14159 / 180) / 0.5  # angle in radians / angular velocity

        self.publish_for_duration(twist_msg, duration)

    def publish_for_duration(self, msg, duration_sec):
        """
        Publish a message for a specific duration
        """
        start_time = self.get_clock().now()
        end_time = start_time + Duration(sec=int(duration_sec))

        while self.get_clock().now() < end_time:
            self.cmd_vel_publisher.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)

def main(args=None):
    rclpy.init(args=args)
    voice_command_node = VoiceCommandNode()

    try:
        rclpy.spin(voice_command_node)
    except KeyboardInterrupt:
        pass
    finally:
        voice_command_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Voice Processing Instructions

When implementing voice processing in robotic systems, follow these instructions:

1. **Initialize Audio Subsystem**: Set up audio input devices and configure sampling rates
2. **Configure Whisper Model**: Select appropriate model size based on computational constraints
3. **Set Up LLM Interface**: Configure access to language model API
4. **Define Action Mappings**: Create mappings between voice commands and robot actions
5. **Implement Safety Checks**: Add validation to ensure commands are safe to execute
6. **Test in Various Conditions**: Validate performance in different acoustic environments

### Voice Recognition Techniques

Several techniques can improve voice recognition in robotic applications:

1. **Beamforming**: Use multiple microphones to focus on the speaker's voice
2. **Noise Suppression**: Apply filters to reduce environmental noise
3. **Echo Cancellation**: Remove robot's own speech from microphone input
4. **Speaker Adaptation**: Adjust recognition models based on individual speaker characteristics
5. **Contextual Recognition**: Use environment context to improve recognition accuracy

## Navigation

- [Previous: Chapter 01 - Introduction to VLA](./chapter-01-introduction.md)
- [Next: Chapter 03 - Cognitive Planning with LLMs](./chapter-03-cognitive-planning.md)