---
title: "Chapter 05: Capstone - Autonomous Humanoid"
description: "Integration of all VLA components into a complete autonomous humanoid system with end-to-end voice, perception, planning, and manipulation"
sidebar_label: "Chapter 05: Capstone - Autonomous Humanoid"
---

# Chapter 05: Capstone - Autonomous Humanoid

This capstone chapter integrates all VLA components into a complete autonomous humanoid system with end-to-end voice, perception, planning, and manipulation.

## VLA Component Integration

Bringing together all the components learned throughout the module to create a comprehensive system.

### Architecture of Integrated VLA System

A complete Vision-Language-Action system integrates multiple components:

```
[Human Voice Command] -----> [Speech Recognition] -----> [Language Understanding] -----> [Task Planning]
         |                          |                           |                            |
         v                          v                           v                            v
[Camera Input] -----> [Computer Vision] -----> [Object Recognition] -----> [Action Planning] -----> [Robot Execution]
         |                          |                           |                            |
         +--------------------------+---------------------------+----------------------------+
```

This diagram illustrates how visual, linguistic, and action components work together in a unified system.

### Implementation of Integrated System

Here's an implementation that brings together all VLA components:

```python
import asyncio
import threading
from typing import Dict, Any, Optional
import numpy as np
import openai
from dataclasses import dataclass

@dataclass
class VLASystemState:
    """
    State of the complete VLA system
    """
    current_task: str = ""
    detected_objects: Dict[str, Any] = None
    robot_position: Dict[str, float] = None
    system_status: str = "idle"
    last_command: str = ""
    execution_result: str = ""

class VLASystemIntegrator:
    def __init__(self, api_key: str):
        # Initialize all component systems
        self.speech_recognizer = self._initialize_speech_recognition()
        self.language_processor = self._initialize_language_processing(api_key)
        self.vision_system = self._initialize_vision_system()
        self.planning_system = self._initialize_planning_system()
        self.execution_system = self._initialize_execution_system()

        # System state
        self.state = VLASystemState()

        # Event loop for async operations
        self.loop = asyncio.new_event_loop()
        threading.Thread(target=self._run_event_loop, args=(self.loop,), daemon=True).start()

    def _initialize_speech_recognition(self):
        """
        Initialize speech recognition component (using Whisper or similar)
        """
        # This would initialize Whisper or other speech recognition system
        from .chapter_02_voice_to_action import RealTimeWhisperProcessor
        return RealTimeWhisperProcessor(model_size="base")

    def _initialize_language_processing(self, api_key: str):
        """
        Initialize language processing with LLM
        """
        # This would use the LLM-based planner from chapter 3
        from .chapter_03_cognitive_planning import LLMBasedPlanner
        return LLMBasedPlanner(api_key=api_key)

    def _initialize_vision_system(self):
        """
        Initialize computer vision system
        """
        # This would use object recognition from chapter 4
        from .chapter_04_simulated_execution import YOLOSimulator
        return YOLOSimulator()

    def _initialize_planning_system(self):
        """
        Initialize cognitive planning system
        """
        from .chapter_03_cognitive_planning import IntelligentDecisionMaker
        return IntelligentDecisionMaker(api_key="temp")  # Will be set later

    def _initialize_execution_system(self):
        """
        Initialize robot execution system
        """
        # This would interface with ROS 2 or simulation
        pass

    def _run_event_loop(self, loop):
        """
        Run the event loop in a separate thread
        """
        asyncio.set_event_loop(loop)
        loop.run_forever()

    async def process_command_async(self, command: str) -> Dict[str, Any]:
        """
        Process a command through the complete VLA pipeline
        """
        # Update state
        self.state.last_command = command
        self.state.system_status = "processing"

        # Step 1: Language Understanding
        task_plan = await self._understand_command(command)

        if not task_plan:
            self.state.system_status = "error"
            return {"status": "error", "message": "Could not understand command"}

        # Step 2: Perception (if needed)
        perception_data = await self._gather_perception_data(task_plan)

        # Step 3: Planning
        action_plan = await self._create_action_plan(task_plan, perception_data)

        # Step 4: Execution
        execution_result = await self._execute_plan(action_plan)

        # Update state with result
        self.state.execution_result = execution_result
        self.state.system_status = "completed"

        return {
            "status": "success",
            "task_plan": task_plan,
            "action_plan": action_plan,
            "execution_result": execution_result
        }

    async def _understand_command(self, command: str) -> Optional[Dict[str, Any]]:
        """
        Use LLM to understand the command and create a high-level task plan
        """
        try:
            # Use the language processing system to understand the command
            task_plan = self.language_processor.generate_plan(
                command,
                {
                    "current_state": self.state,
                    "available_actions": self.execution_system.get_available_actions() if self.execution_system else []
                }
            )
            return task_plan
        except Exception as e:
            print(f"Error in command understanding: {e}")
            return None

    async def _gather_perception_data(self, task_plan: Dict[str, Any]) -> Dict[str, Any]:
        """
        Gather perception data relevant to the task
        """
        perception_data = {"objects": [], "environment": {}}

        # If the task requires object manipulation, detect objects
        if any(step.get('action', '').startswith('pick') or
               step.get('action', '').startswith('place') for step in task_plan.get('steps', [])):
            # Capture image and detect objects
            camera_image = self._get_camera_image()  # This would interface with camera
            detected_objects = self.vision_system.detect_objects(camera_image)
            perception_data["objects"] = detected_objects

        # If the task requires navigation, get current position
        if any(step.get('action', '').startswith('navigate') or
               step.get('action', '').startswith('go_to') for step in task_plan.get('steps', [])):
            perception_data["environment"]["robot_position"] = self._get_robot_position()

        return perception_data

    async def _create_action_plan(self, task_plan: Dict[str, Any],
                                 perception_data: Dict[str, Any]) -> Dict[str, Any]:
        """
        Create detailed action plan based on high-level task and perception data
        """
        # Use planning system to create executable actions
        action_plan = self.planning_system.generate_action_plan(task_plan, perception_data)
        return action_plan

    async def _execute_plan(self, action_plan: Dict[str, Any]) -> str:
        """
        Execute the action plan on the robot
        """
        try:
            # Execute each action in the plan
            for action in action_plan.get('actions', []):
                result = await self._execute_single_action(action)
                if not result.get('success'):
                    return f"Action failed: {result.get('error', 'Unknown error')}"

            return "Plan executed successfully"
        except Exception as e:
            return f"Execution error: {str(e)}"

    async def _execute_single_action(self, action: Dict[str, Any]) -> Dict[str, Any]:
        """
        Execute a single action
        """
        action_type = action.get('type', '')

        if action_type == 'navigation':
            return self._execute_navigation(action)
        elif action_type == 'manipulation':
            return self._execute_manipulation(action)
        elif action_type == 'perception':
            return self._execute_perception(action)
        else:
            return {"success": False, "error": f"Unknown action type: {action_type}"}

    def _execute_navigation(self, action: Dict[str, Any]) -> Dict[str, Any]:
        """
        Execute navigation action
        """
        # This would interface with navigation system
        target = action.get('target', {})
        # Example: move to location
        success = True  # This would be the actual result
        return {"success": success}

    def _execute_manipulation(self, action: Dict[str, Any]) -> Dict[str, Any]:
        """
        Execute manipulation action
        """
        # This would interface with manipulation system
        target = action.get('target', {})
        # Example: pick/place object
        success = True  # This would be the actual result
        return {"success": success}

    def _execute_perception(self, action: Dict[str, Any]) -> Dict[str, Any]:
        """
        Execute perception action
        """
        # This would interface with perception system
        perception_type = action.get('perception_type', '')
        # Example: look for objects, identify scene
        success = True  # This would be the actual result
        return {"success": success}

    def _get_camera_image(self) -> np.ndarray:
        """
        Get current camera image
        """
        # This would interface with camera system
        # For simulation, return a placeholder
        return np.zeros((480, 640, 3), dtype=np.uint8)

    def _get_robot_position(self) -> Dict[str, float]:
        """
        Get current robot position
        """
        # This would interface with localization system
        return {"x": 0.0, "y": 0.0, "z": 0.0, "theta": 0.0}

    def process_voice_command(self, audio_input) -> Dict[str, Any]:
        """
        Process a complete voice command from audio input to robot action
        """
        # Step 1: Speech recognition
        command_text = self.speech_recognizer.transcribe_audio_buffer(audio_input)

        if not command_text.strip():
            return {"status": "error", "message": "Could not recognize speech"}

        # Step 2: Process through VLA pipeline
        future = asyncio.run_coroutine_threadsafe(
            self.process_command_async(command_text),
            self.loop
        )

        # Wait for result
        result = future.result()

        return result

    def start_listening_for_commands(self):
        """
        Start listening for voice commands continuously
        """
        def command_listener():
            # This would continuously listen for commands
            # For this example, we'll simulate
            pass

        listener_thread = threading.Thread(target=command_listener, daemon=True)
        listener_thread.start()

        return listener_thread
```

### Integration Patterns

When integrating VLA components, several patterns emerge:

1. **Sequential Integration**: Process components one after another (language → vision → action)
2. **Parallel Integration**: Process components simultaneously where possible
3. **Feedback Integration**: Use action results to refine perception and language understanding
4. **Hierarchical Integration**: Organize components in a hierarchy of complexity

### Real-time Integration Considerations

For real-time operation of integrated VLA systems:

1. **Latency Management**: Ensure each component processes data within time constraints
2. **Resource Allocation**: Balance computational resources between components
3. **Failure Handling**: Gracefully handle failures in individual components
4. **State Synchronization**: Keep all components synchronized with the current state

This integration demonstrates how the vision, language, and action components work together to create a complete autonomous system capable of understanding natural language commands, perceiving the environment, and executing appropriate actions.

## End-to-End Voice Processing

Complete voice processing pipeline from speech recognition to action execution.

### Voice Processing Pipeline Architecture

The end-to-end voice processing pipeline consists of multiple interconnected stages:

```
[Raw Audio] --> [Preprocessing] --> [Speech Recognition] --> [Language Understanding] --> [Action Generation] --> [Execution]
      |              |                    |                         |                          |                  |
   Microphone    Noise Reduction    Whisper/ASR            LLM/NLU                 Task Planning        Robot Actions
```

### Implementation of Complete Voice Pipeline

Here's a comprehensive implementation of the end-to-end voice processing system:

```python
import asyncio
import threading
import queue
import time
from typing import Dict, Any, Optional, Callable
import numpy as np
import pyaudio
import openai
from dataclasses import dataclass

@dataclass
class VoiceCommand:
    """
    Represents a processed voice command with all necessary information
    """
    text: str
    confidence: float
    timestamp: float
    intent: str
    entities: Dict[str, Any]
    action_plan: Optional[Dict[str, Any]] = None

class EndToEndVoiceProcessor:
    def __init__(self, api_key: str, execution_callback: Callable = None):
        # Initialize components from previous chapters
        self.speech_recognizer = self._initialize_speech_recognizer()
        self.language_processor = self._initialize_language_processor(api_key)
        self.action_planner = self._initialize_action_planner(api_key)
        self.execution_callback = execution_callback or self._default_execution

        # Audio processing parameters
        self.audio_format = pyaudio.paInt16
        self.channels = 1
        self.rate = 16000
        self.chunk = 1024
        self.audio_queue = queue.Queue()

        # Voice activity detection parameters
        self.energy_threshold = 1000
        self.silence_duration = 1.0  # seconds of silence to trigger processing

        # Processing state
        self.is_listening = False
        self.audio_buffer = np.array([])
        self.listening_thread = None

    def _initialize_speech_recognizer(self):
        """
        Initialize speech recognition system (Whisper-based)
        """
        from .chapter_02_voice_to_action import RealTimeWhisperProcessor
        return RealTimeWhisperProcessor(model_size="base")

    def _initialize_language_processor(self, api_key: str):
        """
        Initialize language understanding system (LLM-based)
        """
        from .chapter_03_cognitive_planning import LLMBasedPlanner
        return LLMBasedPlanner(api_key=api_key)

    def _initialize_action_planner(self, api_key: str):
        """
        Initialize action planning system
        """
        from .chapter_03_cognitive_planning import IntelligentDecisionMaker
        return IntelligentDecisionMaker(api_key=api_key)

    def _default_execution(self, action_plan: Dict[str, Any]) -> bool:
        """
        Default execution callback if none provided
        """
        print(f"Executing action plan: {action_plan}")
        # This would interface with the robot execution system
        return True

    def _calculate_audio_energy(self, audio_data: np.ndarray) -> float:
        """
        Calculate energy of audio signal for voice activity detection
        """
        return np.mean(np.abs(audio_data))

    def _is_speech_detected(self, audio_data: np.ndarray) -> bool:
        """
        Simple voice activity detection based on energy threshold
        """
        energy = self._calculate_audio_energy(audio_data)
        return energy > self.energy_threshold

    def _process_audio_chunk(self, audio_chunk: bytes) -> np.ndarray:
        """
        Process raw audio chunk to numpy array
        """
        audio_array = np.frombuffer(audio_chunk, dtype=np.int16)
        return audio_array.astype(np.float32) / 32768.0  # Normalize to [-1, 1]

    def _start_audio_capture(self):
        """
        Start capturing audio from microphone
        """
        p = pyaudio.PyAudio()

        stream = p.open(
            format=self.audio_format,
            channels=self.channels,
            rate=self.rate,
            input=True,
            frames_per_buffer=self.chunk
        )

        print("Listening... (Say 'quit' to stop)")

        while self.is_listening:
            try:
                # Read audio chunk
                audio_chunk = stream.read(self.chunk, exception_on_overflow=False)
                audio_array = self._process_audio_chunk(audio_chunk)

                # Check for speech
                if self._is_speech_detected(audio_array):
                    # Add to buffer for processing
                    self.audio_buffer = np.concatenate([self.audio_buffer, audio_array])
                else:
                    # If we have accumulated speech and now have silence
                    if len(self.audio_buffer) > 0:
                        # Check if silence has been detected long enough
                        # In a real implementation, we'd track silence duration
                        # For simplicity, we'll process when silence is detected
                        if len(audio_array) > 0:  # This is a simplification
                            # Process the accumulated audio
                            if len(self.audio_buffer) > self.rate * 0.5:  # At least 0.5 seconds
                                self._process_voice_command(self.audio_buffer.copy())
                            self.audio_buffer = np.array([])  # Clear buffer

            except Exception as e:
                print(f"Audio capture error: {e}")
                break

        stream.stop_stream()
        stream.close()
        p.terminate()

    def _process_voice_command(self, audio_buffer: np.ndarray):
        """
        Process the accumulated audio buffer through the complete pipeline
        """
        try:
            # Step 1: Speech Recognition
            print("Processing speech...")
            text = self.speech_recognizer.transcribe_audio_buffer(audio_buffer)

            if not text.strip():
                print("No speech recognized")
                return

            print(f"Recognized: {text}")

            # Step 2: Language Understanding and Intent Classification
            print("Understanding command...")
            intent, entities = self._understand_command(text)

            # Step 3: Action Planning
            print("Planning actions...")
            action_plan = self._generate_action_plan(text, intent, entities)

            # Step 4: Execution
            print("Executing actions...")
            success = self._execute_action_plan(action_plan)

            if success:
                print("Command executed successfully")
            else:
                print("Command execution failed")

        except Exception as e:
            print(f"Error processing voice command: {e}")

    def _understand_command(self, text: str) -> tuple[str, Dict[str, Any]]:
        """
        Use LLM to understand the intent and extract entities from the command
        """
        prompt = f"""
        Analyze the following command: "{text}"

        Identify the main intent and extract any entities (objects, locations, etc.).

        Respond in JSON format with:
        - intent: the main action intent
        - entities: a dictionary of extracted entities
        """

        try:
            response = self.language_processor.client.chat.completions.create(
                model="gpt-3.5-turbo",
                messages=[{"role": "user", "content": prompt}],
                temperature=0.1,
                response_format={"type": "json_object"}
            )

            result = eval(response.choices[0].message.content)
            intent = result.get("intent", "unknown")
            entities = result.get("entities", {})

            return intent, entities

        except Exception as e:
            print(f"Error in command understanding: {e}")
            return "unknown", {}

    def _generate_action_plan(self, text: str, intent: str, entities: Dict[str, Any]) -> Dict[str, Any]:
        """
        Generate an action plan based on the understood command
        """
        context = {
            "available_actions": ["move", "navigate", "pick_object", "place_object", "greet", "speak"],
            "current_state": {"position": {"x": 0, "y": 0, "z": 0}, "battery": 0.8}
        }

        action_plan = self.action_planner.generate_plan(text, context)
        return action_plan

    def _execute_action_plan(self, action_plan: Dict[str, Any]) -> bool:
        """
        Execute the generated action plan
        """
        if not action_plan or not action_plan.steps:
            print("No action plan to execute")
            return False

        # Execute each step in the plan
        for step in action_plan.steps:
            print(f"Executing: {step.action} with params {step.parameters}")
            success = self.execution_callback({
                "action": step.action,
                "parameters": step.parameters,
                "preconditions": step.preconditions,
                "effects": step.effects
            })

            if not success:
                print(f"Failed to execute action: {step.action}")
                return False

        return True

    def start_listening(self):
        """
        Start listening for voice commands
        """
        if self.is_listening:
            print("Already listening")
            return

        self.is_listening = True
        self.listening_thread = threading.Thread(target=self._start_audio_capture, daemon=True)
        self.listening_thread.start()

        print("Voice processor started. Listening for commands...")

    def stop_listening(self):
        """
        Stop listening for voice commands
        """
        self.is_listening = False
        if self.listening_thread:
            self.listening_thread.join(timeout=2.0)  # Wait up to 2 seconds for thread to finish

        print("Voice processor stopped.")

    async def process_voice_command_async(self, audio_input: np.ndarray) -> VoiceCommand:
        """
        Process a voice command asynchronously
        """
        # Create a task for each processing step
        tasks = [
            asyncio.create_task(self._async_speech_recognition(audio_input)),
            asyncio.create_task(self._async_language_understanding(audio_input))
        ]

        # Wait for results
        results = await asyncio.gather(*tasks, return_exceptions=True)

        # Process results
        if isinstance(results[0], Exception):
            raise results[0]

        if isinstance(results[1], Exception):
            raise results[1]

        text, confidence = results[0]
        intent, entities = results[1]

        # Generate action plan
        action_plan = await self._async_action_planning(text, intent, entities)

        return VoiceCommand(
            text=text,
            confidence=confidence,
            timestamp=time.time(),
            intent=intent,
            entities=entities,
            action_plan=action_plan
        )

    async def _async_speech_recognition(self, audio_input: np.ndarray) -> tuple[str, float]:
        """
        Asynchronous speech recognition
        """
        loop = asyncio.get_event_loop()
        # In a real implementation, this would be truly async
        text = await loop.run_in_executor(None, lambda: self.speech_recognizer.transcribe_audio_buffer(audio_input))
        # For simplicity, assume high confidence
        return text, 0.9

    async def _async_language_understanding(self, audio_input: np.ndarray) -> tuple[str, Dict[str, Any]]:
        """
        Asynchronous language understanding
        """
        # First convert audio to text
        text = await self._async_speech_recognition(audio_input)
        intent, entities = await asyncio.get_event_loop().run_in_executor(
            None, lambda: self._understand_command(text[0])
        )
        return intent, entities

    async def _async_action_planning(self, text: str, intent: str, entities: Dict[str, Any]) -> Dict[str, Any]:
        """
        Asynchronous action planning
        """
        loop = asyncio.get_event_loop()
        action_plan = await loop.run_in_executor(
            None, lambda: self._generate_action_plan(text, intent, entities)
        )
        return action_plan

class VoiceProcessingSystem:
    """
    Complete voice processing system that integrates all components
    """
    def __init__(self, api_key: str):
        self.voice_processor = EndToEndVoiceProcessor(api_key)
        self.is_running = False

    def start_system(self):
        """
        Start the complete voice processing system
        """
        print("Starting VLA Voice Processing System...")
        self.voice_processor.start_listening()
        self.is_running = True

        try:
            # Keep the system running
            while self.is_running:
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\nShutting down voice processing system...")
        finally:
            self.stop_system()

    def stop_system(self):
        """
        Stop the voice processing system
        """
        self.voice_processor.stop_listening()
        self.is_running = False
        print("Voice processing system stopped.")

# Example usage
def example_execution_callback(action_data: Dict[str, Any]) -> bool:
    """
    Example execution callback that simulates robot actions
    """
    action = action_data.get("action", "")
    params = action_data.get("parameters", {})

    print(f"Simulating execution of: {action} with parameters: {params}")
    time.sleep(0.5)  # Simulate execution time
    return True

# Usage example:
# system = VoiceProcessingSystem("your-openai-api-key")
# system.voice_processor.execution_callback = example_execution_callback
# system.start_system()
```

### Voice Pipeline Optimization

For efficient end-to-end voice processing:

1. **Streaming Processing**: Process audio in real-time rather than waiting for complete utterances
2. **Wake Word Detection**: Use lightweight models to detect when the robot should start listening
3. **Confidence Thresholding**: Only process commands with sufficient confidence
4. **Context Awareness**: Use environmental context to improve recognition accuracy
5. **Error Recovery**: Implement fallback strategies for failed recognitions

This end-to-end voice processing system demonstrates how all components work together to enable natural human-robot interaction through spoken commands.

## Perception Systems for Autonomy

Advanced perception systems that enable autonomous operation of humanoid robots.

### Overview of Perception Systems

Perception systems form the foundation of robot autonomy, enabling robots to understand and interact with their environment. For humanoid robots, perception systems must handle complex 3D environments and social interactions.

### Multi-Modal Perception Architecture

Humanoid robots require integration of multiple sensory modalities:

```
Visual Perception (Cameras) ----+
                                |
                                +--> [Sensor Fusion] --> [Environment Model]
                                |         |
Audio Perception (Microphones) --+         +--> [Action Planning]
                                |         |
Tactile Perception (Sensors) ----+         +--> [Navigation]
                                          |
LIDAR/Depth Perception -------------------+
```

### Implementation of Integrated Perception System

Here's an implementation of a multi-modal perception system for humanoid autonomy:

```python
import numpy as np
import cv2
from typing import Dict, Any, List, Tuple, Optional
import threading
import time
from dataclasses import dataclass
from enum import Enum

class SensorType(Enum):
    CAMERA_RGB = "camera_rgb"
    CAMERA_DEPTH = "camera_depth"
    LIDAR = "lidar"
    MICROPHONE = "microphone"
    TACTILE = "tactile"
    IMU = "imu"

@dataclass
class PerceptionData:
    """
    Container for perception data from different sensors
    """
    timestamp: float
    sensor_type: SensorType
    data: Any
    confidence: float = 1.0

@dataclass
class EnvironmentObject:
    """
    Represents an object detected in the environment
    """
    id: str
    class_name: str
    position: np.ndarray  # 3D position [x, y, z]
    orientation: np.ndarray  # 3D orientation [roll, pitch, yaw]
    dimensions: np.ndarray  # 3D dimensions [width, height, depth]
    confidence: float
    tracked: bool = False

@dataclass
class HumanoidPerceptionState:
    """
    State of the humanoid perception system
    """
    objects: List[EnvironmentObject]
    robot_pose: np.ndarray  # [x, y, z, roll, pitch, yaw]
    environment_map: np.ndarray  # Occupancy grid or point cloud
    audio_context: Dict[str, Any]  # Detected speech, sounds
    last_update_time: float

class MultiModalPerceptionSystem:
    def __init__(self):
        # Initialize sensor interfaces
        self.sensors = {}
        self.perception_queue = {}
        self.fusion_lock = threading.Lock()

        # Perception state
        self.state = HumanoidPerceptionState(
            objects=[],
            robot_pose=np.zeros(6),  # [x, y, z, roll, pitch, yaw]
            environment_map=np.zeros((100, 100)),  # Placeholder occupancy grid
            audio_context={},
            last_update_time=time.time()
        )

        # Object tracking
        self.object_tracker = self._initialize_object_tracker()

        # Threading for real-time processing
        self.processing_thread = None
        self.is_running = False

    def _initialize_object_tracker(self):
        """
        Initialize object tracking system
        """
        # For this example, we'll use a simple tracking approach
        # In practice, this could use SORT, DeepSORT, or other tracking algorithms
        class SimpleObjectTracker:
            def __init__(self):
                self.next_id = 0
                self.tracked_objects = {}  # object_id -> object_data

            def track_objects(self, detected_objects: List[EnvironmentObject]) -> List[EnvironmentObject]:
                """
                Track objects across frames
                """
                updated_objects = []

                for obj in detected_objects:
                    # Check if similar object was detected before
                    matched = False
                    for tracked_id, tracked_obj in self.tracked_objects.items():
                        # Simple distance-based matching
                        distance = np.linalg.norm(obj.position - tracked_obj.position)
                        if distance < 0.5:  # 50cm threshold
                            # Update tracked object
                            tracked_obj.position = obj.position
                            tracked_obj.confidence = max(tracked_obj.confidence, obj.confidence)
                            tracked_obj.tracked = True
                            updated_objects.append(tracked_obj)
                            matched = True
                            break

                    if not matched:
                        # Assign new ID to new object
                        obj.id = f"obj_{self.next_id}"
                        self.next_id += 1
                        self.tracked_objects[obj.id] = obj
                        updated_objects.append(obj)

                # Mark untracked objects
                for obj in self.tracked_objects.values():
                    if not obj.tracked:
                        # Remove old objects that haven't been seen recently
                        if time.time() - obj.last_seen > 5.0:  # 5 seconds
                            del self.tracked_objects[obj.id]
                    else:
                        obj.tracked = False  # Reset for next frame

                return updated_objects

        return SimpleObjectTracker()

    def register_sensor(self, sensor_type: SensorType, sensor_interface):
        """
        Register a sensor interface with the perception system
        """
        self.sensors[sensor_type] = sensor_interface
        self.perception_queue[sensor_type] = []

    def process_sensor_data(self, sensor_type: SensorType, data: Any) -> PerceptionData:
        """
        Process data from a specific sensor
        """
        timestamp = time.time()

        if sensor_type == SensorType.CAMERA_RGB:
            return self._process_camera_data(data, timestamp)
        elif sensor_type == SensorType.CAMERA_DEPTH:
            return self._process_depth_data(data, timestamp)
        elif sensor_type == SensorType.LIDAR:
            return self._process_lidar_data(data, timestamp)
        elif sensor_type == SensorType.MICROPHONE:
            return self._process_audio_data(data, timestamp)
        else:
            # For other sensor types, just wrap the data
            return PerceptionData(
                timestamp=timestamp,
                sensor_type=sensor_type,
                data=data,
                confidence=0.8  # Default confidence
            )

    def _process_camera_data(self, image: np.ndarray, timestamp: float) -> PerceptionData:
        """
        Process RGB camera data for object detection and recognition
        """
        # This would interface with the object recognition system from chapter 4
        from .chapter_04_simulated_execution import YOLOSimulator
        detector = YOLOSimulator()

        # Detect objects in the image
        detections = detector.detect_objects(image)

        # Convert to environment objects
        environment_objects = []
        for detection in detections:
            # For simplicity, assume we can estimate 3D position
            # In reality, this would require depth information or stereo vision
            obj = EnvironmentObject(
                id=f"cam_obj_{len(environment_objects)}",
                class_name=detection["class"],
                position=np.array([0, 0, 0]),  # Placeholder - would need depth
                orientation=np.array([0, 0, 0]),
                dimensions=np.array([0.1, 0.1, 0.1]),  # Placeholder
                confidence=detection["confidence"]
            )
            environment_objects.append(obj)

        return PerceptionData(
            timestamp=timestamp,
            sensor_type=SensorType.CAMERA_RGB,
            data=environment_objects,
            confidence=0.9
        )

    def _process_depth_data(self, depth_image: np.ndarray, timestamp: float) -> PerceptionData:
        """
        Process depth camera data for 3D reconstruction and object localization
        """
        # Convert depth image to 3D points
        height, width = depth_image.shape
        # Assume camera intrinsics for 3D reconstruction
        fx, fy = 525.0, 525.0  # Focal lengths
        cx, cy = width / 2, height / 2  # Principal point

        # Create coordinate grids
        x_coords, y_coords = np.meshgrid(np.arange(width), np.arange(height))

        # Convert to 3D coordinates
        z_coords = depth_image
        x_coords = (x_coords - cx) * z_coords / fx
        y_coords = (y_coords - cy) * z_coords / fy

        # Stack to create point cloud
        point_cloud = np.stack([x_coords, y_coords, z_coords], axis=-1)

        # Remove invalid points
        valid_mask = ~np.isnan(point_cloud).any(axis=2) & ~np.isinf(point_cloud).any(axis=2)
        valid_points = point_cloud[valid_mask]

        return PerceptionData(
            timestamp=timestamp,
            sensor_type=SensorType.CAMERA_DEPTH,
            data=valid_points,
            confidence=0.95
        )

    def _process_lidar_data(self, lidar_scan: List[float], timestamp: float) -> PerceptionData:
        """
        Process LIDAR data for environment mapping and obstacle detection
        """
        # Convert LIDAR scan to occupancy grid or point cloud
        # This is a simplified example
        occupancy_grid = np.zeros((100, 100))  # 10m x 10m grid with 0.1m resolution

        # Convert polar coordinates to Cartesian
        for i, distance in enumerate(lidar_scan):
            if distance < 10.0:  # Max range 10m
                angle = i * (2 * np.pi / len(lidar_scan)) - np.pi
                x = int(distance * np.cos(angle) / 0.1) + 50  # Center at (50, 50)
                y = int(distance * np.sin(angle) / 0.1) + 50

                if 0 <= x < 100 and 0 <= y < 100:
                    occupancy_grid[x, y] = 1  # Mark as occupied

        return PerceptionData(
            timestamp=timestamp,
            sensor_type=SensorType.LIDAR,
            data=occupancy_grid,
            confidence=0.9
        )

    def _process_audio_data(self, audio_data: np.ndarray, timestamp: float) -> PerceptionData:
        """
        Process audio data for sound source localization and speech recognition
        """
        # This would interface with the voice processing system
        # For now, return a placeholder
        audio_context = {
            "speech_detected": False,
            "sound_sources": [],
            "ambient_noise_level": np.mean(np.abs(audio_data))
        }

        return PerceptionData(
            timestamp=timestamp,
            sensor_type=SensorType.MICROPHONE,
            data=audio_context,
            confidence=0.8
        )

    def fuse_sensor_data(self, sensor_data_list: List[PerceptionData]) -> HumanoidPerceptionState:
        """
        Fuse data from multiple sensors to create a coherent perception state
        """
        with self.fusion_lock:
            new_state = HumanoidPerceptionState(
                objects=[],
                robot_pose=self.state.robot_pose.copy(),
                environment_map=self.state.environment_map.copy(),
                audio_context=self.state.audio_context.copy(),
                last_update_time=time.time()
            )

            # Process each sensor's data
            for sensor_data in sensor_data_list:
                if sensor_data.sensor_type == SensorType.CAMERA_RGB:
                    # Update objects with visual detections
                    new_state.objects.extend(sensor_data.data)
                elif sensor_data.sensor_type == SensorType.CAMERA_DEPTH:
                    # Use depth data to refine object positions
                    self._refine_object_positions(new_state, sensor_data.data)
                elif sensor_data.sensor_type == SensorType.LIDAR:
                    # Update environment map with LIDAR data
                    new_state.environment_map = sensor_data.data
                elif sensor_data.sensor_type == SensorType.MICROPHONE:
                    # Update audio context
                    new_state.audio_context.update(sensor_data.data)

            # Track objects across frames
            new_state.objects = self.object_tracker.track_objects(new_state.objects)

            # Update the system state
            self.state = new_state

            return new_state

    def _refine_object_positions(self, state: HumanoidPerceptionState, depth_data: np.ndarray):
        """
        Use depth data to refine the 3D positions of detected objects
        """
        # This would use the depth information to assign real 3D positions to objects
        # For this example, we'll just update the positions if we have depth data
        for obj in state.objects:
            # In a real implementation, this would project the object's 2D bounding box
            # into 3D space using the depth data
            pass

    def get_environment_state(self) -> HumanoidPerceptionState:
        """
        Get the current environment state
        """
        return self.state

    def get_reachable_objects(self, robot_position: np.ndarray, max_distance: float = 1.0) -> List[EnvironmentObject]:
        """
        Get objects that are within reach of the robot
        """
        reachable = []
        for obj in self.state.objects:
            distance = np.linalg.norm(obj.position - robot_position[:3])
            if distance <= max_distance:
                reachable.append(obj)
        return reachable

    def get_navigation_map(self) -> np.ndarray:
        """
        Get the current navigation map based on perception data
        """
        return self.state.environment_map

    def start_perception_loop(self):
        """
        Start the continuous perception processing loop
        """
        def perception_loop():
            while self.is_running:
                try:
                    # Collect data from all sensors
                    sensor_data_list = []

                    for sensor_type, sensor_interface in self.sensors.items():
                        # Get latest sensor data
                        sensor_data = self.process_sensor_data(sensor_type, sensor_interface.get_data())
                        sensor_data_list.append(sensor_data)

                    # Fuse sensor data
                    self.fuse_sensor_data(sensor_data_list)

                    # Sleep to control processing rate
                    time.sleep(0.1)  # 10Hz processing rate

                except Exception as e:
                    print(f"Perception loop error: {e}")

        self.is_running = True
        self.processing_thread = threading.Thread(target=perception_loop, daemon=True)
        self.processing_thread.start()

    def stop_perception_loop(self):
        """
        Stop the perception processing loop
        """
        self.is_running = False
        if self.processing_thread:
            self.processing_thread.join(timeout=2.0)

class AutonomousPerceptionManager:
    """
    Manager for the complete perception system in autonomous operation
    """
    def __init__(self, api_key: str):
        self.perception_system = MultiModalPerceptionSystem()
        self.vla_integrator = self._initialize_vla_integrator(api_key)

    def _initialize_vla_integrator(self, api_key: str):
        """
        Initialize the VLA system integrator
        """
        from .chapter_05_capstone import VLASystemIntegrator
        return VLASystemIntegrator(api_key=api_key)

    def process_environment(self) -> Dict[str, Any]:
        """
        Process the current environment and return actionable information
        """
        # Get current perception state
        perception_state = self.perception_system.get_environment_state()

        # Analyze environment for actionable insights
        environment_analysis = {
            "reachable_objects": len(self.perception_system.get_reachable_objects(
                perception_state.robot_pose)),
            "navigation_clear": self._check_navigation_clearance(),
            "human_presence": self._detect_human_presence(perception_state),
            "obstacle_density": self._calculate_obstacle_density(perception_state)
        }

        return environment_analysis

    def _check_navigation_clearance(self) -> bool:
        """
        Check if navigation path is clear
        """
        # This would check the navigation map for obstacles
        nav_map = self.perception_system.get_navigation_map()
        # Simple check: if more than 50% of the map is free space
        free_space_ratio = np.sum(nav_map == 0) / nav_map.size
        return free_space_ratio > 0.5

    def _detect_human_presence(self, state: HumanoidPerceptionState) -> bool:
        """
        Detect if humans are present in the environment
        """
        # Check if any detected objects are classified as "person"
        for obj in state.objects:
            if obj.class_name == "person":
                return True
        return False

    def _calculate_obstacle_density(self, state: HumanoidPerceptionState) -> float:
        """
        Calculate the density of obstacles in the environment
        """
        nav_map = self.perception_system.get_navigation_map()
        obstacle_ratio = np.sum(nav_map == 1) / nav_map.size
        return obstacle_ratio

# Example usage
def example_perception_callback(sensor_data: PerceptionData):
    """
    Example callback for sensor data processing
    """
    print(f"Received {sensor_data.sensor_type.value} data at {sensor_data.timestamp}")

# Usage example:
# manager = AutonomousPerceptionManager("your-api-key")
# manager.perception_system.register_sensor(SensorType.CAMERA_RGB, your_camera_interface)
# manager.perception_system.start_perception_loop()
#
# # In your main loop:
# environment_info = manager.process_environment()
# print(f"Environment analysis: {environment_info}")
```

### Perception System Integration with VLA

The perception system integrates with the VLA framework by providing the environmental context needed for:

1. **Visual Processing**: Object recognition and scene understanding for the vision component
2. **Context Awareness**: Environmental state for the language understanding component
3. **Action Constraints**: Physical constraints and object locations for the action component

### Perception System Optimization

For efficient operation of perception systems in humanoid robots:

1. **Selective Processing**: Focus computational resources on relevant areas of interest
2. **Multi-Resolution Processing**: Use different resolution levels for different tasks
3. **Predictive Processing**: Anticipate where to focus perception based on robot's goals
4. **Sensor Fusion**: Combine data from multiple sensors for robust perception
5. **Real-time Constraints**: Maintain real-time processing for responsive behavior

This perception system provides the foundation for autonomous operation of humanoid robots by continuously monitoring and understanding the environment.

## Planning Algorithms for Autonomy

Sophisticated planning algorithms that enable complex autonomous behaviors.

### Overview of Autonomous Planning

Autonomous planning in humanoid robots involves generating sequences of actions that achieve complex goals while considering environmental constraints, robot capabilities, and safety requirements. The planning process must operate at multiple levels of abstraction and time scales.

### Hierarchical Planning Architecture

Autonomous planning typically follows a hierarchical structure:

```
[Task Planning]        # High-level goal decomposition
     |
[Behavior Planning]    # Behavior selection and coordination
     |
[Motion Planning]      # Path and trajectory generation
     |
[Control Execution]     # Low-level motor control
```

### Implementation of Hierarchical Planning System

Here's an implementation of a hierarchical planning system for humanoid autonomy:

```python
import numpy as np
from typing import List, Dict, Any, Optional, Tuple
from dataclasses import dataclass
from enum import Enum
import heapq
import time
from abc import ABC, abstractmethod

class PlanningLevel(Enum):
    TASK = "task"
    BEHAVIOR = "behavior"
    MOTION = "motion"
    CONTROL = "control"

@dataclass
class PlanStep:
    """
    A single step in a plan
    """
    action: str
    parameters: Dict[str, Any]
    duration: float
    preconditions: List[str]
    effects: List[str]
    priority: int = 1

@dataclass
class Plan:
    """
    A complete plan consisting of multiple steps
    """
    steps: List[PlanStep]
    start_time: float
    estimated_duration: float
    confidence: float

@dataclass
class PlanningContext:
    """
    Context information for planning
    """
    current_state: Dict[str, Any]
    goal_state: Dict[str, Any]
    environment_map: np.ndarray
    robot_capabilities: Dict[str, Any]
    constraints: Dict[str, Any]

class BasePlanner(ABC):
    """
    Abstract base class for planners at different levels
    """
    def __init__(self, level: PlanningLevel):
        self.level = level
        self.last_plan_time = 0.0

    @abstractmethod
    def plan(self, context: PlanningContext) -> Optional[Plan]:
        """
        Generate a plan based on the context
        """
        pass

    def validate_plan(self, plan: Plan, context: PlanningContext) -> bool:
        """
        Validate that the plan is executable in the current context
        """
        return True

class TaskPlanner(BasePlanner):
    """
    High-level task planning that decomposes complex goals into subtasks
    """
    def __init__(self):
        super().__init__(PlanningLevel.TASK)

    def plan(self, context: PlanningContext) -> Optional[Plan]:
        """
        Generate high-level task plan
        """
        goal = context.goal_state.get("task", "")
        current_state = context.current_state

        # Decompose complex tasks into subtasks
        subtasks = self._decompose_task(goal, current_state)

        # Create plan steps
        steps = []
        for i, subtask in enumerate(subtasks):
            step = PlanStep(
                action="execute_subtask",
                parameters={"subtask": subtask, "index": i},
                duration=self._estimate_subtask_duration(subtask),
                preconditions=[],
                effects=[f"subtask_{i}_completed"],
                priority=1
            )
            steps.append(step)

        if not steps:
            return None

        return Plan(
            steps=steps,
            start_time=time.time(),
            estimated_duration=sum(step.duration for step in steps),
            confidence=0.8
        )

    def _decompose_task(self, goal: str, current_state: Dict[str, Any]) -> List[str]:
        """
        Decompose a high-level goal into subtasks
        """
        subtasks = []

        if "bring" in goal.lower() or "fetch" in goal.lower():
            # Fetch task: go to object, pick up, bring back
            subtasks = ["navigate_to_object", "pick_object", "navigate_to_destination", "place_object"]
        elif "navigate" in goal.lower() or "go to" in goal.lower():
            # Navigation task
            subtasks = ["plan_path", "execute_navigation"]
        elif "clean" in goal.lower():
            # Cleaning task
            subtasks = ["identify_dirty_areas", "navigate_to_area", "clean_area", "check_cleanliness"]
        else:
            # Default: single action
            subtasks = [goal]

        return subtasks

    def _estimate_subtask_duration(self, subtask: str) -> float:
        """
        Estimate duration for a subtask
        """
        duration_map = {
            "navigate_to_object": 30.0,
            "pick_object": 10.0,
            "navigate_to_destination": 30.0,
            "place_object": 5.0,
            "plan_path": 2.0,
            "execute_navigation": 25.0,
            "identify_dirty_areas": 15.0,
            "clean_area": 45.0,
            "check_cleanliness": 5.0
        }
        return duration_map.get(subtask, 10.0)

class BehaviorPlanner(BasePlanner):
    """
    Behavior planning that selects appropriate robot behaviors
    """
    def __init__(self):
        super().__init__(PlanningLevel.BEHAVIOR)
        self.behavior_library = self._initialize_behavior_library()

    def _initialize_behavior_library(self) -> Dict[str, Dict[str, Any]]:
        """
        Initialize the library of available behaviors
        """
        return {
            "approach_object": {
                "actions": ["align_to_object", "move_closer", "reach_out"],
                "conditions": ["object_detected", "reachable"],
                "duration": 8.0
            },
            "grasp_object": {
                "actions": ["position_hand", "close_gripper", "verify_grasp"],
                "conditions": ["object_aligned", "gripper_ready"],
                "duration": 5.0
            },
            "avoid_obstacle": {
                "actions": ["stop_motion", "plan_detour", "execute_detour"],
                "conditions": ["obstacle_detected", "path_blocked"],
                "duration": 15.0
            },
            "greet_human": {
                "actions": ["turn_towards_human", "wave", "speak_greeting"],
                "conditions": ["human_detected", "face_to_face"],
                "duration": 10.0
            }
        }

    def plan(self, context: PlanningContext) -> Optional[Plan]:
        """
        Generate behavior plan based on current context
        """
        # Determine appropriate behavior based on context
        behavior = self._select_behavior(context)

        if not behavior:
            return None

        # Create plan steps for the behavior
        steps = []
        behavior_actions = self.behavior_library[behavior]["actions"]

        for i, action in enumerate(behavior_actions):
            step = PlanStep(
                action=action,
                parameters={"behavior": behavior, "step": i},
                duration=2.0,  # Average duration per action
                preconditions=self.behavior_library[behavior]["conditions"],
                effects=[f"action_{i}_completed"],
                priority=2
            )
            steps.append(step)

        return Plan(
            steps=steps,
            start_time=time.time(),
            estimated_duration=self.behavior_library[behavior]["duration"],
            confidence=0.85
        )

    def _select_behavior(self, context: PlanningContext) -> Optional[str]:
        """
        Select the most appropriate behavior based on context
        """
        current_state = context.current_state
        goal_state = context.goal_state

        # Priority-based behavior selection
        if current_state.get("obstacle_detected", False):
            return "avoid_obstacle"
        elif current_state.get("human_detected", False) and not current_state.get("human_greeted", False):
            return "greet_human"
        elif current_state.get("object_detected", False) and goal_state.get("action") == "pick":
            return "grasp_object"
        elif current_state.get("object_detected", False):
            return "approach_object"

        return None

class MotionPlanner(BasePlanner):
    """
    Motion planning for path generation and trajectory planning
    """
    def __init__(self):
        super().__init__(PlanningLevel.MOTION)

    def plan(self, context: PlanningContext) -> Optional[Plan]:
        """
        Generate motion plan (path and trajectory)
        """
        # This would interface with the path planning system from chapter 4
        from .chapter_04_simulated_execution import AStarPlanner

        # Get start and goal positions
        start_pos = context.current_state.get("position", [0, 0])
        goal_pos = context.goal_state.get("position", [10, 10])

        # Convert to grid coordinates if needed
        if isinstance(start_pos, (list, tuple)) and len(start_pos) >= 2:
            start_grid = (int(start_pos[0]), int(start_pos[1]))
            goal_grid = (int(goal_pos[0]), int(goal_pos[1]))
        else:
            start_grid = (0, 0)
            goal_grid = (10, 10)

        # Plan path using A* (simplified)
        try:
            # Create a simple grid for demonstration
            grid = np.zeros((20, 20))  # 20x20 grid
            # Add some obstacles
            grid[5:7, 5:15] = 1  # Vertical wall
            grid[10:15, 8:10] = 1  # Horizontal wall

            astar = AStarPlanner(grid)
            path = astar.plan_path(start_grid, goal_grid)

            if not path:
                return None

            # Convert path to plan steps
            steps = []
            for i, (x, y) in enumerate(path):
                step = PlanStep(
                    action="move_to_waypoint",
                    parameters={"x": x, "y": y, "waypoint_id": i},
                    duration=1.0,  # 1 second per waypoint
                    preconditions=[f"waypoint_{i-1}_reached"] if i > 0 else [],
                    effects=[f"waypoint_{i}_reached"],
                    priority=3
                )
                steps.append(step)

            return Plan(
                steps=steps,
                start_time=time.time(),
                estimated_duration=len(steps),
                confidence=0.9
            )
        except Exception as e:
            print(f"Motion planning error: {e}")
            return None

class ControlPlanner(BasePlanner):
    """
    Low-level control planning for motor commands
    """
    def __init__(self):
        super().__init__(PlanningLevel.CONTROL)

    def plan(self, context: PlanningContext) -> Optional[Plan]:
        """
        Generate low-level control commands
        """
        # For this example, we'll create a simple trajectory following plan
        steps = []

        # Get trajectory from higher-level planner
        trajectory = context.current_state.get("trajectory", [])
        if not trajectory:
            return None

        for i, waypoint in enumerate(trajectory):
            # Generate motor commands to reach the waypoint
            motor_commands = self._generate_motor_commands(waypoint)

            step = PlanStep(
                action="execute_motor_command",
                parameters={
                    "joint_positions": motor_commands,
                    "waypoint_id": i,
                    "duration": 0.1  # 100ms per control step
                },
                duration=0.1,
                preconditions=[f"waypoint_{i-1}_reached"] if i > 0 else [],
                effects=[f"waypoint_{i}_reached"],
                priority=4
            )
            steps.append(step)

        return Plan(
            steps=steps,
            start_time=time.time(),
            estimated_duration=len(steps) * 0.1,
            confidence=0.95
        )

    def _generate_motor_commands(self, waypoint: Dict[str, Any]) -> Dict[str, float]:
        """
        Generate motor commands to reach a waypoint
        """
        # This would interface with the robot's inverse kinematics
        # For simplicity, return a placeholder
        return {
            "joint_1": waypoint.get("x", 0) * 0.1,
            "joint_2": waypoint.get("y", 0) * 0.1,
            "joint_3": 0.0,  # Default position
            "gripper": 0.5  # Half open
        }

class HierarchicalPlanner:
    """
    Coordinated hierarchical planning system
    """
    def __init__(self, api_key: str):
        self.task_planner = TaskPlanner()
        self.behavior_planner = BehaviorPlanner()
        self.motion_planner = MotionPlanner()
        self.control_planner = ControlPlanner()

        # Plan cache for efficiency
        self.plan_cache = {}
        self.cache_timeout = 5.0  # seconds

        # Integration with VLA system
        self.vla_integrator = self._initialize_vla_integrator(api_key)

    def _initialize_vla_integrator(self, api_key: str):
        """
        Initialize the VLA system integrator
        """
        from .chapter_05_capstone import VLASystemIntegrator
        return VLASystemIntegrator(api_key=api_key)

    def plan(self, context: PlanningContext) -> Dict[PlanningLevel, Optional[Plan]]:
        """
        Generate plans at all levels of the hierarchy
        """
        plans = {}

        # Generate plans at each level
        plans[PlanningLevel.TASK] = self.task_planner.plan(context)
        plans[PlanningLevel.BEHAVIOR] = self.behavior_planner.plan(context)
        plans[PlanningLevel.MOTION] = self.motion_planner.plan(context)
        plans[PlanningLevel.CONTROL] = self.control_planner.plan(context)

        return plans

    def coordinate_plans(self, plans: Dict[PlanningLevel, Optional[Plan]],
                        context: PlanningContext) -> Plan:
        """
        Coordinate plans across different levels to create a unified execution plan
        """
        # This would implement plan coordination logic
        # For now, we'll create a simple coordinated plan

        # Start with the highest level plan (task) and refine with lower levels
        task_plan = plans.get(PlanningLevel.TASK)
        behavior_plan = plans.get(PlanningLevel.BEHAVIOR)
        motion_plan = plans.get(PlanningLevel.MOTION)
        control_plan = plans.get(PlanningLevel.CONTROL)

        # Create a unified plan by combining elements from all levels
        unified_steps = []

        # Add task-level steps
        if task_plan:
            for step in task_plan.steps:
                unified_steps.append(step)

        # Add behavior-level steps
        if behavior_plan:
            for step in behavior_plan.steps:
                unified_steps.append(step)

        # Add motion-level steps
        if motion_plan:
            for step in motion_plan.steps:
                unified_steps.append(step)

        # Add control-level steps
        if control_plan:
            for step in control_plan.steps:
                unified_steps.append(step)

        # Sort by priority and temporal order
        unified_steps.sort(key=lambda x: (x.priority, unified_steps.index(x)))

        return Plan(
            steps=unified_steps,
            start_time=time.time(),
            estimated_duration=sum(step.duration for step in unified_steps),
            confidence=min(
                plan.confidence for plan in plans.values()
                if plan is not None
            ) if any(plans.values()) else 0.5
        )

    def replan_if_needed(self, current_context: PlanningContext,
                        executed_steps: List[PlanStep]) -> Optional[Plan]:
        """
        Determine if replanning is needed and generate a new plan if so
        """
        # Check if the environment has changed significantly
        # or if the current plan is no longer valid

        # Simple condition: replan if more than 30 seconds have passed
        # or if a critical condition has changed
        if (time.time() - current_context.current_state.get("last_replan_time", 0) > 30.0 or
            current_context.current_state.get("replan_requested", False)):

            # Generate new plans
            new_plans = self.plan(current_context)
            new_plan = self.coordinate_plans(new_plans, current_context)

            # Update last replan time
            current_context.current_state["last_replan_time"] = time.time()

            return new_plan

        return None

    def execute_plan_with_monitoring(self, plan: Plan, execution_callback=None):
        """
        Execute a plan while monitoring progress and handling exceptions
        """
        for i, step in enumerate(plan.steps):
            print(f"Executing step {i+1}/{len(plan.steps)}: {step.action}")

            # Execute the step
            success = True
            if execution_callback:
                success = execution_callback(step)

            if not success:
                print(f"Step {i+1} failed: {step.action}")
                # Handle failure - could involve replanning or error recovery
                break

            # Check for interruption or cancellation
            if not self._check_execution_continuity():
                print("Execution interrupted")
                break

        print("Plan execution completed")

    def _check_execution_continuity(self) -> bool:
        """
        Check if execution should continue
        """
        # This would check for external interrupts, new goals, etc.
        return True

# Example usage
def example_execution_callback(step: PlanStep) -> bool:
    """
    Example execution callback
    """
    print(f"Executing: {step.action} with params {step.parameters}")
    time.sleep(step.duration * 0.1)  # Simulate execution time
    return True

# Usage example:
# context = PlanningContext(
#     current_state={
#         "position": [0, 0],
#         "battery_level": 0.8,
#         "object_detected": False,
#         "human_detected": False
#     },
#     goal_state={
#         "task": "bring_cup_cup",
#         "position": [5, 5]
#     },
#     environment_map=np.zeros((20, 20)),
#     robot_capabilities={
#         "max_speed": 1.0,
#         "manipulation_reachable": True
#     },
#     constraints={
#         "max_time": 300,  # 5 minutes
#         "safety_margin": 0.5  # 50cm safety distance
#     }
# )
#
# planner = HierarchicalPlanner("your-api-key")
# plans = planner.plan(context)
# unified_plan = planner.coordinate_plans(plans, context)
# planner.execute_plan_with_monitoring(unified_plan, example_execution_callback)
```

### Planning Algorithm Optimization

For efficient autonomous planning:

1. **Hierarchical Decomposition**: Break complex problems into manageable subproblems
2. **Reactive Planning**: Update plans based on new information during execution
3. **Plan Repair**: Modify existing plans rather than replanning from scratch
4. **Multi-objective Optimization**: Balance competing objectives like time, energy, and safety
5. **Uncertainty Handling**: Account for uncertain environment states and action outcomes

### Integration with Perception and Action Systems

The planning system integrates with perception and action systems through:

1. **State Estimation**: Using perception data to maintain accurate state estimates
2. **Action Selection**: Choosing appropriate actions based on current state and goals
3. **Plan Execution**: Coordinating with control systems to execute planned actions
4. **Feedback Integration**: Incorporating execution feedback to update plans

This hierarchical planning system enables complex autonomous behaviors by coordinating planning across multiple levels of abstraction while maintaining real-time responsiveness.

## Manipulation Capabilities

Implementing manipulation capabilities for complete robot autonomy.

### Overview of Manipulation Systems

Manipulation is a critical component of humanoid robot autonomy, enabling robots to interact with objects in their environment. Effective manipulation requires precise control of robotic arms, hands, and other end-effectors, combined with sophisticated perception and planning.

### Manipulation Architecture

A complete manipulation system consists of several key components:

```
[Perception] --> [Grasp Planning] --> [Trajectory Generation] --> [Control Execution] --> [Object Manipulation]
      ^              |                      |                          |                       |
      |              +----------------------+--------------------------+-----------------------+
      +-------------------------------------- Feedback Loop ----------------------------------+
```

### Implementation of Manipulation System

Here's an implementation of a manipulation system for humanoid robots:

```python
import numpy as np
from typing import Dict, Any, List, Optional, Tuple
from dataclasses import dataclass
import time
import math

@dataclass
class ManipulationState:
    """
    State of the manipulation system
    """
    arm_positions: Dict[str, np.ndarray]  # Joint positions for each arm
    end_effector_pose: Dict[str, np.ndarray]  # Position and orientation of end effectors
    gripper_state: Dict[str, float]  # Gripper opening (0.0-1.0)
    grasped_objects: List[str]  # Objects currently being held
    manipulation_mode: str  # "idle", "grasping", "holding", "releasing"

@dataclass
class GraspCandidate:
    """
    A potential grasp pose and associated properties
    """
    position: np.ndarray  # 3D position [x, y, z]
    orientation: np.ndarray  # 3D orientation [roll, pitch, yaw]
    grasp_type: str  # "power", "precision", "pinch", etc.
    stability_score: float  # How stable the grasp is expected to be
    approach_direction: np.ndarray  # Direction to approach the object

@dataclass
class ManipulationPlan:
    """
    A plan for manipulation actions
    """
    approach_trajectory: List[np.ndarray]  # Waypoints to approach object
    grasp_pose: np.ndarray  # Final grasp pose
    lift_trajectory: List[np.ndarray]  # Trajectory to lift object
    placement_trajectory: List[np.ndarray]  # Trajectory to place object
    gripper_commands: List[Tuple[float, float]]  # (gripper_opening, timestamp)

class ManipulationController:
    """
    Controller for manipulation actions
    """
    def __init__(self):
        self.state = ManipulationState(
            arm_positions={"left": np.zeros(7), "right": np.zeros(7)},
            end_effector_pose={"left": np.eye(4), "right": np.eye(4)},
            gripper_state={"left": 1.0, "right": 1.0},  # Fully open
            grasped_objects=[],
            manipulation_mode="idle"
        )

        # Manipulation parameters
        self.approach_distance = 0.1  # 10cm approach distance
        self.lift_height = 0.05  # 5cm lift after grasping
        self.safety_margin = 0.02  # 2cm safety margin

        # Inverse kinematics solver
        self.ik_solver = self._initialize_ik_solver()

    def _initialize_ik_solver(self):
        """
        Initialize inverse kinematics solver
        """
        # For this example, we'll use a simple pseudo-inverse Jacobian method
        # In practice, this would be a more sophisticated IK solver
        class SimpleIKSolver:
            def solve(self, target_pose: np.ndarray, current_joints: np.ndarray) -> Optional[np.ndarray]:
                """
                Solve inverse kinematics for target pose
                """
                # Simplified implementation - in reality this would be more complex
                # This is a placeholder that returns a modified version of current joints
                return current_joints + np.random.normal(0, 0.1, current_joints.shape)

        return SimpleIKSolver()

    def generate_grasp_candidates(self, object_info: Dict[str, Any]) -> List[GraspCandidate]:
        """
        Generate potential grasp poses for an object
        """
        candidates = []
        obj_position = np.array(object_info["position"])
        obj_dimensions = np.array(object_info["dimensions"])
        obj_class = object_info["class_name"]

        # Generate different grasp types based on object properties
        if obj_class in ["bottle", "cup", "mug"]:
            # Generate side grasp for cylindrical objects
            grasp_pos = obj_position + np.array([0, 0, obj_dimensions[2]/2])  # Side grasp at center height
            candidates.append(GraspCandidate(
                position=grasp_pos,
                orientation=self._calculate_side_grasp_orientation(obj_position, "cylindrical"),
                grasp_type="power",
                stability_score=0.8,
                approach_direction=np.array([-1, 0, 0])  # Approach from side
            ))

            # Top-down grasp for handles
            if object_info.get("has_handle", False):
                top_pos = obj_position + np.array([0, 0, obj_dimensions[2]/2 + 0.05])
                candidates.append(GraspCandidate(
                    position=top_pos,
                    orientation=self._calculate_top_grasp_orientation(),
                    grasp_type="precision",
                    stability_score=0.7,
                    approach_direction=np.array([0, 0, -1])  # Approach from above
                ))

        elif obj_class in ["book", "plate", "box"]:
            # Generate top-down grasp for flat objects
            top_pos = obj_position + np.array([0, 0, obj_dimensions[2]/2 + 0.02])
            candidates.append(GraspCandidate(
                position=top_pos,
                orientation=self._calculate_top_grasp_orientation(),
                grasp_type="power",
                stability_score=0.9,
                approach_direction=np.array([0, 0, -1])  # Approach from above
            ))

        elif obj_class in ["pen", "stick", "tube"]:
            # Generate multiple grasp points along the object
            num_grasps = 3
            for i in range(num_grasps):
                t = (i + 1) / (num_grasps + 1)  # Distribute along object
                grasp_pos = obj_position + np.array([t * obj_dimensions[0], 0, 0])
                candidates.append(GraspCandidate(
                    position=grasp_pos,
                    orientation=self._calculate_side_grasp_orientation(grasp_pos, "elongated"),
                    grasp_type="pinch",
                    stability_score=0.6,
                    approach_direction=np.array([-1, 0, 0])  # Approach from side
                ))

        else:
            # Default grasp - center of object
            candidates.append(GraspCandidate(
                position=obj_position,
                orientation=np.array([0, 0, 0]),  # Default orientation
                grasp_type="power",
                stability_score=0.5,
                approach_direction=np.array([0, 0, -1])  # Approach from above
            ))

        return candidates

    def _calculate_side_grasp_orientation(self, obj_position: np.ndarray, obj_type: str) -> np.ndarray:
        """
        Calculate appropriate orientation for side grasp
        """
        if obj_type == "cylindrical":
            # For cylindrical objects, align gripper perpendicular to cylinder axis
            return np.array([0, math.pi/2, 0])  # Rotate 90 degrees to grasp side
        elif obj_type == "elongated":
            # For elongated objects, grasp along the narrow axis
            return np.array([math.pi/2, 0, 0])  # Rotate to align with object axis
        else:
            return np.array([0, 0, 0])  # Default orientation

    def _calculate_top_grasp_orientation(self) -> np.ndarray:
        """
        Calculate appropriate orientation for top-down grasp
        """
        # Standard top-down grasp with gripper fingers pointing down
        return np.array([0, 0, 0])  # Identity orientation for top-down grasp

    def select_best_grasp(self, candidates: List[GraspCandidate],
                         robot_config: Dict[str, Any]) -> Optional[GraspCandidate]:
        """
        Select the best grasp candidate based on feasibility and stability
        """
        if not candidates:
            return None

        # Score candidates based on multiple factors
        scored_candidates = []
        for candidate in candidates:
            score = self._score_grasp_candidate(candidate, robot_config)
            scored_candidates.append((candidate, score))

        # Return the best scoring candidate
        best_candidate, best_score = max(scored_candidates, key=lambda x: x[1])
        return best_candidate

    def _score_grasp_candidate(self, candidate: GraspCandidate,
                              robot_config: Dict[str, Any]) -> float:
        """
        Score a grasp candidate based on multiple factors
        """
        # Base score from stability
        score = candidate.stability_score

        # Check reachability
        robot_position = np.array(robot_config["position"])
        distance = np.linalg.norm(candidate.position - robot_position[:3])
        if distance > robot_config.get("arm_reach", 1.0):
            return 0.0  # Not reachable

        # Prefer certain grasp types based on object
        if candidate.grasp_type == "power":
            score += 0.1
        elif candidate.grasp_type == "precision":
            score += 0.05

        # Check approach direction feasibility
        approach_feasible = self._check_approach_feasibility(candidate)
        if not approach_feasible:
            score *= 0.5  # Reduce score if approach is difficult

        return score

    def _check_approach_feasibility(self, candidate: GraspCandidate) -> bool:
        """
        Check if the approach direction is feasible
        """
        # Check if approach direction is blocked by environment
        # This would interface with the perception system to check for obstacles
        # For this example, assume approach is feasible
        return True

    def plan_manipulation(self, object_info: Dict[str, Any],
                         target_location: Optional[np.ndarray] = None) -> Optional[ManipulationPlan]:
        """
        Plan a complete manipulation sequence: approach, grasp, lift, place
        """
        # Generate grasp candidates
        candidates = self.generate_grasp_candidates(object_info)

        # Select best grasp
        best_grasp = self.select_best_grasp(candidates, {
            "position": self.state.end_effector_pose["right"][:3, 3],  # Current position
            "arm_reach": 1.0  # Example reach
        })

        if not best_grasp:
            return None

        # Plan approach trajectory
        approach_start = best_grasp.position + best_grasp.approach_direction * self.approach_distance
        approach_trajectory = self._plan_linear_trajectory(approach_start, best_grasp.position)

        # Plan lift trajectory (after grasping)
        lift_start = best_grasp.position
        lift_end = best_grasp.position + np.array([0, 0, self.lift_height])
        lift_trajectory = self._plan_linear_trajectory(lift_start, lift_end)

        # Plan placement trajectory if target location specified
        placement_trajectory = []
        if target_location is not None:
            place_start = target_location + np.array([0, 0, self.lift_height])  # Start lifted
            place_end = target_location
            placement_trajectory = self._plan_linear_trajectory(place_start, place_end)

        # Plan gripper commands
        gripper_commands = [
            (1.0, 0.0),  # Fully open at start
            (0.3, len(approach_trajectory) * 0.1),  # Close partially when approaching
            (0.0, len(approach_trajectory) * 0.1 + 0.5),  # Fully close to grasp
            (0.0, len(approach_trajectory) * 0.1 + len(lift_trajectory) * 0.1 + 0.5),  # Hold during lift
        ]

        if target_location is not None:
            # Open gripper at placement location
            release_time = (len(approach_trajectory) * 0.1 + len(lift_trajectory) * 0.1 +
                           len(placement_trajectory) * 0.1 + 0.5)
            gripper_commands.append((1.0, release_time))

        return ManipulationPlan(
            approach_trajectory=approach_trajectory,
            grasp_pose=np.concatenate([best_grasp.position, best_grasp.orientation]),
            lift_trajectory=lift_trajectory,
            placement_trajectory=placement_trajectory,
            gripper_commands=gripper_commands
        )

    def _plan_linear_trajectory(self, start: np.ndarray, end: np.ndarray,
                               num_waypoints: int = 10) -> List[np.ndarray]:
        """
        Plan a linear trajectory between two points
        """
        trajectory = []
        for i in range(num_waypoints + 1):
            t = i / num_waypoints
            point = start + t * (end - start)
            trajectory.append(point)
        return trajectory

    def execute_manipulation_plan(self, plan: ManipulationPlan,
                                 arm_name: str = "right") -> bool:
        """
        Execute a manipulation plan
        """
        success = True

        try:
            # Execute approach trajectory
            print(f"Executing approach trajectory with {len(plan.approach_trajectory)} waypoints")
            for i, waypoint in enumerate(plan.approach_trajectory):
                success = self._move_arm_to(waypoint, arm_name)
                if not success:
                    print(f"Failed to reach approach waypoint {i}")
                    return False
                time.sleep(0.1)  # Small delay between waypoints

            # Execute grasp
            print("Executing grasp")
            success = self._execute_grasp(plan.grasp_pose, arm_name)
            if not success:
                print("Grasp execution failed")
                return False

            # Execute lift trajectory
            print(f"Executing lift trajectory with {len(plan.lift_trajectory)} waypoints")
            for i, waypoint in enumerate(plan.lift_trajectory):
                success = self._move_arm_to(waypoint, arm_name)
                if not success:
                    print(f"Failed to reach lift waypoint {i}")
                    return False
                time.sleep(0.1)

            # Execute placement if specified
            if plan.placement_trajectory:
                print(f"Executing placement trajectory with {len(plan.placement_trajectory)} waypoints")
                for i, waypoint in enumerate(plan.placement_trajectory):
                    success = self._move_arm_to(waypoint, arm_name)
                    if not success:
                        print(f"Failed to reach placement waypoint {i}")
                        return False
                    time.sleep(0.1)

                # Release object
                print("Releasing object")
                success = self._release_object(arm_name)
                if not success:
                    print("Object release failed")
                    return False

        except Exception as e:
            print(f"Manipulation execution error: {e}")
            success = False

        return success

    def _move_arm_to(self, target_position: np.ndarray, arm_name: str) -> bool:
        """
        Move the specified arm to target position
        """
        try:
            # This would interface with the robot's joint controllers
            # For simulation, we'll just update the state
            current_joints = self.state.arm_positions[arm_name]

            # Solve inverse kinematics for target position
            target_joints = self.ik_solver.solve(
                self._position_to_pose(target_position),
                current_joints
            )

            if target_joints is not None:
                self.state.arm_positions[arm_name] = target_joints
                # Update end effector pose based on forward kinematics
                self.state.end_effector_pose[arm_name] = self._forward_kinematics(target_joints)
                return True
            else:
                return False

        except Exception as e:
            print(f"Arm movement error: {e}")
            return False

    def _position_to_pose(self, position: np.ndarray) -> np.ndarray:
        """
        Convert position to 4x4 transformation matrix
        """
        pose = np.eye(4)
        pose[:3, 3] = position[:3]  # Position
        if len(position) >= 6:  # If orientation is provided
            # Apply rotation based on orientation angles
            roll, pitch, yaw = position[3:6]
            pose[:3, :3] = self._euler_to_rotation_matrix(roll, pitch, yaw)
        return pose

    def _euler_to_rotation_matrix(self, roll: float, pitch: float, yaw: float) -> np.ndarray:
        """
        Convert Euler angles to rotation matrix
        """
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)

        rotation = np.array([
            [cp * cy, sr * sp * cy - cr * sy, cr * sp * cy + sr * sy],
            [cp * sy, sr * sp * sy + cr * cy, cr * sp * sy - sr * cy],
            [-sp, sr * cp, cr * cp]
        ])

        return rotation

    def _forward_kinematics(self, joint_angles: np.ndarray) -> np.ndarray:
        """
        Calculate end effector pose from joint angles (simplified)
        """
        # This is a simplified forward kinematics calculation
        # In reality, this would depend on the specific robot kinematic structure
        pose = np.eye(4)

        # Just return identity for this example
        # In practice, this would compute the actual transformation
        return pose

    def _execute_grasp(self, grasp_pose: np.ndarray, arm_name: str) -> bool:
        """
        Execute grasp at the specified pose
        """
        try:
            # Move to grasp pose
            success = self._move_arm_to(grasp_pose, arm_name)
            if not success:
                return False

            # Close gripper
            self.state.gripper_state[arm_name] = 0.0  # Fully closed
            time.sleep(0.5)  # Wait for grasp to complete

            # Add object to grasped objects list
            # In a real system, this would be determined by tactile sensors
            self.state.grasped_objects.append(f"object_{time.time()}")  # Temporary ID

            # Update manipulation mode
            self.state.manipulation_mode = "grasping"

            return True
        except Exception as e:
            print(f"Grasp execution error: {e}")
            return False

    def _release_object(self, arm_name: str) -> bool:
        """
        Release object from specified arm
        """
        try:
            # Open gripper
            self.state.gripper_state[arm_name] = 1.0  # Fully open
            time.sleep(0.5)  # Wait for release to complete

            # Remove object from grasped objects list
            if self.state.grasped_objects:
                self.state.grasped_objects.pop()

            # Update manipulation mode
            self.state.manipulation_mode = "idle"

            return True
        except Exception as e:
            print(f"Object release error: {e}")
            return False

class AutonomousManipulationSystem:
    """
    Complete autonomous manipulation system integrating perception, planning, and control
    """
    def __init__(self, api_key: str):
        self.manipulation_controller = ManipulationController()

        # Integration with other VLA components
        self.perception_system = self._initialize_perception_system()
        self.planning_system = self._initialize_planning_system(api_key)

        # State tracking
        self.current_task = None
        self.execution_history = []

    def _initialize_perception_system(self):
        """
        Initialize perception system for object detection and localization
        """
        from .chapter_04_simulated_execution import YOLOSimulator
        return YOLOSimulator()

    def _initialize_planning_system(self, api_key: str):
        """
        Initialize planning system for manipulation planning
        """
        from .chapter_05_capstone import HierarchicalPlanner
        return HierarchicalPlanner(api_key=api_key)

    def pick_object(self, object_name: str, arm: str = "right") -> bool:
        """
        Pick up an object with the specified arm
        """
        # Detect object in environment
        object_info = self._detect_object(object_name)
        if not object_info:
            print(f"Object '{object_name}' not found")
            return False

        # Plan manipulation
        manipulation_plan = self.manipulation_controller.plan_manipulation(object_info)
        if not manipulation_plan:
            print(f"Could not plan manipulation for object '{object_name}'")
            return False

        # Execute manipulation
        success = self.manipulation_controller.execute_manipulation_plan(manipulation_plan, arm)

        if success:
            print(f"Successfully picked up {object_name}")
            self.execution_history.append({
                "action": "pick",
                "object": object_name,
                "arm": arm,
                "timestamp": time.time(),
                "success": True
            })
        else:
            print(f"Failed to pick up {object_name}")
            self.execution_history.append({
                "action": "pick",
                "object": object_name,
                "arm": arm,
                "timestamp": time.time(),
                "success": False
            })

        return success

    def place_object(self, target_location: np.ndarray, arm: str = "right") -> bool:
        """
        Place currently held object at target location
        """
        if not self.manipulation_controller.state.grasped_objects:
            print("No object currently held")
            return False

        # Get current object info
        current_object = self.manipulation_controller.state.grasped_objects[-1]

        # Plan placement
        object_info = {
            "position": self.manipulation_controller.state.end_effector_pose[arm][:3, 3],
            "dimensions": np.array([0.1, 0.1, 0.1]),  # Placeholder
            "class_name": "held_object"
        }

        manipulation_plan = self.manipulation_controller.plan_manipulation(
            object_info, target_location
        )

        if not manipulation_plan:
            print("Could not plan placement")
            return False

        # Execute manipulation
        success = self.manipulation_controller.execute_manipulation_plan(manipulation_plan, arm)

        if success:
            print(f"Successfully placed object at {target_location}")
            self.execution_history.append({
                "action": "place",
                "target_location": target_location,
                "arm": arm,
                "timestamp": time.time(),
                "success": True
            })
        else:
            print("Failed to place object")
            self.execution_history.append({
                "action": "place",
                "target_location": target_location,
                "arm": arm,
                "timestamp": time.time(),
                "success": False
            })

        return success

    def _detect_object(self, object_name: str) -> Optional[Dict[str, Any]]:
        """
        Detect and localize an object in the environment
        """
        # This would interface with the perception system
        # For this example, return a placeholder
        return {
            "class_name": object_name,
            "position": np.array([0.5, 0.0, 0.0]),  # Example position
            "dimensions": np.array([0.1, 0.1, 0.2]),  # Example dimensions
            "orientation": np.array([0, 0, 0]),  # Example orientation
            "confidence": 0.9
        }

    def execute_complex_manipulation(self, task_description: str) -> bool:
        """
        Execute complex manipulation based on natural language description
        """
        # Use LLM to interpret task description and generate manipulation plan
        # This would interface with the LLM-based planning system
        print(f"Interpreting manipulation task: {task_description}")

        # For this example, we'll parse simple commands
        if "pick" in task_description.lower() and "place" in task_description.lower():
            # Extract object name and destination
            import re
            object_match = re.search(r"pick\s+(\w+)", task_description.lower())
            dest_match = re.search(r"place\s+on\s+(\w+)", task_description.lower())

            if object_match and dest_match:
                object_name = object_match.group(1)
                # For destination, we'll use a predefined location
                destination_map = {
                    "table": np.array([0.8, 0.2, 0.0]),
                    "counter": np.array([0.9, -0.1, 0.0]),
                    "shelf": np.array([0.7, 0.3, 0.8])
                }

                target_location = destination_map.get(dest_match.group(1), np.array([0.8, 0.0, 0.0]))

                # Execute pick and place
                pick_success = self.pick_object(object_name)
                if pick_success:
                    place_success = self.place_object(target_location)
                    return place_success
                return False
        elif "pick" in task_description.lower():
            # Extract object name
            import re
            object_match = re.search(r"pick\s+(\w+)", task_description.lower())
            if object_match:
                object_name = object_match.group(1)
                return self.pick_object(object_name)

        return False

# Example usage
def example_manipulation_callback(action: str, params: Dict[str, Any]) -> bool:
    """
    Example callback for manipulation actions
    """
    print(f"Executing manipulation: {action} with parameters: {params}")
    time.sleep(0.5)  # Simulate execution time
    return True

# Usage example:
# manipulator = AutonomousManipulationSystem("your-api-key")
# success = manipulator.execute_complex_manipulation("Pick up the red cup and place it on the table")
# print(f"Manipulation success: {success}")
```

### Manipulation System Integration

The manipulation system integrates with the broader VLA framework by:

1. **Perception Integration**: Using object detection and localization from the perception system
2. **Planning Coordination**: Working with the planning system to generate manipulation sequences
3. **Control Execution**: Executing low-level motor commands through the control system
4. **Language Understanding**: Interpreting manipulation commands from the language processing system

### Manipulation Safety and Validation

For safe manipulation operations:

1. **Force Limiting**: Limit forces applied during grasping to prevent damage
2. **Collision Avoidance**: Check trajectories for potential collisions
3. **Grasp Stability**: Verify grasp quality before lifting objects
4. **Workspace Limits**: Ensure all movements stay within safe workspace boundaries
5. **Emergency Stop**: Implement rapid stopping mechanisms for safety

This manipulation system provides humanoid robots with the capability to interact with objects in their environment, completing the full VLA loop by connecting language understanding and planning with physical action execution.

## Navigation

- [Previous: Chapter 04 - Simulated Robot Execution](./chapter-04-simulated-execution.md)
- [Next: Module Summary](./summary.md)