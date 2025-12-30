---
title: "Chapter 03: Cognitive Planning with LLMs"
description: "Using LLMs for cognitive planning to translate natural language commands to ROS 2 actions"
sidebar_label: "Chapter 03: Cognitive Planning with LLMs"
---

# Chapter 03: Cognitive Planning with LLMs

This chapter explores the use of Large Language Models (LLMs) for cognitive planning to translate natural language commands to ROS 2 actions.

## LLMs for Cognitive Planning

Large Language Models can serve as powerful cognitive engines for robotic systems, enabling sophisticated decision-making and planning capabilities.

### Role of LLMs in Cognitive Planning

Large Language Models play several crucial roles in cognitive planning for robotic systems:

1. **High-Level Task Decomposition**: Breaking complex tasks into sequences of executable subtasks
2. **Contextual Reasoning**: Understanding the current situation and making decisions based on context
3. **Knowledge Integration**: Incorporating external knowledge and common sense reasoning
4. **Plan Generation**: Creating detailed action plans based on goals and constraints
5. **Adaptive Planning**: Modifying plans based on changing conditions or unexpected events

### Cognitive Planning Architecture

A cognitive planning system using LLMs typically includes:

- **Goal Parser**: Interprets high-level goals from natural language
- **Context Integrator**: Incorporates current environment state and constraints
- **Plan Generator**: Creates detailed action sequences using LLM reasoning
- **Plan Validator**: Ensures generated plans are feasible and safe
- **Execution Monitor**: Tracks plan execution and handles deviations

### Implementation Example: LLM-Based Planner

```python
import openai
from typing import List, Dict, Any, Optional
from dataclasses import dataclass
import json

@dataclass
class PlanStep:
    action: str
    parameters: Dict[str, Any]
    preconditions: List[str]
    effects: List[str]

@dataclass
class CognitivePlan:
    steps: List[PlanStep]
    estimated_duration: float
    confidence: float

class LLMBasedPlanner:
    def __init__(self, api_key: str):
        self.client = openai.OpenAI(api_key=api_key)

    def generate_plan(self, goal: str, context: Dict[str, Any]) -> Optional[CognitivePlan]:
        """
        Generate a cognitive plan for achieving the specified goal
        """
        prompt = f"""
        Generate a detailed action plan to achieve the goal: "{goal}"

        Current context:
        {json.dumps(context, indent=2)}

        Respond with a JSON object containing:
        - steps: array of plan steps, each with:
          - action: the action to perform
          - parameters: parameters for the action
          - preconditions: conditions that must be true before executing
          - effects: effects of the action on the world state
        - estimated_duration: estimated time in seconds
        - confidence: confidence level (0-1)

        Ensure the plan is executable by a robotic system.
        """

        try:
            response = self.client.chat.completions.create(
                model="gpt-3.5-turbo",
                messages=[{"role": "user", "content": prompt}],
                temperature=0.1,
                response_format={"type": "json_object"}
            )

            plan_data = json.loads(response.choices[0].message.content)
            return self._parse_plan(plan_data)

        except Exception as e:
            print(f"Error generating plan: {e}")
            return None

    def _parse_plan(self, plan_data: Dict[str, Any]) -> CognitivePlan:
        """
        Parse the JSON plan data into a CognitivePlan object
        """
        steps = []
        for step_data in plan_data["steps"]:
            step = PlanStep(
                action=step_data["action"],
                parameters=step_data.get("parameters", {}),
                preconditions=step_data.get("preconditions", []),
                effects=step_data.get("effects", [])
            )
            steps.append(step)

        return CognitivePlan(
            steps=steps,
            estimated_duration=plan_data.get("estimated_duration", 0.0),
            confidence=plan_data.get("confidence", 0.5)
        )

    def refine_plan(self, plan: CognitivePlan, feedback: str) -> Optional[CognitivePlan]:
        """
        Refine an existing plan based on feedback
        """
        prompt = f"""
        Refine the following plan based on this feedback: "{feedback}"

        Original plan:
        {json.dumps({
            "steps": [
                {
                    "action": step.action,
                    "parameters": step.parameters,
                    "preconditions": step.preconditions,
                    "effects": step.effects
                } for step in plan.steps
            ],
            "estimated_duration": plan.estimated_duration,
            "confidence": plan.confidence
        }, indent=2)}

        Return an improved JSON plan in the same format.
        """

        try:
            response = self.client.chat.completions.create(
                model="gpt-3.5-turbo",
                messages=[{"role": "user", "content": prompt}],
                temperature=0.1,
                response_format={"type": "json_object"}
            )

            plan_data = json.loads(response.choices[0].message.content)
            return self._parse_plan(plan_data)

        except Exception as e:
            print(f"Error refining plan: {e}")
            return plan  # Return original plan if refinement fails
```

### Planning Strategies

Effective cognitive planning with LLMs employs several strategies:

1. **Hierarchical Planning**: Breaking complex tasks into hierarchical subtasks
2. **Symbolic Grounding**: Connecting abstract concepts to concrete robot capabilities
3. **Plan Repair**: Automatically fixing plan failures during execution
4. **Multi-Modal Integration**: Combining language, vision, and action information
5. **Uncertainty Handling**: Managing uncertainty in plan execution

### Integration with ROS 2

To integrate the LLM-based planner with ROS 2:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from action_msgs.msg import GoalStatus

class CognitivePlannerNode(Node):
    def __init__(self):
        super().__init__('cognitive_planner_node')

        # Initialize LLM-based planner
        self.planner = LLMBasedPlanner(api_key="your-api-key")

        # Publishers and subscribers
        self.plan_publisher = self.create_publisher(String, 'robot_plan', 10)
        self.goal_subscriber = self.create_subscription(
            String, 'high_level_goals', self.goal_callback, 10)

        # Service for plan refinement
        self.refine_service = self.create_service(
            String, 'refine_plan', self.refine_plan_callback)

    def goal_callback(self, msg):
        """
        Process high-level goal and generate plan
        """
        goal = msg.data
        context = self._get_current_context()

        plan = self.planner.generate_plan(goal, context)

        if plan and plan.confidence > 0.7:  # Only execute high-confidence plans
            self._publish_plan(plan)
        else:
            self.get_logger().warn(f"Low confidence plan for goal: {goal}")

    def _get_current_context(self) -> Dict[str, Any]:
        """
        Get current environment context for planning
        """
        # This would integrate with perception and state monitoring systems
        return {
            "robot_position": {"x": 0.0, "y": 0.0, "z": 0.0},
            "available_objects": [],
            "robot_state": "idle",
            "battery_level": 0.8,
            "last_known_locations": {}
        }

    def _publish_plan(self, plan: CognitivePlan):
        """
        Publish the cognitive plan to the execution system
        """
        plan_msg = String()
        plan_msg.data = json.dumps({
            "steps": [
                {
                    "action": step.action,
                    "parameters": step.parameters,
                    "preconditions": step.preconditions,
                    "effects": step.effects
                } for step in plan.steps
            ],
            "estimated_duration": plan.estimated_duration,
            "confidence": plan.confidence
        })

        self.plan_publisher.publish(plan_msg)
        self.get_logger().info(f"Published plan with {len(plan.steps)} steps")
```

This implementation demonstrates how LLMs can serve as cognitive engines that generate executable plans from high-level goals, incorporating contextual information and reasoning capabilities.

## Natural Language to ROS 2 Translation

Translating natural language commands to ROS 2 actions requires understanding both linguistic intent and robotic capabilities.

### Translation Architecture

The translation process involves multiple components working together:

1. **Natural Language Parser**: Interprets the linguistic structure and meaning
2. **Intent Extractor**: Identifies the user's intended action
3. **Entity Recognizer**: Identifies objects, locations, and parameters
4. **Action Mapper**: Maps to specific ROS 2 actions and services
5. **Parameter Validator**: Ensures parameters are valid for the robot
6. **Safety Checker**: Verifies commands are safe to execute

### ROS 2 Action Mapping Example

Here's an implementation that maps natural language to ROS 2 actions:

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class NaturalLanguageToROS2Mapper(Node):
    def __init__(self):
        super().__init__('nl_to_ros2_mapper')

        # Action clients for different robot capabilities
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Publishers for other actions
        self.joint_publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscriber for natural language commands
        self.nl_subscriber = self.create_subscription(
            String, 'natural_language_commands', self.nl_command_callback, 10)

    def nl_command_callback(self, msg):
        """
        Process natural language command and translate to ROS 2 actions
        """
        command = msg.data
        self.get_logger().info(f'Received NL command: {command}')

        # Parse the command
        action_request = self.parse_natural_language(command)

        if action_request:
            self.execute_action_request(action_request)
        else:
            self.get_logger().warn(f'Could not parse command: {command}')

    def parse_natural_language(self, command: str) -> Optional[Dict[str, Any]]:
        """
        Parse natural language command and return action request
        """
        command_lower = command.lower().strip()

        # Movement commands
        if any(word in command_lower for word in ['go to', 'navigate to', 'move to', 'walk to']):
            location = self.extract_location(command_lower)
            if location:
                return {
                    'action_type': 'navigation',
                    'target_location': location
                }

        # Manipulation commands
        elif any(word in command_lower for word in ['pick', 'grasp', 'take', 'grab']):
            obj = self.extract_object(command_lower)
            if obj:
                return {
                    'action_type': 'manipulation',
                    'operation': 'pick',
                    'target_object': obj
                }

        # Movement direction commands
        elif any(word in command_lower for word in ['move', 'go', 'turn']):
            direction = self.extract_direction(command_lower)
            distance = self.extract_distance(command_lower)
            if direction:
                return {
                    'action_type': 'motion',
                    'direction': direction,
                    'distance': distance
                }

        # Simple commands
        elif 'stop' in command_lower:
            return {
                'action_type': 'stop'
            }
        elif 'sit' in command_lower:
            return {
                'action_type': 'posture',
                'posture': 'sit'
            }
        elif 'stand' in command_lower:
            return {
                'action_type': 'posture',
                'posture': 'stand'
            }

        return None

    def extract_location(self, command: str) -> Optional[Dict[str, float]]:
        """
        Extract location from natural language command
        """
        # This would typically use more sophisticated NLP
        # For this example, we'll use simple pattern matching

        location_keywords = {
            'kitchen': {'x': 5.0, 'y': 3.0, 'z': 0.0},
            'living room': {'x': 2.0, 'y': 1.0, 'z': 0.0},
            'bedroom': {'x': 8.0, 'y': 4.0, 'z': 0.0},
            'office': {'x': 1.0, 'y': 5.0, 'z': 0.0},
            'dining room': {'x': 4.0, 'y': 2.0, 'z': 0.0}
        }

        for location, coordinates in location_keywords.items():
            if location in command:
                return coordinates

        return None

    def extract_object(self, command: str) -> Optional[str]:
        """
        Extract object from natural language command
        """
        # Simple object extraction
        objects = ['cup', 'book', 'ball', 'box', 'bottle', 'phone', 'keys']

        for obj in objects:
            if obj in command:
                return obj

        return None

    def extract_direction(self, command: str) -> Optional[str]:
        """
        Extract movement direction from command
        """
        if 'forward' in command or 'ahead' in command:
            return 'forward'
        elif 'backward' in command or 'back' in command:
            return 'backward'
        elif 'left' in command:
            return 'left'
        elif 'right' in command:
            return 'right'
        elif 'turn left' in command:
            return 'turn_left'
        elif 'turn right' in command:
            return 'turn_right'

        return None

    def extract_distance(self, command: str) -> float:
        """
        Extract distance from command
        """
        import re
        # Look for number patterns followed by distance units
        pattern = r"(\d+(?:\.\d+)?)\s*(meters?|m|feet|ft|steps?)"
        match = re.search(pattern, command)

        if match:
            value = float(match.group(1))
            unit = match.group(2)

            if unit in ["feet", "ft"]:
                return value * 0.3048
            elif unit in ["steps"]:
                return value * 0.75  # average step is 0.75 meters
            else:
                return value

        return 1.0  # default distance

    def execute_action_request(self, request: Dict[str, Any]):
        """
        Execute the translated action request
        """
        action_type = request['action_type']

        if action_type == 'navigation':
            self.execute_navigation(request['target_location'])
        elif action_type == 'manipulation':
            self.execute_manipulation(request)
        elif action_type == 'motion':
            self.execute_motion(request)
        elif action_type == 'stop':
            self.execute_stop()
        elif action_type == 'posture':
            self.execute_posture(request['posture'])

    def execute_navigation(self, location: Dict[str, float]):
        """
        Execute navigation to specified location
        """
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.pose.position.x = location['x']
        goal_msg.pose.pose.position.y = location['y']
        goal_msg.pose.pose.position.z = location['z']
        goal_msg.pose.pose.orientation.w = 1.0  # Default orientation

        # Wait for action server
        self.nav_client.wait_for_server()

        # Send goal
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.navigation_done_callback)

    def navigation_done_callback(self, future):
        """
        Callback for navigation completion
        """
        result = future.result()
        if result:
            self.get_logger().info('Navigation completed successfully')
        else:
            self.get_logger().error('Navigation failed')

    def execute_motion(self, motion_params: Dict[str, Any]):
        """
        Execute motion command based on direction and distance
        """
        from geometry_msgs.msg import Twist

        twist_msg = Twist()

        direction = motion_params['direction']
        distance = motion_params.get('distance', 1.0)

        # Set velocity based on direction
        if direction == 'forward':
            twist_msg.linear.x = 0.5
        elif direction == 'backward':
            twist_msg.linear.x = -0.5
        elif direction == 'left':
            twist_msg.angular.z = 0.5
        elif direction == 'right':
            twist_msg.angular.z = -0.5
        elif direction == 'turn_left':
            twist_msg.angular.z = 0.5
        elif direction == 'turn_right':
            twist_msg.angular.z = -0.5

        # Calculate duration based on distance and velocity
        duration = distance / 0.5  # assuming 0.5 m/s speed

        # Execute for calculated duration
        self.execute_timed_motion(twist_msg, duration)

    def execute_timed_motion(self, twist_msg, duration):
        """
        Execute motion command for a specified duration
        """
        import time
        start_time = time.time()

        while time.time() - start_time < duration:
            self.cmd_vel_publisher.publish(twist_msg)
            rclpy.spin_once(self, timeout_sec=0.1)

        # Stop the robot after motion
        stop_msg = Twist()
        self.cmd_vel_publisher.publish(stop_msg)

    def execute_stop(self):
        """
        Stop all robot motion
        """
        from geometry_msgs.msg import Twist
        stop_msg = Twist()
        self.cmd_vel_publisher.publish(stop_msg)
        self.get_logger().info('Robot stopped')

    def execute_posture(self, posture: str):
        """
        Execute posture change (e.g., sit, stand)
        """
        # This would send joint trajectory commands to achieve the posture
        joint_msg = JointState()
        joint_msg.name = ['joint1', 'joint2', 'joint3']  # Example joint names

        if posture == 'sit':
            # Example joint positions for sitting
            joint_msg.position = [0.0, -1.0, 0.5]
        elif posture == 'stand':
            # Example joint positions for standing
            joint_msg.position = [0.0, 0.0, 0.0]

        self.joint_publisher.publish(joint_msg)
        self.get_logger().info(f'Robot posture changed to {posture}')
```

### Translation Strategies

Effective natural language to ROS 2 translation employs these strategies:

1. **Pattern Matching**: Using predefined patterns for common commands
2. **Entity Recognition**: Identifying objects, locations, and parameters
3. **Context Awareness**: Using environment context to disambiguate commands
4. **Fallback Mechanisms**: Providing alternatives when exact matches fail
5. **Validation**: Ensuring translated commands are valid and safe

### Integration with LLMs

For more sophisticated translation, LLMs can be integrated:

```python
class AdvancedNLRos2Mapper(NaturalLanguageToROS2Mapper):
    def __init__(self, api_key: str):
        super().__init__()
        self.llm_client = openai.OpenAI(api_key=api_key)

    def parse_natural_language_with_llm(self, command: str, context: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """
        Use LLM to parse natural language command with context
        """
        prompt = f"""
        Parse the following natural language command: "{command}"

        Robot context: {context}

        Return a JSON object with:
        - action_type: navigation, manipulation, motion, etc.
        - parameters: specific parameters for the action
        - target: target object or location if applicable

        If the command cannot be parsed, return null.
        """

        try:
            response = self.llm_client.chat.completions.create(
                model="gpt-3.5-turbo",
                messages=[{"role": "user", "content": prompt}],
                temperature=0.1,
                response_format={"type": "json_object"}
            )

            result = response.choices[0].message.content
            if result.lower() != 'null':
                return json.loads(result)
        except Exception as e:
            self.get_logger().error(f"LLM parsing failed: {e}")

        return None
```

This approach combines rule-based parsing with LLM-based understanding for more robust natural language to ROS 2 translation.

## Intelligent Decision-Making Systems

Creating intelligent decision-making systems involves combining LLM reasoning with robotic action execution.

### Architecture of Intelligent Decision-Making Systems

An intelligent decision-making system for humanoid robots typically includes:

1. **Perception Module**: Processes sensory input (vision, audio, tactile)
2. **State Estimator**: Maintains internal model of the world state
3. **Goal Manager**: Tracks current goals and objectives
4. **Reasoning Engine**: Uses LLMs for high-level reasoning and planning
5. **Action Selector**: Chooses appropriate actions based on reasoning
6. **Execution Monitor**: Tracks action execution and updates world state
7. **Learning Component**: Adapts behavior based on experience

### Implementation Example: Decision-Making System

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import PoseStamped
import json
from typing import Dict, Any, List, Optional

class IntelligentDecisionMaker(Node):
    def __init__(self, api_key: str):
        super().__init__('intelligent_decision_maker')

        # Initialize LLM client
        self.llm_client = openai.OpenAI(api_key=api_key)

        # Internal state
        self.current_state = {
            "robot_position": {"x": 0.0, "y": 0.0, "z": 0.0},
            "battery_level": 1.0,
            "current_goal": None,
            "detected_objects": [],
            "known_locations": {},
            "robot_status": "idle"
        }

        # Publishers and subscribers
        self.action_publisher = self.create_publisher(String, 'robot_actions', 10)
        self.state_subscriber = self.create_subscription(
            String, 'world_state', self.state_callback, 10)
        self.goal_subscriber = self.create_subscription(
            String, 'high_level_goals', self.goal_callback, 10)
        self.perception_subscribers = [
            self.create_subscription(Image, 'camera/image_raw', self.image_callback, 10),
            self.create_subscription(LaserScan, 'scan', self.laser_callback, 10)
        ]

        # Timer for continuous decision making
        self.decision_timer = self.create_timer(1.0, self.make_decision)

    def state_callback(self, msg):
        """
        Update internal state from world state
        """
        try:
            state_update = json.loads(msg.data)
            self.current_state.update(state_update)
        except json.JSONDecodeError:
            self.get_logger().error('Invalid state message')

    def goal_callback(self, msg):
        """
        Set new goal for the decision maker
        """
        self.current_state['current_goal'] = msg.data
        self.get_logger().info(f'New goal set: {msg.data}')

    def image_callback(self, msg):
        """
        Process image data for object detection
        """
        # In a real implementation, this would use computer vision
        # For this example, we'll just update detected objects
        pass

    def laser_callback(self, msg):
        """
        Process laser scan data for navigation
        """
        # Process laser data to update state
        self.current_state['obstacle_distance'] = min(msg.ranges)

    def make_decision(self):
        """
        Main decision-making loop
        """
        if not self.current_state['current_goal']:
            return  # No goal to pursue

        # Generate decision using LLM
        decision = self.generate_decision_with_llm()

        if decision:
            self.execute_decision(decision)

    def generate_decision_with_llm(self) -> Optional[Dict[str, Any]]:
        """
        Use LLM to generate a decision based on current state and goal
        """
        prompt = f"""
        Current goal: {self.current_state['current_goal']}

        Current state: {json.dumps(self.current_state, indent=2)}

        Available actions: move_to_location, pick_object, place_object,
        navigate_around_obstacle, charge_battery, report_status

        Generate the next action to take. Return as JSON with:
        - action: the action to perform
        - parameters: parameters for the action
        - reasoning: brief explanation of why this action was chosen
        - confidence: confidence level (0-1)
        """

        try:
            response = self.llm_client.chat.completions.create(
                model="gpt-3.5-turbo",
                messages=[{"role": "user", "content": prompt}],
                temperature=0.1,
                response_format={"type": "json_object"}
            )

            decision = json.loads(response.choices[0].message.content)

            # Validate confidence level
            if decision.get('confidence', 0) < 0.6:
                self.get_logger().warn(f'Low confidence decision: {decision}')
                return None

            return decision

        except Exception as e:
            self.get_logger().error(f'LLM decision generation failed: {e}')
            return None

    def execute_decision(self, decision: Dict[str, Any]):
        """
        Execute the decision generated by the LLM
        """
        action = decision['action']
        parameters = decision.get('parameters', {})

        self.get_logger().info(f'Executing action: {action} with params: {parameters}')

        # Publish action to robot execution system
        action_msg = String()
        action_msg.data = json.dumps({
            'action': action,
            'parameters': parameters,
            'reasoning': decision.get('reasoning', ''),
            'confidence': decision.get('confidence', 0.8)
        })

        self.action_publisher.publish(action_msg)

    def update_known_locations(self, location_name: str, pose: Dict[str, float]):
        """
        Update the map of known locations
        """
        self.current_state['known_locations'][location_name] = pose
        self.get_logger().info(f'Updated location: {location_name} at {pose}')

class DecisionMakingSystemManager:
    def __init__(self):
        self.decision_maker = None
        self.is_running = False

    def start_system(self, api_key: str):
        """
        Start the decision making system
        """
        rclpy.init()
        self.decision_maker = IntelligentDecisionMaker(api_key)
        self.is_running = True

        try:
            rclpy.spin(self.decision_maker)
        except KeyboardInterrupt:
            pass
        finally:
            self.stop_system()

    def stop_system(self):
        """
        Stop the decision making system
        """
        if self.decision_maker:
            self.decision_maker.destroy_node()
        rclpy.shutdown()
        self.is_running = False
```

### Decision-Making Strategies

Effective intelligent decision-making systems employ several strategies:

1. **Hierarchical Decision Making**: Breaking complex decisions into hierarchical sub-decisions
2. **Uncertainty Management**: Handling uncertain or incomplete information
3. **Multi-Objective Optimization**: Balancing competing goals and constraints
4. **Learning from Experience**: Adapting decision strategies based on outcomes
5. **Collaborative Decision Making**: Coordinating with other agents or humans

### Safety and Validation

When implementing intelligent decision-making systems:

1. **Constraint Checking**: Ensure decisions satisfy safety constraints
2. **Plan Validation**: Verify that action sequences are feasible
3. **Fallback Mechanisms**: Provide safe alternatives when primary decisions fail
4. **Human Oversight**: Allow human intervention when needed
5. **Monitoring and Logging**: Track decision-making processes for debugging

This approach creates a robust decision-making system that can handle complex tasks by combining LLM reasoning with real-time robot control and environmental awareness.

## Navigation

- [Previous: Chapter 02 - Voice-to-Action Integration](./chapter-02-voice-to-action.md)
- [Next: Chapter 04 - Simulated Robot Execution](./chapter-04-simulated-execution.md)