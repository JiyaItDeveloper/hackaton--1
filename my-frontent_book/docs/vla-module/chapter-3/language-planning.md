---
sidebar_position: 4
---
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

# Chapter 3: Language-Driven Cognitive Planning

## Learning Goals
- Use Large Language Models (LLMs) to convert natural language into action plans
- Decompose complex tasks into ROS 2 actions
- Implement safety and grounding mechanisms for robot capabilities

## Introduction to Language-Driven Cognitive Planning

Language-driven cognitive planning bridges the gap between high-level natural language commands and low-level robot actions. This chapter explores how Large Language Models (LLMs) can be used to interpret human instructions and generate executable action sequences for humanoid robots.

### The Cognitive Planning Pipeline

The cognitive planning process involves:
1. **Language Understanding**: Interpreting natural language commands
2. **Task Decomposition**: Breaking down complex tasks into subtasks
3. **Action Sequencing**: Ordering actions in executable sequences
4. **Capability Verification**: Ensuring actions are within robot capabilities
5. **Safety Validation**: Checking for safety constraints

## Using LLMs for Natural Language to Action Conversion

### LLM Integration Architecture

```python
import openai
import json
from typing import List, Dict, Any

class LLMPlanner:
    def __init__(self, api_key: str, model: str = "gpt-4"):
        openai.api_key = api_key
        self.model = model
        self.robot_capabilities = self.load_robot_capabilities()

    def load_robot_capabilities(self) -> Dict:
        # Define what the robot can and cannot do
        return {
            "navigation": {
                "movable": True,
                "max_distance": 10.0,  # meters
                "locations": ["kitchen", "living room", "bedroom", "office"]
            },
            "manipulation": {
                "graspable": True,
                "max_weight": 2.0,  # kg
                "objects": ["cup", "book", "bottle", "phone", "keys"]
            },
            "interaction": {
                "speak": True,
                "listen": True,
                "greet": True
            }
        }

    def generate_action_plan(self, command: str, environment_context: Dict = None) -> Dict:
        # Create a prompt for the LLM
        prompt = self.create_planning_prompt(command, environment_context)

        response = openai.ChatCompletion.create(
            model=self.model,
            messages=[
                {"role": "system", "content": self.get_system_prompt()},
                {"role": "user", "content": prompt}
            ],
            temperature=0.1,
            max_tokens=1000,
            functions=[
                {
                    "name": "create_action_plan",
                    "description": "Create a sequence of actions for the robot to execute",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "actions": {
                                "type": "array",
                                "items": {
                                    "type": "object",
                                    "properties": {
                                        "action_type": {"type": "string", "enum": ["navigation", "manipulation", "interaction", "wait", "speak"]},
                                        "parameters": {"type": "object"},
                                        "description": {"type": "string"}
                                    },
                                    "required": ["action_type", "parameters", "description"]
                                }
                            },
                            "reasoning": {"type": "string"}
                        },
                        "required": ["actions", "reasoning"]
                    }
                }
            ],
            function_call={"name": "create_action_plan"}
        )

        # Parse the function call result
        function_call = response.choices[0].message.function_call
        result = json.loads(function_call.arguments)

        return result

    def create_planning_prompt(self, command: str, environment_context: Dict = None) -> str:
        context = environment_context or {}

        prompt = f"""
        Command: "{command}"

        Robot Capabilities:
        {json.dumps(self.robot_capabilities, indent=2)}

        Current Environment Context:
        {json.dumps(context, indent=2)}

        Please create a detailed action plan to execute this command. Consider:
        1. The robot's capabilities and limitations
        2. Safety constraints
        3. Environmental context
        4. Feasibility of each action
        5. Proper sequence of operations
        """

        return prompt

    def get_system_prompt(self) -> str:
        return """
        You are an expert robotic action planner. Your role is to convert natural language commands into detailed, executable action plans for a humanoid robot. Each action should be specific, safe, and within the robot's capabilities. Prioritize safety and feasibility in your planning.
        """
```

### Example Usage

```python
# Initialize the planner
planner = LLMPlanner(api_key="your-api-key")

# Example command
command = "Go to the kitchen and bring me a cup of water"

# Environment context (from perception system)
environment_context = {
    "kitchen_objects": ["cup", "bottle", "fridge", "table"],
    "robot_position": "living room",
    "obstacles": [],
    "time_of_day": "afternoon"
}

# Generate action plan
action_plan = planner.generate_action_plan(command, environment_context)
print(json.dumps(action_plan, indent=2))
```

## Task Decomposition into ROS 2 Actions

### Action Planning Framework

```python
from enum import Enum
from dataclasses import dataclass
from typing import List, Dict, Any
import rospy
from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped

class ActionType(Enum):
    NAVIGATION = "navigation"
    MANIPULATION = "manipulation"
    INTERACTION = "interaction"
    SPEAK = "speak"
    WAIT = "wait"

@dataclass
class ActionStep:
    action_type: ActionType
    parameters: Dict[str, Any]
    description: str
    priority: int = 0

class ActionExecutor:
    def __init__(self):
        # Initialize ROS clients
        self.nav_client = SimpleActionClient('move_base', MoveBaseAction)
        self.nav_client.wait_for_server()

        # Publishers for other actions
        self.speech_pub = rospy.Publisher('/tts_input', String, queue_size=10)
        self.manipulation_pub = rospy.Publisher('/manipulation_command', String, queue_size=10)

    def execute_action(self, action_step: ActionStep) -> bool:
        """Execute a single action step"""
        try:
            if action_step.action_type == ActionType.NAVIGATION:
                return self.execute_navigation(action_step.parameters)
            elif action_step.action_type == ActionType.MANIPULATION:
                return self.execute_manipulation(action_step.parameters)
            elif action_step.action_type == ActionType.SPEAK:
                return self.execute_speak(action_step.parameters)
            elif action_step.action_type == ActionType.WAIT:
                return self.execute_wait(action_step.parameters)
            elif action_step.action_type == ActionType.INTERACTION:
                return self.execute_interaction(action_step.parameters)
            else:
                rospy.logerr(f"Unknown action type: {action_step.action_type}")
                return False
        except Exception as e:
            rospy.logerr(f"Error executing action: {e}")
            return False

    def execute_navigation(self, params: Dict) -> bool:
        """Execute navigation action"""
        goal = MoveBaseGoal()

        # Set goal position (example)
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = params.get('x', 0.0)
        goal.target_pose.pose.position.y = params.get('y', 0.0)
        goal.target_pose.pose.position.z = 0.0
        goal.target_pose.pose.orientation.w = 1.0  # No rotation

        # Send goal
        self.nav_client.send_goal(goal)

        # Wait for result
        finished_within_time = self.nav_client.wait_for_result(rospy.Duration(30))

        if not finished_within_time:
            rospy.logerr("Navigation action timed out")
            return False

        result = self.nav_client.get_result()
        return result is not None

    def execute_manipulation(self, params: Dict) -> bool:
        """Execute manipulation action"""
        command = json.dumps({
            'action': params.get('action', 'grasp'),
            'object': params.get('object', ''),
            'position': params.get('position', {})
        })

        msg = String()
        msg.data = command
        self.manipulation_pub.publish(msg)

        # Wait for completion (simplified)
        rospy.sleep(5.0)
        return True

    def execute_speak(self, params: Dict) -> bool:
        """Execute speech action"""
        text = params.get('text', '')
        msg = String()
        msg.data = text
        self.speech_pub.publish(msg)
        return True

    def execute_wait(self, params: Dict) -> bool:
        """Execute wait action"""
        duration = params.get('duration', 1.0)
        rospy.sleep(duration)
        return True

    def execute_interaction(self, params: Dict) -> bool:
        """Execute interaction action"""
        interaction_type = params.get('type', 'greet')
        if interaction_type == 'greet':
            return self.execute_speak({'text': 'Hello! How can I help you?'})
        # Add more interaction types as needed
        return False
```

### ROS 2 Action Client (Alternative Implementation)

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String

class ROS2ActionExecutor(Node):
    def __init__(self):
        super().__init__('action_executor')

        # Action clients
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Publishers
        self.speech_pub = self.create_publisher(String, '/tts_input', 10)
        self.manipulation_pub = self.create_publisher(String, '/manipulation_command', 10)

    def execute_navigation_ros2(self, x: float, y: float, theta: float = 0.0) -> bool:
        """Execute navigation using ROS 2 action"""
        # Wait for action server
        self.nav_client.wait_for_server()

        # Create goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        # Convert theta to quaternion
        import math
        goal_msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(theta / 2.0)

        # Send goal
        goal_future = self.nav_client.send_goal_async(goal_msg)

        # Wait for result
        rclpy.spin_until_future_complete(self, goal_future)

        goal_handle = goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        return result is not None
```

## Safety and Grounding in Robot Capabilities

### Capability Validation System

```python
class CapabilityValidator:
    def __init__(self, robot_capabilities: Dict):
        self.capabilities = robot_capabilities

    def validate_action(self, action_step: ActionStep) -> Dict:
        """Validate if an action is feasible given robot capabilities"""
        validation_result = {
            'is_valid': True,
            'errors': [],
            'warnings': [],
            'suggested_alternatives': []
        }

        if action_step.action_type == ActionType.NAVIGATION:
            validation_result = self.validate_navigation(action_step.parameters)
        elif action_step.action_type == ActionType.MANIPULATION:
            validation_result = self.validate_manipulation(action_step.parameters)
        elif action_step.action_type == ActionType.SPEAK:
            validation_result = self.validate_speech(action_step.parameters)

        return validation_result

    def validate_navigation(self, params: Dict) -> Dict:
        """Validate navigation action"""
        result = {
            'is_valid': True,
            'errors': [],
            'warnings': [],
            'suggested_alternatives': []
        }

        # Check if location is in known locations
        target_location = params.get('location')
        if target_location and target_location not in self.capabilities['navigation']['locations']:
            result['is_valid'] = False
            result['errors'].append(f"Unknown location: {target_location}")
            result['suggested_alternatives'] = self.capabilities['navigation']['locations']

        # Check distance constraint
        distance = params.get('distance', 0)
        max_distance = self.capabilities['navigation']['max_distance']
        if distance > max_distance:
            result['is_valid'] = False
            result['errors'].append(f"Distance {distance}m exceeds maximum {max_distance}m")

        return result

    def validate_manipulation(self, params: Dict) -> Dict:
        """Validate manipulation action"""
        result = {
            'is_valid': True,
            'errors': [],
            'warnings': [],
            'suggested_alternatives': []
        }

        # Check if object is graspable
        obj = params.get('object')
        if obj and obj not in self.capabilities['manipulation']['objects']:
            result['is_valid'] = False
            result['errors'].append(f"Unknown object: {obj}")
            result['suggested_alternatives'] = self.capabilities['manipulation']['objects']

        # Check weight constraint
        weight = params.get('weight', 0)
        max_weight = self.capabilities['manipulation']['max_weight']
        if weight > max_weight:
            result['is_valid'] = False
            result['errors'].append(f"Weight {weight}kg exceeds maximum {max_weight}kg")

        return result

    def validate_speech(self, params: Dict) -> Dict:
        """Validate speech action"""
        result = {
            'is_valid': True,
            'errors': [],
            'warnings': [],
            'suggested_alternatives': []
        }

        # Check if robot has speech capability
        if not self.capabilities['interaction']['speak']:
            result['is_valid'] = False
            result['errors'].append("Robot does not have speech capability")

        return result
```

### Safety Constraint Checker

```python
class SafetyConstraintChecker:
    def __init__(self):
        self.safety_rules = self.load_safety_rules()

    def load_safety_rules(self) -> Dict:
        """Load safety rules for the robot"""
        return {
            'no_go_zones': ['stairs', 'cliff', 'pool'],
            'speed_limits': {
                'indoor': 0.5,  # m/s
                'outdoor': 1.0,  # m/s
                'near_people': 0.2  # m/s
            },
            'minimum_distance_to_people': 0.5,  # meters
            'maximum_payload': 2.0,  # kg
            'timeouts': {
                'navigation': 30,  # seconds
                'manipulation': 60,  # seconds
                'interaction': 10   # seconds
            }
        }

    def check_action_safety(self, action_step: ActionStep, environment: Dict) -> Dict:
        """Check if an action is safe to execute in the current environment"""
        safety_check = {
            'is_safe': True,
            'violations': [],
            'suggested_safeguards': []
        }

        # Check for no-go zones in navigation
        if action_step.action_type == ActionType.NAVIGATION:
            target_location = action_step.parameters.get('location', '').lower()
            if any(zone in target_location for zone in self.safety_rules['no_go_zones']):
                safety_check['is_safe'] = False
                safety_check['violations'].append(f"Target location {target_location} is in a no-go zone")

        # Check distance to people for navigation
        if action_step.action_type == ActionType.NAVIGATION:
            people_nearby = environment.get('people_nearby', [])
            min_distance = self.safety_rules['minimum_distance_to_people']

            for person in people_nearby:
                distance = self.calculate_distance(
                    action_step.parameters.get('x', 0),
                    action_step.parameters.get('y', 0),
                    person.get('x', 0),
                    person.get('y', 0)
                )

                if distance < min_distance:
                    safety_check['is_safe'] = False
                    safety_check['violations'].append(f"Target too close to person (distance: {distance:.2f}m)")

        # Check payload for manipulation
        if action_step.action_type == ActionType.MANIPULATION:
            payload = action_step.parameters.get('weight', 0)
            max_payload = self.safety_rules['maximum_payload']

            if payload > max_payload:
                safety_check['is_safe'] = False
                safety_check['violations'].append(f"Payload {payload}kg exceeds maximum {max_payload}kg")

        return safety_check

    def calculate_distance(self, x1, y1, x2, y2):
        """Calculate Euclidean distance between two points"""
        return ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5
```

## Complete Planning and Execution Pipeline

### Integrated Planning System

```python
class IntegratedPlanningSystem:
    def __init__(self, llm_api_key: str):
        self.llm_planner = LLMPlanner(llm_api_key)
        self.action_executor = ActionExecutor()  # or ROS2ActionExecutor
        self.capability_validator = CapabilityValidator(self.llm_planner.robot_capabilities)
        self.safety_checker = SafetyConstraintChecker()

        # ROS publishers for status updates
        self.status_pub = rospy.Publisher('/cognitive_planning_status', String, queue_size=10)

    def execute_command(self, command: str, environment_context: Dict = None) -> bool:
        """Execute a complete command from natural language to robot action"""

        # Step 1: Generate action plan using LLM
        rospy.loginfo(f"Generating action plan for: {command}")
        plan = self.llm_planner.generate_action_plan(command, environment_context)

        # Log the plan
        status_msg = String()
        status_msg.data = f"Generated plan: {json.dumps(plan['actions'], indent=2)}"
        self.status_pub.publish(status_msg)

        # Step 2: Validate each action in the plan
        validated_actions = []
        for i, action_dict in enumerate(plan['actions']):
            action_step = ActionStep(
                action_type=ActionType[action_dict['action_type'].upper()],
                parameters=action_dict['parameters'],
                description=action_dict['description']
            )

            # Validate capability
            capability_validation = self.capability_validator.validate_action(action_step)
            if not capability_validation['is_valid']:
                rospy.logerr(f"Action {i} failed capability validation: {capability_validation['errors']}")

                # Try to suggest alternatives
                if capability_validation['suggested_alternatives']:
                    rospy.loginfo(f"Suggested alternatives: {capability_validation['suggested_alternatives']}")

                return False

            # Check safety
            safety_check = self.safety_checker.check_action_safety(action_step, environment_context or {})
            if not safety_check['is_safe']:
                rospy.logerr(f"Action {i} failed safety check: {safety_check['violations']}")
                return False

            validated_actions.append(action_step)

        # Step 3: Execute validated actions
        rospy.loginfo(f"Executing plan with {len(validated_actions)} actions")
        for i, action_step in enumerate(validated_actions):
            rospy.loginfo(f"Executing action {i+1}/{len(validated_actions)}: {action_step.description}")

            success = self.action_executor.execute_action(action_step)
            if not success:
                rospy.logerr(f"Action {i+1} failed to execute: {action_step.description}")

                # Log failure
                status_msg.data = f"Action failed: {action_step.description}"
                self.status_pub.publish(status_msg)

                return False

        rospy.loginfo("Command executed successfully")
        status_msg.data = "Command executed successfully"
        self.status_pub.publish(status_msg)

        return True

# Example usage
def main():
    rospy.init_node('integrated_planning_system')

    # Initialize the system
    planning_system = IntegratedPlanningSystem(api_key="your-api-key")

    # Example command
    command = "Go to the kitchen and bring me a cup of water"
    environment_context = {
        "robot_position": "living room",
        "kitchen_objects": ["cup", "bottle", "fridge"],
        "people_nearby": []
    }

    # Execute the command
    success = planning_system.execute_command(command, environment_context)

    if success:
        rospy.loginfo("Command completed successfully")
    else:
        rospy.logerr("Command failed")

    rospy.spin()

if __name__ == '__main__':
    main()
```

## Error Handling and Recovery

### Plan Refinement and Recovery

```python
class PlanRefiner:
    def __init__(self):
        self.failure_history = []

    def refine_plan_after_failure(self, original_command: str, failed_action_idx: int,
                                 failure_reason: str, environment_context: Dict) -> Dict:
        """Refine the plan after a failure occurs"""

        # Add to failure history
        self.failure_history.append({
            'command': original_command,
            'failed_action_idx': failed_action_idx,
            'failure_reason': failure_reason,
            'timestamp': rospy.Time.now()
        })

        # Generate a new plan that accounts for the failure
        refined_prompt = f"""
        Original command: "{original_command}"
        Action at index {failed_action_idx} failed because: {failure_reason}
        Current environment: {json.dumps(environment_context)}

        Please generate a new action plan that accounts for this failure and achieves the same goal.
        Consider alternative approaches if the original approach is not feasible.
        """

        # Use LLM to generate refined plan
        # (Implementation similar to original planner)
        pass

    def get_recovery_strategies(self, failure_type: str) -> List[str]:
        """Get possible recovery strategies for different failure types"""
        strategies = {
            'navigation_failure': [
                'Try alternative path',
                'Ask for clarification of destination',
                'Return to safe location'
            ],
            'manipulation_failure': [
                'Try different grasp approach',
                'Ask for help',
                'Find alternative object'
            ],
            'communication_failure': [
                'Repeat command',
                'Ask for clarification',
                'Use visual confirmation'
            ]
        }

        return strategies.get(failure_type, ['Retry action', 'Report failure'])
```

## Summary

Language-driven cognitive planning enables robots to understand and execute complex natural language commands by breaking them down into executable actions. By using LLMs for high-level planning, validating actions against robot capabilities, and ensuring safety constraints are met, we can create robust systems that bridge the gap between human language and robot action. The next chapter will integrate all these components into a complete autonomous humanoid system.