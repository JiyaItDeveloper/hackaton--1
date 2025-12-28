"""
LLM-based cognitive planner for VLA systems.
Uses Large Language Models to convert natural language into action plans.
"""
from typing import Dict, List, Any, Optional
import json
import time
import threading
import numpy as np
from dataclasses import dataclass


@dataclass
class PlanResult:
    """Result of the LLM planning process"""
    actions: List[Dict[str, Any]]
    reasoning: str
    confidence: float
    success: bool
    error_message: Optional[str] = None


class LLMPlanner:
    """
    LLM-based planner that converts natural language commands into action plans.
    This is a simulated implementation that mimics LLM behavior for demonstration.
    In a real implementation, this would interface with actual LLM APIs.
    """

    def __init__(self, model_name: str = "gpt-4"):
        self.model_name = model_name
        self.robot_capabilities = self._define_robot_capabilities()
        self.is_initialized = True
        print(f"LLM Planner initialized with {model_name} model simulation")

    def _define_robot_capabilities(self) -> Dict[str, Any]:
        """Define what the robot can and cannot do"""
        return {
            "navigation": {
                "movable": True,
                "max_distance": 10.0,  # meters
                "locations": ["kitchen", "living room", "bedroom", "office", "dining room", "bathroom"]
            },
            "manipulation": {
                "graspable": True,
                "max_weight": 2.0,  # kg
                "objects": ["cup", "book", "bottle", "phone", "keys", "plate", "fork", "spoon", "glass"]
            },
            "interaction": {
                "speak": True,
                "listen": True,
                "greet": True
            },
            "sensors": {
                "camera": True,
                "microphone": True,
                "lidar": False
            }
        }

    def generate_action_plan(self, command: str, environment_context: Dict[str, Any] = None) -> PlanResult:
        """
        Generate an action plan from a natural language command using LLM simulation.

        Args:
            command: Natural language command
            environment_context: Context about the current environment

        Returns:
            PlanResult with the generated action plan
        """
        start_time = time.time()

        try:
            # Create a prompt for the LLM simulation
            prompt = self._create_planning_prompt(command, environment_context)

            # Simulate LLM processing
            plan = self._simulate_llm_response(prompt)

            duration = time.time() - start_time

            # Calculate confidence based on the plan quality
            confidence = self._calculate_plan_confidence(plan)

            return PlanResult(
                actions=plan.get('actions', []),
                reasoning=plan.get('reasoning', ''),
                confidence=confidence,
                success=True
            )

        except Exception as e:
            duration = time.time() - start_time
            return PlanResult(
                actions=[],
                reasoning='',
                confidence=0.0,
                success=False,
                error_message=str(e)
            )

    def _create_planning_prompt(self, command: str, environment_context: Dict[str, Any] = None) -> str:
        """Create a prompt for the LLM"""
        context = environment_context or {}

        prompt = f"""
        Command: "{command}"

        Robot Capabilities:
        {json.dumps(self.robot_capabilities, indent=2)}

        Current Environment Context:
        {json.dumps(self._make_serializable(context), indent=2)}

        Please create a detailed action plan to execute this command. Consider:
        1. The robot's capabilities and limitations
        2. Safety constraints
        3. Environmental context
        4. Feasibility of each action
        5. Proper sequence of operations

        Return the plan in JSON format with:
        - "actions": array of action objects with action_type, parameters, and description
        - "reasoning": explanation of the plan
        """
        return prompt

    def _simulate_llm_response(self, prompt: str) -> Dict[str, Any]:
        """Simulate LLM response based on the prompt"""
        # Extract the command from the prompt
        command = ""
        for line in prompt.split('\n'):
            if line.startswith('Command:'):
                command = line.split('"')[1] if '"' in line else ""
                break

        # Generate mock action plan based on the command
        command_lower = command.lower()

        if 'go to' in command_lower or 'navigate to' in command_lower or 'move to' in command_lower:
            # Navigation command
            target = self._extract_target_location(command_lower)
            actions = [
                {
                    'action_type': 'navigation',
                    'action_name': 'move_to_location',
                    'parameters': {'location': target},
                    'description': f'Navigating to {target}'
                }
            ]
            reasoning = f"Command requires navigation to {target}. Robot is capable of navigation."

        elif 'pick up' in command_lower or 'grab' in command_lower or 'get' in command_lower:
            # Manipulation command
            target = self._extract_target_object(command_lower)
            actions = [
                {
                    'action_type': 'navigation',
                    'action_name': 'move_to_object',
                    'parameters': {'object': target},
                    'description': f'Moving to {target}'
                },
                {
                    'action_type': 'manipulation',
                    'action_name': 'grasp_object',
                    'parameters': {'object': target},
                    'description': f'Grasping {target}'
                }
            ]
            reasoning = f"Command requires picking up {target}. Robot will navigate to the object and grasp it."

        elif 'say' in command_lower or 'speak' in command_lower:
            # Speech command
            speech_text = self._extract_speech_content(command_lower)
            actions = [
                {
                    'action_type': 'interaction',
                    'action_name': 'speak',
                    'parameters': {'text': speech_text},
                    'description': f'Speaking: {speech_text}'
                }
            ]
            reasoning = f"Command requires speaking: {speech_text}"

        elif 'stop' in command_lower or 'wait' in command_lower:
            # Action command
            actions = [
                {
                    'action_type': 'action',
                    'action_name': 'wait',
                    'parameters': {'duration': 5},
                    'description': 'Waiting for 5 seconds'
                }
            ]
            reasoning = f"Command is to stop/wait. Robot will pause for 5 seconds."

        else:
            # Default case
            actions = [
                {
                    'action_type': 'action',
                    'action_name': 'idle',
                    'parameters': {},
                    'description': 'No specific action identified'
                }
            ]
            reasoning = f"Command '{command}' was not clearly understood. Robot will remain idle."

        return {
            'actions': actions,
            'reasoning': reasoning
        }

    def _extract_target_location(self, command: str) -> str:
        """Extract target location from command"""
        locations = self.robot_capabilities['navigation']['locations']
        for loc in locations:
            if loc in command:
                return loc
        return "unknown location"

    def _extract_target_object(self, command: str) -> str:
        """Extract target object from command"""
        objects = self.robot_capabilities['manipulation']['objects']
        for obj in objects:
            if obj in command:
                return obj
        return "unknown object"

    def _extract_speech_content(self, command: str) -> str:
        """Extract speech content from command"""
        # Look for content between quotes or after 'say'/'speak'
        import re
        match = re.search(r'(?:say|speak)\s+["\']([^"\']*)["\']', command, re.IGNORECASE)
        if match:
            return match.group(1)
        else:
            # If no quotes, take everything after 'say' or 'speak'
            for word in ['say', 'speak']:
                if word in command:
                    idx = command.find(word) + len(word)
                    return command[idx:].strip()
        return "Hello, world!"

    def _calculate_plan_confidence(self, plan: Dict[str, Any]) -> float:
        """Calculate confidence score for the plan"""
        actions = plan.get('actions', [])
        if not actions:
            return 0.1  # Low confidence if no actions

        # Base confidence on number of actions and their validity
        confidence = 0.5  # Base confidence

        for action in actions:
            action_type = action.get('action_type', '')
            if action_type in ['navigation', 'manipulation', 'interaction', 'action']:
                confidence += 0.15  # Boost for valid action types

        return min(1.0, confidence)  # Cap at 1.0

    def _make_serializable(self, obj):
        """Convert numpy types and other non-serializable objects to JSON serializable types"""
        if isinstance(obj, dict):
            return {key: self._make_serializable(value) for key, value in obj.items()}
        elif isinstance(obj, list):
            return [self._make_serializable(item) for item in obj]
        elif isinstance(obj, tuple):
            return [self._make_serializable(item) for item in obj]  # Convert tuple to list for JSON
        elif isinstance(obj, (np.integer, np.floating)):
            return float(obj)  # Convert numpy numbers to Python native types
        elif isinstance(obj, np.ndarray):
            return self._make_serializable(obj.tolist())  # Convert numpy arrays to lists
        else:
            return obj


class CapabilityValidator:
    """Validates action plans against robot capabilities"""

    def __init__(self, robot_capabilities: Dict[str, Any]):
        self.capabilities = robot_capabilities

    def validate_action(self, action: Dict[str, Any]) -> Dict[str, Any]:
        """Validate a single action against robot capabilities"""
        validation_result = {
            'is_valid': True,
            'errors': [],
            'warnings': [],
            'suggested_alternatives': []
        }

        action_type = action.get('action_type', '')
        parameters = action.get('parameters', {})

        if action_type == 'navigation':
            validation_result = self._validate_navigation(parameters)
        elif action_type == 'manipulation':
            validation_result = self._validate_manipulation(parameters)
        elif action_type == 'interaction':
            validation_result = self._validate_interaction(parameters)
        elif action_type == 'action':
            validation_result = self._validate_general_action(parameters)

        return validation_result

    def validate_plan(self, action_plan: List[Dict[str, Any]]) -> Dict[str, Any]:
        """Validate an entire action plan"""
        all_valid = True
        all_errors = []
        all_warnings = []
        all_alternatives = []

        for i, action in enumerate(action_plan):
            validation = self.validate_action(action)
            if not validation['is_valid']:
                all_valid = False
                all_errors.extend([f"Action {i}: {error}" for error in validation['errors']])
            all_warnings.extend([f"Action {i}: {warning}" for warning in validation['warnings']])
            all_alternatives.extend([f"Action {i}: {alt}" for alt in validation['suggested_alternatives']])

        return {
            'is_valid': all_valid,
            'errors': all_errors,
            'warnings': all_warnings,
            'suggested_alternatives': all_alternatives
        }

    def _validate_navigation(self, parameters: Dict[str, Any]) -> Dict[str, Any]:
        """Validate navigation action"""
        result = {
            'is_valid': True,
            'errors': [],
            'warnings': [],
            'suggested_alternatives': []
        }

        target_location = parameters.get('location', '').lower()

        # Check if location is in known locations
        known_locations = self.capabilities['navigation']['locations']
        if target_location not in known_locations:
            result['is_valid'] = False
            result['errors'].append(f"Unknown location: {target_location}")
            result['suggested_alternatives'] = known_locations

        # Check distance constraint if coordinates are provided
        if 'coordinates' in parameters:
            coords = parameters['coordinates']
            if isinstance(coords, (list, tuple)) and len(coords) >= 2:
                distance = (coords[0]**2 + coords[1]**2)**0.5
                max_distance = self.capabilities['navigation']['max_distance']
                if distance > max_distance:
                    result['is_valid'] = False
                    result['errors'].append(f"Distance {distance:.2f}m exceeds maximum {max_distance}m")

        return result

    def _validate_manipulation(self, parameters: Dict[str, Any]) -> Dict[str, Any]:
        """Validate manipulation action"""
        result = {
            'is_valid': True,
            'errors': [],
            'warnings': [],
            'suggested_alternatives': []
        }

        target_object = parameters.get('object', '').lower()

        # Check if object is in known graspable objects
        graspable_objects = self.capabilities['manipulation']['objects']
        if target_object not in [obj.lower() for obj in graspable_objects]:
            result['is_valid'] = False
            result['errors'].append(f"Unknown or ungraspable object: {target_object}")
            result['suggested_alternatives'] = graspable_objects

        # Check weight constraint if specified
        weight = parameters.get('weight', 0)
        max_weight = self.capabilities['manipulation']['max_weight']
        if weight > max_weight:
            result['is_valid'] = False
            result['errors'].append(f"Weight {weight}kg exceeds maximum {max_weight}kg")

        return result

    def _validate_interaction(self, parameters: Dict[str, Any]) -> Dict[str, Any]:
        """Validate interaction action"""
        result = {
            'is_valid': True,
            'errors': [],
            'warnings': [],
            'suggested_alternatives': []
        }

        # Check if robot has interaction capabilities
        if not self.capabilities['interaction']['speak']:
            result['is_valid'] = False
            result['errors'].append("Robot does not have speech capability")

        return result

    def _validate_general_action(self, parameters: Dict[str, Any]) -> Dict[str, Any]:
        """Validate general action"""
        result = {
            'is_valid': True,
            'errors': [],
            'warnings': [],
            'suggested_alternatives': []
        }

        # General validation for other action types
        return result


class SafetyConstraintChecker:
    """Checks safety constraints for action plans"""

    def __init__(self):
        self.safety_rules = self._define_safety_rules()

    def _define_safety_rules(self) -> Dict[str, Any]:
        """Define safety rules for the robot"""
        return {
            'no_go_zones': ['stairs', 'cliff', 'pool', 'construction_area'],
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

    def check_action_safety(self, action: Dict[str, Any], environment: Dict[str, Any]) -> Dict[str, Any]:
        """Check if an action is safe to execute in the current environment"""
        safety_check = {
            'is_safe': True,
            'violations': [],
            'suggested_safeguards': []
        }

        action_type = action.get('action_type', '')
        parameters = action.get('parameters', {})
        environment = environment or {}

        if action_type == 'navigation':
            safety_check = self._check_navigation_safety(parameters, environment)
        elif action_type == 'manipulation':
            safety_check = self._check_manipulation_safety(parameters, environment)
        elif action_type in ['interaction', 'action']:
            safety_check = self._check_general_safety(action_type, parameters, environment)

        return safety_check

    def _check_navigation_safety(self, parameters: Dict[str, Any], environment: Dict[str, Any]) -> Dict[str, Any]:
        """Check safety for navigation action"""
        safety_check = {
            'is_safe': True,
            'violations': [],
            'suggested_safeguards': []
        }

        # Check for no-go zones
        target_location = parameters.get('location', '').lower()
        if any(zone in target_location for zone in self.safety_rules['no_go_zones']):
            safety_check['is_safe'] = False
            safety_check['violations'].append(f"Target location {target_location} is in a no-go zone")

        # Check distance to people
        people_nearby = environment.get('people_nearby', [])
        min_distance = self.safety_rules['minimum_distance_to_people']

        for person in people_nearby:
            person_pos = person.get('position', (0, 0))
            target_pos = parameters.get('coordinates', (0, 0))
            if isinstance(person_pos, (list, tuple)) and isinstance(target_pos, (list, tuple)):
                distance = ((target_pos[0] - person_pos[0])**2 + (target_pos[1] - person_pos[1])**2)**0.5
                if distance < min_distance:
                    safety_check['is_safe'] = False
                    safety_check['violations'].append(f"Target too close to person (distance: {distance:.2f}m)")

        return safety_check

    def _check_manipulation_safety(self, parameters: Dict[str, Any], environment: Dict[str, Any]) -> Dict[str, Any]:
        """Check safety for manipulation action"""
        safety_check = {
            'is_safe': True,
            'violations': [],
            'suggested_safeguards': []
        }

        # Check payload for manipulation
        payload = parameters.get('weight', 0)
        max_payload = self.safety_rules['maximum_payload']

        if payload > max_payload:
            safety_check['is_safe'] = False
            safety_check['violations'].append(f"Payload {payload}kg exceeds maximum {max_payload}kg")

        # Check if people are nearby during manipulation
        people_nearby = environment.get('people_nearby', [])
        if people_nearby:
            safety_check['suggested_safeguards'].append("Verify no people are in immediate vicinity before manipulation")

        return safety_check

    def _check_general_safety(self, action_type: str, parameters: Dict[str, Any], environment: Dict[str, Any]) -> Dict[str, Any]:
        """Check safety for general actions"""
        safety_check = {
            'is_safe': True,
            'violations': [],
            'suggested_safeguards': []
        }

        # General safety checks
        return safety_check

    def check_plan_safety(self, action_plan: List[Dict[str, Any]], environment: Dict[str, Any]) -> Dict[str, Any]:
        """Check safety for an entire action plan"""
        all_safe = True
        all_violations = []
        all_safeguards = []

        for i, action in enumerate(action_plan):
            safety = self.check_action_safety(action, environment)
            if not safety['is_safe']:
                all_safe = False
                all_violations.extend([f"Action {i}: {violation}" for violation in safety['violations']])
            all_safeguards.extend([f"Action {i}: {safeguard}" for safeguard in safety['suggested_safeguards']])

        return {
            'is_safe': all_safe,
            'violations': all_violations,
            'suggested_safeguards': all_safeguards
        }


# Example usage and testing
if __name__ == "__main__":
    print("Testing LLM Planner...")

    # Create the planner
    planner = LLMPlanner(model_name="gpt-4-simulation")

    # Create validator and safety checker
    validator = CapabilityValidator(planner.robot_capabilities)
    safety_checker = SafetyConstraintChecker()

    # Test commands
    test_commands = [
        "Go to the kitchen",
        "Pick up the red cup",
        "Say hello to everyone",
        "Stop moving immediately"
    ]

    for command in test_commands:
        print(f"\n--- Processing Command: {command} ---")

        # Generate action plan
        plan_result = planner.generate_action_plan(command)

        if plan_result.success:
            print(f"Plan generated with confidence: {plan_result.confidence:.2f}")
            print(f"Reasoning: {plan_result.reasoning}")
            print("Actions:")
            for i, action in enumerate(plan_result.actions):
                print(f"  {i+1}. {action['description']}")

            # Validate the plan
            validation = validator.validate_plan(plan_result.actions)
            print(f"Validation: {'PASS' if validation['is_valid'] else 'FAIL'}")
            if validation['errors']:
                print(f"  Errors: {validation['errors']}")

            # Check safety (with mock environment)
            mock_environment = {
                'people_nearby': [{'position': (1, 1), 'name': 'person1'}]
            }
            safety = safety_checker.check_plan_safety(plan_result.actions, mock_environment)
            print(f"Safety: {'SAFE' if safety['is_safe'] else 'UNSAFE'}")
            if safety['violations']:
                print(f"  Violations: {safety['violations']}")
        else:
            print(f"Failed to generate plan: {plan_result.error_message}")

    print("\nLLM Planner test completed!")