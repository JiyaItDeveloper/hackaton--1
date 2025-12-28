"""
Language system for Vision-Language-Action (VLA) systems.
Handles natural language processing, intent classification, and action mapping.
"""
from typing import Dict, List, Any, Optional, Tuple
import re
from dataclasses import dataclass


@dataclass
class IntentResult:
    """Result of intent classification"""
    intent_type: str
    parameters: Dict[str, Any]
    confidence: float
    original_text: str


@dataclass
class ActionPlan:
    """Plan of actions to execute"""
    actions: List[Dict[str, Any]]
    reasoning: str
    confidence: float


class LanguageSystem:
    """Core language system for VLA applications"""

    def __init__(self):
        self.intent_classifier = IntentClassifier()
        self.action_mapper = ActionMapper()
        self.nlp_processor = NLPProcessor()

    def process_command(self, command: str) -> IntentResult:
        """
        Process a natural language command to extract intent.

        Args:
            command: Natural language command from user

        Returns:
            IntentResult containing the classified intent
        """
        # Preprocess the command
        processed_command = self.nlp_processor.preprocess(command)

        # Classify the intent
        intent_result = self.intent_classifier.classify(processed_command)

        return intent_result

    def generate_action_plan(self, command: str, environment_context: Dict[str, Any] = None) -> ActionPlan:
        """
        Generate an action plan from a command and environmental context.

        Args:
            command: Natural language command
            environment_context: Context from the vision system

        Returns:
            ActionPlan containing sequence of actions to execute
        """
        # Process the command to get intent
        intent_result = self.process_command(command)

        # Map intent to actions
        action_plan = self.action_mapper.map_to_actions(
            intent_result,
            environment_context or {}
        )

        return action_plan


class IntentClassifier:
    """Classifies user commands into specific intents"""

    def __init__(self):
        # Define command patterns for different intent types
        self.command_patterns = {
            'navigation': [
                r'go to (.+)',
                r'move to (.+)',
                r'navigate to (.+)',
                r'go (.+)',
                r'move (.+)',
                r'walk to (.+)',
                r'head to (.+)',
                r'get to (.+)'
            ],
            'manipulation': [
                r'pick up (.+)',
                r'grab (.+)',
                r'take (.+)',
                r'get (.+)',
                r'lift (.+)',
                r'collect (.+)',
                r'hold (.+)',
                r'pick (.+)'
            ],
            'action': [
                r'stop',
                r'wait',
                r'continue',
                r'start',
                r'pause',
                r'help',
                r'say (.+)',
                r'tell (.+)'
            ],
            'query': [
                r'where is (.+)',
                r'find (.+)',
                r'locate (.+)',
                r'what is (.+)',
                r'how many (.+)'
            ]
        }

    def classify(self, text: str) -> IntentResult:
        """
        Classify a text command into an intent.

        Args:
            text: Input text to classify

        Returns:
            IntentResult with the classification
        """
        text_lower = text.lower().strip()

        for intent_type, patterns in self.command_patterns.items():
            for pattern in patterns:
                match = re.search(pattern, text_lower)
                if match:
                    # Extract parameters from the match
                    parameters = {
                        'target': match.group(1) if len(match.groups()) > 0 else None,
                        'original_command': text
                    }

                    # Calculate confidence based on pattern match
                    confidence = self._calculate_confidence(pattern, text_lower)

                    return IntentResult(
                        intent_type=intent_type,
                        parameters=parameters,
                        confidence=confidence,
                        original_text=text
                    )

        # If no pattern matches, return unknown intent
        return IntentResult(
            intent_type='unknown',
            parameters={'original_command': text},
            confidence=0.1,
            original_text=text
        )

    def _calculate_confidence(self, pattern: str, text: str) -> float:
        """Calculate confidence score for a pattern match"""
        # Simple confidence calculation based on pattern specificity
        # More specific patterns (longer) get higher confidence
        return min(0.9, len(pattern) / 20.0)


class ActionMapper:
    """Maps intents to executable action plans"""

    def __init__(self):
        self.action_templates = {
            'navigation': self._create_navigation_actions,
            'manipulation': self._create_manipulation_actions,
            'action': self._create_action_actions,
            'query': self._create_query_actions
        }

    def map_to_actions(self, intent_result: IntentResult, environment_context: Dict[str, Any]) -> ActionPlan:
        """
        Map an intent to a sequence of actions.

        Args:
            intent_result: Classified intent
            environment_context: Context from vision system

        Returns:
            ActionPlan with sequence of actions to execute
        """
        intent_type = intent_result.intent_type

        if intent_type in self.action_templates:
            actions = self.action_templates[intent_type](intent_result, environment_context)
        else:
            actions = self._create_unknown_actions(intent_result)

        reasoning = f"Converted {intent_type} intent to action sequence"
        confidence = intent_result.confidence

        return ActionPlan(
            actions=actions,
            reasoning=reasoning,
            confidence=confidence
        )

    def _create_navigation_actions(self, intent_result: IntentResult, env_ctx: Dict[str, Any]) -> List[Dict[str, Any]]:
        """Create navigation actions"""
        target_location = intent_result.parameters.get('target', '').lower()

        # In a real system, this would look up coordinates for known locations
        # For now, we'll create a generic navigation action
        actions = []

        # Check if target is a known location
        if target_location in env_ctx.get('known_locations', []):
            actions.append({
                'action_type': 'navigation',
                'action_name': 'move_to_location',
                'parameters': {
                    'location': target_location,
                    'coordinates': env_ctx.get('known_locations', {}).get(target_location, (0, 0))
                },
                'description': f'Navigating to {target_location}'
            })
        else:
            # Try to find the location in the current environment
            if 'objects' in env_ctx:
                for obj in env_ctx['objects']:
                    if target_location in obj.get('label', '').lower():
                        actions.append({
                            'action_type': 'navigation',
                            'action_name': 'move_to_object',
                            'parameters': {
                                'object_label': obj.get('label'),
                                'position': obj.get('position', (0, 0))
                            },
                            'description': f'Navigating to {obj.get("label")}'
                        })
                        break

        if not actions:
            # If no specific location found, create a general navigation action
            actions.append({
                'action_type': 'navigation',
                'action_name': 'explore',
                'parameters': {
                    'target_description': target_location
                },
                'description': f'Exploring to find {target_location}'
            })

        return actions

    def _create_manipulation_actions(self, intent_result: IntentResult, env_ctx: Dict[str, Any]) -> List[Dict[str, Any]]:
        """Create manipulation actions"""
        target_object = intent_result.parameters.get('target', '').lower()

        actions = []

        # Find the object in the environment
        if 'objects' in env_ctx:
            for obj in env_ctx['objects']:
                if target_object in obj.get('label', '').lower():
                    actions.extend([
                        {
                            'action_type': 'navigation',
                            'action_name': 'move_to_object',
                            'parameters': {
                                'object_label': obj.get('label'),
                                'position': obj.get('position', (0, 0))
                            },
                            'description': f'Moving to {obj.get("label")}'
                        },
                        {
                            'action_type': 'manipulation',
                            'action_name': 'grasp_object',
                            'parameters': {
                                'object_label': obj.get('label'),
                                'position': obj.get('position', (0, 0))
                            },
                            'description': f'Grasping {obj.get("label")}'
                        }
                    ])
                    break

        if not actions:
            # If object not found, create a search action
            actions.append({
                'action_type': 'action',
                'action_name': 'search',
                'parameters': {
                    'target_object': target_object
                },
                'description': f'Searching for {target_object}'
            })

        return actions

    def _create_action_actions(self, intent_result: IntentResult, env_ctx: Dict[str, Any]) -> List[Dict[str, Any]]:
        """Create general action commands"""
        command = intent_result.original_text.lower()

        actions = []

        if 'stop' in command:
            actions.append({
                'action_type': 'action',
                'action_name': 'stop',
                'parameters': {},
                'description': 'Stopping current activity'
            })
        elif 'wait' in command:
            actions.append({
                'action_type': 'action',
                'action_name': 'wait',
                'parameters': {'duration': 5},
                'description': 'Waiting for 5 seconds'
            })
        elif 'start' in command or 'continue' in command:
            actions.append({
                'action_type': 'action',
                'action_name': 'resume',
                'parameters': {},
                'description': 'Resuming activity'
            })
        elif 'say' in command or 'tell' in command:
            # Extract what to say from the command
            say_match = re.search(r'(?:say|tell) (.+)', command)
            if say_match:
                text_to_say = say_match.group(1)
                actions.append({
                    'action_type': 'action',
                    'action_name': 'speak',
                    'parameters': {'text': text_to_say},
                    'description': f'Speaking: {text_to_say}'
                })

        if not actions:
            # Default action for unrecognized commands
            actions.append({
                'action_type': 'action',
                'action_name': 'idle',
                'parameters': {},
                'description': 'Remaining idle - command not understood'
            })

        return actions

    def _create_query_actions(self, intent_result: IntentResult, env_ctx: Dict[str, Any]) -> List[Dict[str, Any]]:
        """Create query response actions"""
        command = intent_result.original_text.lower()
        target = intent_result.parameters.get('target', '')

        actions = []

        # Handle "where is X" queries
        if 'where is' in command:
            if 'objects' in env_ctx:
                for obj in env_ctx['objects']:
                    if target.lower() in obj.get('label', '').lower():
                        actions.append({
                            'action_type': 'action',
                            'action_name': 'report_location',
                            'parameters': {
                                'object': obj.get('label'),
                                'location': obj.get('position', 'unknown')
                            },
                            'description': f'Reporting location of {obj.get("label")}'
                        })
                        break

        # Default response if object not found
        if not actions:
            actions.append({
                'action_type': 'action',
                'action_name': 'report_not_found',
                'parameters': {'target': target},
                'description': f'Could not find {target}'
            })

        return actions

    def _create_unknown_actions(self, intent_result: IntentResult) -> List[Dict[str, Any]]:
        """Create actions for unknown intents"""
        return [{
            'action_type': 'action',
            'action_name': 'request_clarification',
            'parameters': {
                'original_command': intent_result.original_text
            },
            'description': f'Could not understand command: {intent_result.original_text}'
        }]


class NLPProcessor:
    """Basic NLP preprocessing"""

    def __init__(self):
        self.stop_words = {'the', 'a', 'an', 'and', 'or', 'but', 'in', 'on', 'at', 'to', 'for', 'of', 'with', 'by'}

    def preprocess(self, text: str) -> str:
        """
        Preprocess text for intent classification.

        Args:
            text: Input text

        Returns:
            Preprocessed text
        """
        # Convert to lowercase
        text = text.lower()

        # Remove extra whitespace
        text = ' '.join(text.split())

        # In a real implementation, you might also:
        # - Remove punctuation
        # - Lemmatize words
        # - Handle synonyms
        # - Perform named entity recognition

        return text


# Example usage
if __name__ == "__main__":
    lang_system = LanguageSystem()

    # Test commands
    test_commands = [
        "Go to the kitchen",
        "Pick up the red cup",
        "Stop moving",
        "Where is the table?"
    ]

    for cmd in test_commands:
        print(f"\nCommand: {cmd}")
        intent = lang_system.process_command(cmd)
        print(f"Intent: {intent.intent_type}, Parameters: {intent.parameters}")

        # Create a mock environment context
        env_ctx = {
            'known_locations': {
                'kitchen': (5, 5),
                'living room': (0, 0),
                'bedroom': (-3, 4)
            },
            'objects': [
                {'label': 'red cup', 'position': (2, 3)},
                {'label': 'table', 'position': (1, 1)}
            ]
        }

        action_plan = lang_system.generate_action_plan(cmd, env_ctx)
        print(f"Action plan: {action_plan.actions}")