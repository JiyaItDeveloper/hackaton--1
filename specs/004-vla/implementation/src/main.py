"""
Main entry point for the Vision-Language-Action (VLA) system.
Integrates all components into a complete autonomous humanoid system.
"""
from typing import Dict, Any, Optional
import time
import threading
import numpy as np
from dataclasses import dataclass

from .vision.vision_system import VisionSystem, SceneContext
from .language.language_system import LanguageSystem, ActionPlan
from .language.whisper_stt import WhisperSTTNode, VoiceCommandProcessor
from .language.llm_planner import LLMPlanner, CapabilityValidator, SafetyConstraintChecker
from .action.action_system import ActionSystem, ActionResult
from .pipeline.vla_pipeline import VLAPipeline, AutonomousHumanoidSystem
from .utils.ros_integration import VoiceCommandNode, NavigationNode, ActionNode


@dataclass
class SystemStatus:
    """Current status of the autonomous humanoid system"""
    mode: str  # idle, listening, processing, executing, error
    battery_level: float
    position: tuple
    last_command: str
    active_tasks: int
    components_status: Dict[str, bool]


class IntegratedVLA:
    """Complete integrated VLA system"""

    def __init__(self):
        # Initialize all core components
        self.vision_system = VisionSystem()
        self.language_system = LanguageSystem()
        self.action_system = ActionSystem()
        self.vla_pipeline = VLAPipeline()

        # Initialize voice components
        self.whisper_stt = WhisperSTTNode(model_size="base")
        self.voice_processor = VoiceCommandProcessor(self.whisper_stt)

        # Initialize planning components
        self.llm_planner = LLMPlanner(model_name="gpt-4-simulation")
        self.capability_validator = CapabilityValidator(self.llm_planner.robot_capabilities)
        self.safety_checker = SafetyConstraintChecker()

        # Initialize ROS integration
        self.voice_node = VoiceCommandNode()
        self.nav_node = NavigationNode()
        self.action_node = ActionNode()

        # Set up voice processor in ROS node
        self.voice_node.set_voice_processor(self.voice_processor)

        # System state
        self.system_state = {
            'mode': 'idle',
            'battery_level': 100.0,
            'position': (0, 0),
            'last_command': '',
            'active_tasks': 0,
            'components_status': {
                'vision': True,
                'language': True,
                'action': True,
                'stt': True,
                'planning': True
            }
        }

        self.running = False
        self.command_queue = []
        self.lock = threading.Lock()

    def start_system(self):
        """Start the integrated VLA system"""
        print("Starting Integrated VLA System...")

        # Initialize all components
        self.system_state['mode'] = 'initializing'

        # Set up ROS node connections (simulated)
        self._connect_ros_nodes()

        self.system_state['mode'] = 'idle'
        self.running = True

        print("Integrated VLA System started successfully!")
        return True

    def stop_system(self):
        """Stop the integrated VLA system"""
        print("Stopping Integrated VLA System...")
        self.running = False

        # Clean up resources
        self.system_state['mode'] = 'shutdown'

        print("Integrated VLA System stopped.")

    def _connect_ros_nodes(self):
        """Connect ROS nodes for communication (simulated)"""
        # In a real implementation, this would establish actual ROS communication
        # For simulation, we'll just acknowledge the connection
        print("Connected ROS nodes: voice, navigation, action")

    def process_voice_command(self, audio_data: np.ndarray) -> Dict[str, Any]:
        """Process a voice command through the complete pipeline"""
        with self.lock:
            self.system_state['mode'] = 'processing'
            self.system_state['active_tasks'] += 1

        try:
            # Process voice command
            voice_result = self.voice_processor.process_voice_command(audio_data)

            if not voice_result['success']:
                error_response = {
                    'success': False,
                    'error': voice_result['error'],
                    'command_executed': False
                }
                with self.lock:
                    self.system_state['mode'] = 'idle'
                    self.system_state['active_tasks'] -= 1
                return error_response

            command_text = voice_result['transcription']
            self.system_state['last_command'] = command_text

            # Generate action plan using LLM
            environment_context = self._get_environment_context()
            plan_result = self.llm_planner.generate_action_plan(command_text, environment_context)

            if not plan_result.success:
                error_response = {
                    'success': False,
                    'error': plan_result.error_message,
                    'command_executed': False
                }
                with self.lock:
                    self.system_state['mode'] = 'idle'
                    self.system_state['active_tasks'] -= 1
                return error_response

            # Validate the plan against capabilities
            validation_result = self.capability_validator.validate_plan(plan_result.actions)
            if not validation_result['is_valid']:
                error_response = {
                    'success': False,
                    'error': f"Plan validation failed: {validation_result['errors']}",
                    'command_executed': False
                }
                with self.lock:
                    self.system_state['mode'] = 'idle'
                    self.system_state['active_tasks'] -= 1
                return error_response

            # Check safety of the plan
            safety_result = self.safety_checker.check_plan_safety(plan_result.actions, environment_context)
            if not safety_result['is_safe']:
                error_response = {
                    'success': False,
                    'error': f"Plan safety check failed: {safety_result['violations']}",
                    'command_executed': False
                }
                with self.lock:
                    self.system_state['mode'] = 'idle'
                    self.system_state['active_tasks'] -= 1
                return error_response

            # Execute the validated and safe plan
            action_results = self.action_system.execute_plan(plan_result.actions, environment_context)

            success = all(result.success for result in action_results) if action_results else True

            response = {
                'success': success,
                'transcription': command_text,
                'confidence': voice_result['confidence'],
                'plan_actions': len(plan_result.actions),
                'action_results': [result.success for result in action_results],
                'command_executed': success,
                'reasoning': plan_result.reasoning
            }

            with self.lock:
                self.system_state['mode'] = 'idle'
                self.system_state['active_tasks'] -= 1

            return response

        except Exception as e:
            with self.lock:
                self.system_state['mode'] = 'error'
                self.system_state['active_tasks'] -= 1

            return {
                'success': False,
                'error': str(e),
                'command_executed': False
            }

    def process_visual_command(self, command_text: str, image: np.ndarray = None) -> Dict[str, Any]:
        """Process a text command with optional visual context"""
        with self.lock:
            self.system_state['mode'] = 'processing'
            self.system_state['active_tasks'] += 1

        try:
            # Get environment context from vision system if image provided
            environment_context = {}
            if image is not None:
                scene_context = self.vision_system.process_environment(image)
                environment_context = self._convert_scene_to_context(scene_context)

            # Generate action plan using LLM
            plan_result = self.llm_planner.generate_action_plan(command_text, environment_context)

            if not plan_result.success:
                error_response = {
                    'success': False,
                    'error': plan_result.error_message,
                    'command_executed': False
                }
                with self.lock:
                    self.system_state['mode'] = 'idle'
                    self.system_state['active_tasks'] -= 1
                return error_response

            # Validate and check safety
            validation_result = self.capability_validator.validate_plan(plan_result.actions)
            if not validation_result['is_valid']:
                error_response = {
                    'success': False,
                    'error': f"Plan validation failed: {validation_result['errors']}",
                    'command_executed': False
                }
                with self.lock:
                    self.system_state['mode'] = 'idle'
                    self.system_state['active_tasks'] -= 1
                return error_response

            safety_result = self.safety_checker.check_plan_safety(plan_result.actions, environment_context)
            if not safety_result['is_safe']:
                error_response = {
                    'success': False,
                    'error': f"Plan safety check failed: {safety_result['violations']}",
                    'command_executed': False
                }
                with self.lock:
                    self.system_state['mode'] = 'idle'
                    self.system_state['active_tasks'] -= 1
                return error_response

            # Execute the plan
            action_results = self.action_system.execute_plan(plan_result.actions, environment_context)

            success = all(result.success for result in action_results) if action_results else True

            response = {
                'success': success,
                'command': command_text,
                'plan_actions': len(plan_result.actions),
                'action_results': [result.success for result in action_results],
                'command_executed': success,
                'reasoning': plan_result.reasoning
            }

            with self.lock:
                self.system_state['mode'] = 'idle'
                self.system_state['active_tasks'] -= 1

            return response

        except Exception as e:
            with self.lock:
                self.system_state['mode'] = 'error'
                self.system_state['active_tasks'] -= 1

            return {
                'success': False,
                'error': str(e),
                'command_executed': False
            }

    def get_system_status(self) -> SystemStatus:
        """Get current system status"""
        return SystemStatus(
            mode=self.system_state['mode'],
            battery_level=self.system_state['battery_level'],
            position=self.system_state['position'],
            last_command=self.system_state['last_command'],
            active_tasks=self.system_state['active_tasks'],
            components_status=self.system_state['components_status']
        )

    def _get_environment_context(self) -> Dict[str, Any]:
        """Get current environment context (simulated)"""
        # In a real system, this would come from sensors and perception
        return {
            'known_locations': ['kitchen', 'living room', 'bedroom', 'office'],
            'objects': [
                {'label': 'cup', 'position': (2, 3), 'confidence': 0.9},
                {'label': 'table', 'position': (1, 1), 'confidence': 0.95}
            ],
            'people_nearby': [{'position': (0, 0), 'name': 'user'}],
            'current_position': self.system_state['position']
        }

    def _convert_scene_to_context(self, scene_context: SceneContext) -> Dict[str, Any]:
        """Convert scene context to environment context"""
        if scene_context is None:
            return {}

        objects = []
        for obj in scene_context.objects:
            objects.append({
                'label': obj.label,
                'confidence': obj.confidence,
                'position': obj.center,
                'bbox': obj.bbox
            })

        return {
            'objects': objects,
            'spatial_relations': scene_context.spatial_relations,
            'environmental_features': scene_context.environmental_features
        }

    def run_continuous(self):
        """Run the system in continuous mode"""
        if not self.start_system():
            print("Failed to start system")
            return

        print("Integrated VLA System running in continuous mode...")
        print("Press Ctrl+C to stop")

        try:
            while self.running:
                # In a real implementation, this would listen for commands
                # For simulation, we'll just keep the system alive
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\nShutting down system...")
        finally:
            self.stop_system()


def main():
    """Main function to demonstrate the integrated VLA system"""
    print("Vision-Language-Action (VLA) System")
    print("=====================================")

    # Create the integrated system
    vla_system = IntegratedVLA()

    # Test with simulated audio data
    print("\n1. Testing Voice Command Processing:")
    mock_audio = np.random.uniform(-0.3, 0.3, size=(16000,))  # 1 second of mock audio

    result = vla_system.process_voice_command(mock_audio)
    print(f"   Voice command result: {result['success']}")
    if result['success']:
        print(f"   Executed command: {result.get('transcription', 'N/A')}")
        print(f"   Actions executed: {result.get('plan_actions', 0)}")
    else:
        print(f"   Error: {result.get('error', 'Unknown error')}")

    # Test with text command and mock image
    print("\n2. Testing Text Command with Visual Context:")
    mock_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)

    text_result = vla_system.process_visual_command("Go to the kitchen", mock_image)
    print(f"   Text command result: {text_result['success']}")
    if text_result['success']:
        print(f"   Command: {text_result.get('command', 'N/A')}")
        print(f"   Actions planned: {text_result.get('plan_actions', 0)}")
    else:
        print(f"   Error: {text_result.get('error', 'Unknown error')}")

    # Check system status
    print("\n3. System Status:")
    status = vla_system.get_system_status()
    print(f"   Mode: {status.mode}")
    print(f"   Battery: {status.battery_level}%")
    print(f"   Position: {status.position}")
    print(f"   Active tasks: {status.active_tasks}")
    print(f"   Last command: {status.last_command}")

    print("\n4. Demonstrating Continuous Operation:")
    print("   (This would run continuously in a real application)")
    print("   System initialized and ready for commands!")

    # Show that the system can be run continuously
    # (We'll simulate this without actually running continuously)
    print("   System ready for real-time command processing...")


if __name__ == "__main__":
    main()