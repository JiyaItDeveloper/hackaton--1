"""
Vision-Language-Action (VLA) pipeline integration.
Coordinates the flow between vision, language, and action systems.
"""
from typing import Dict, List, Any, Optional
import numpy as np
import threading
import time
from dataclasses import dataclass

from ..vision.vision_system import VisionSystem, SceneContext
from ..language.language_system import LanguageSystem, ActionPlan
from ..action.action_system import ActionSystem, ActionResult


@dataclass
class VLAPipelineResult:
    """Result of the complete VLA pipeline execution"""
    success: bool
    vision_result: Optional[SceneContext]
    language_result: Optional[ActionPlan]
    action_results: List[ActionResult]
    execution_time: float
    error_message: Optional[str] = None


class VLAPipeline:
    """Main pipeline that integrates vision, language, and action systems"""

    def __init__(self):
        self.vision_system = VisionSystem()
        self.language_system = LanguageSystem()
        self.action_system = ActionSystem()
        self.pipeline_history = []

    def execute_sequential_pipeline(self, command: str, image: np.ndarray = None) -> VLAPipelineResult:
        """
        Execute the VLA pipeline sequentially: Language -> Vision -> Action.

        Args:
            command: Natural language command from user
            image: Image from robot's camera (optional)

        Returns:
            VLAPipelineResult with execution details
        """
        start_time = time.time()

        try:
            # Step 1: Process the command with the language system
            print(f"Processing command: {command}")
            action_plan = self.language_system.generate_action_plan(command)

            # Step 2: Process the environment with the vision system (if image provided)
            scene_context = None
            if image is not None:
                print("Processing environment with vision system")
                scene_context = self.vision_system.process_environment(image)

            # Step 3: Execute the action plan with the action system
            print("Executing action plan")
            environment_context = self._create_environment_context(scene_context)
            action_results = self.action_system.execute_plan(action_plan.actions, environment_context)

            execution_time = time.time() - start_time

            result = VLAPipelineResult(
                success=all(result.success for result in action_results) if action_results else True,
                vision_result=scene_context,
                language_result=action_plan,
                action_results=action_results,
                execution_time=execution_time
            )

            self.pipeline_history.append(result)
            return result

        except Exception as e:
            execution_time = time.time() - start_time
            error_msg = str(e)

            result = VLAPipelineResult(
                success=False,
                vision_result=None,
                language_result=None,
                action_results=[],
                execution_time=execution_time,
                error_message=error_msg
            )

            self.pipeline_history.append(result)
            return result

    def execute_parallel_pipeline(self, command: str, image: np.ndarray = None) -> VLAPipelineResult:
        """
        Execute the VLA pipeline with parallel processing where possible.

        Args:
            command: Natural language command from user
            image: Image from robot's camera (optional)

        Returns:
            VLAPipelineResult with execution details
        """
        start_time = time.time()

        # Shared results
        language_result = [None]
        vision_result = [None]
        execution_error = [None]

        def process_language():
            try:
                language_result[0] = self.language_system.generate_action_plan(command)
            except Exception as e:
                execution_error[0] = f"Language processing error: {str(e)}"

        def process_vision():
            try:
                if image is not None:
                    vision_result[0] = self.vision_system.process_environment(image)
            except Exception as e:
                execution_error[0] = f"Vision processing error: {str(e)}"

        # Run language and vision processing in parallel
        language_thread = threading.Thread(target=process_language)
        vision_thread = threading.Thread(target=process_vision)

        language_thread.start()
        vision_thread.start()

        # Wait for both to complete
        language_thread.join()
        vision_thread.join()

        if execution_error[0]:
            execution_time = time.time() - start_time
            result = VLAPipelineResult(
                success=False,
                vision_result=None,
                language_result=None,
                action_results=[],
                execution_time=execution_time,
                error_message=execution_error[0]
            )
            self.pipeline_history.append(result)
            return result

        try:
            # Execute the action plan
            environment_context = self._create_environment_context(vision_result[0])
            action_results = self.action_system.execute_plan(
                language_result[0].actions,
                environment_context
            )

            execution_time = time.time() - start_time

            result = VLAPipelineResult(
                success=all(result.success for result in action_results) if action_results else True,
                vision_result=vision_result[0],
                language_result=language_result[0],
                action_results=action_results,
                execution_time=execution_time
            )

            self.pipeline_history.append(result)
            return result

        except Exception as e:
            execution_time = time.time() - start_time
            error_msg = str(e)

            result = VLAPipelineResult(
                success=False,
                vision_result=vision_result[0],
                language_result=language_result[0],
                action_results=[],
                execution_time=execution_time,
                error_message=error_msg
            )

            self.pipeline_history.append(result)
            return result

    def execute_with_feedback(self, command: str, image: np.ndarray = None) -> VLAPipelineResult:
        """
        Execute the VLA pipeline with feedback loops for improved performance.

        Args:
            command: Natural language command from user
            image: Image from robot's camera (optional)

        Returns:
            VLAPipelineResult with execution details
        """
        start_time = time.time()

        try:
            # Initial vision processing
            current_scene = None
            if image is not None:
                current_scene = self.vision_system.process_environment(image)

            # Generate initial action plan
            environment_context = self._create_environment_context(current_scene)
            action_plan = self.language_system.generate_action_plan(command, environment_context)

            # Execute plan with periodic environment updates
            action_results = []
            for i, action in enumerate(action_plan.actions):
                print(f"Executing action {i+1}/{len(action_plan.actions)}: {action.get('description', 'Unknown')}")

                # Update environment before each action (if possible)
                if image is not None:
                    current_scene = self.vision_system.process_environment(image)
                    environment_context = self._create_environment_context(current_scene)

                # Execute the action
                result = self.action_system.execute_action(action, environment_context)
                action_results.append(result)

                # Check if execution was successful
                if not result.success:
                    print(f"Action {i+1} failed: {result.error_message}")
                    break

            execution_time = time.time() - start_time

            result = VLAPipelineResult(
                success=all(r.success for r in action_results) if action_results else True,
                vision_result=current_scene,
                language_result=action_plan,
                action_results=action_results,
                execution_time=execution_time
            )

            self.pipeline_history.append(result)
            return result

        except Exception as e:
            execution_time = time.time() - start_time
            error_msg = str(e)

            result = VLAPipelineResult(
                success=False,
                vision_result=None,
                language_result=None,
                action_results=[],
                execution_time=execution_time,
                error_message=error_msg
            )

            self.pipeline_history.append(result)
            return result

    def _create_environment_context(self, scene_context: SceneContext) -> Dict[str, Any]:
        """Convert scene context to environment context for other systems"""
        if scene_context is None:
            return {}

        # Convert scene context to a format usable by language and action systems
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

    def get_pipeline_statistics(self) -> Dict[str, Any]:
        """Get statistics about pipeline execution"""
        if not self.pipeline_history:
            return {
                'total_executions': 0,
                'success_rate': 0.0,
                'avg_execution_time': 0.0
            }

        total = len(self.pipeline_history)
        successful = sum(1 for result in self.pipeline_history if result.success)
        avg_time = sum(result.execution_time for result in self.pipeline_history) / total

        return {
            'total_executions': total,
            'successful_executions': successful,
            'failed_executions': total - successful,
            'success_rate': successful / total,
            'avg_execution_time': avg_time
        }


class AutonomousHumanoidSystem:
    """Complete autonomous humanoid system integrating all VLA components"""

    def __init__(self):
        self.vla_pipeline = VLAPipeline()
        self.running = False
        self.command_queue = []
        self.system_state = {
            'current_mode': 'idle',
            'battery_level': 100.0,
            'position': (0, 0),
            'last_command': None
        }

    def start(self):
        """Start the autonomous system"""
        self.running = True
        print("Autonomous Humanoid System started")

    def stop(self):
        """Stop the autonomous system"""
        self.running = False
        print("Autonomous Humanoid System stopped")

    def process_command(self, command: str, image: np.ndarray = None) -> VLAPipelineResult:
        """
        Process a single command through the VLA pipeline.

        Args:
            command: Natural language command
            image: Image from robot's camera (optional)

        Returns:
            VLAPipelineResult with execution details
        """
        print(f"Processing command: {command}")
        self.system_state['last_command'] = command
        self.system_state['current_mode'] = 'processing'

        # Execute the pipeline
        result = self.vla_pipeline.execute_with_feedback(command, image)

        # Update system state based on result
        if result.success:
            self.system_state['current_mode'] = 'idle'
        else:
            self.system_state['current_mode'] = 'error'

        return result

    def run_continuous(self):
        """Run the system in continuous mode, processing commands from a queue"""
        self.start()

        while self.running:
            if self.command_queue:
                command, image = self.command_queue.pop(0)
                result = self.process_command(command, image)

                print(f"Command '{command}' execution: {'Success' if result.success else 'Failed'}")

            time.sleep(0.1)  # Small delay to prevent busy waiting

        self.stop()

    def add_command(self, command: str, image: np.ndarray = None):
        """Add a command to the processing queue"""
        self.command_queue.append((command, image))


# Example usage and testing
if __name__ == "__main__":
    # Create the VLA pipeline
    vla_pipeline = VLAPipeline()

    # Mock image (in a real system, this would come from a camera)
    mock_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)

    # Test commands
    test_commands = [
        "Go to the kitchen",
        "Pick up the red cup",
        "Stop moving",
        "Say hello to everyone"
    ]

    print("Testing Sequential Pipeline:")
    for cmd in test_commands:
        print(f"\n--- Processing: {cmd} ---")
        result = vla_pipeline.execute_sequential_pipeline(cmd, mock_image)
        print(f"Success: {result.success}")
        print(f"Execution time: {result.execution_time:.2f}s")
        if result.error_message:
            print(f"Error: {result.error_message}")

    print("\n" + "="*50)
    print("Testing Autonomous Humanoid System:")

    # Create autonomous system
    auto_system = AutonomousHumanoidSystem()

    # Add some commands to the queue
    auto_system.add_command("Go to the table", mock_image)
    auto_system.add_command("Find the cup", mock_image)
    auto_system.add_command("Wait for 2 seconds", mock_image)

    # Process commands one by one (without running continuously for this demo)
    for _ in range(3):
        if auto_system.command_queue:
            cmd, img = auto_system.command_queue.pop(0)
            result = auto_system.process_command(cmd, img)
            print(f"Processed: {cmd} -> {'Success' if result.success else 'Failed'}")

    # Print pipeline statistics
    stats = vla_pipeline.get_pipeline_statistics()
    print(f"\nPipeline Statistics:")
    print(f"Total executions: {stats['total_executions']}")
    print(f"Success rate: {stats['success_rate']:.2%}")
    print(f"Average execution time: {stats['avg_execution_time']:.2f}s")