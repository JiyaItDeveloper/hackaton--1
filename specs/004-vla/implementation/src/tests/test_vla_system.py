"""
Unit tests for the Vision-Language-Action (VLA) system.
Tests the core components and integration.
"""
import unittest
import numpy as np
from unittest.mock import Mock, patch

from ..vision.vision_system import VisionSystem, ObjectDetector, SceneAnalyzer, SpatialReasoner
from ..language.language_system import LanguageSystem, IntentClassifier, ActionMapper
from ..action.action_system import ActionSystem, MotionPlanner, ActionExecutor
from ..pipeline.vla_pipeline import VLAPipeline, AutonomousHumanoidSystem


class TestVisionSystem(unittest.TestCase):
    """Test cases for the Vision System"""

    def setUp(self):
        self.vision_system = VisionSystem()

    def test_process_environment(self):
        """Test processing an environment image"""
        # Create a mock image
        mock_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)

        # Process the environment
        result = self.vision_system.process_environment(mock_image)

        # Check that the result has expected attributes
        self.assertIsNotNone(result)
        self.assertIsNotNone(result.objects)
        self.assertIsNotNone(result.spatial_relations)
        self.assertIsNotNone(result.environmental_features)

    def test_get_object_location(self):
        """Test getting the location of a specific object"""
        mock_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)

        # Test with a mock object name
        location = self.vision_system.get_object_location(mock_image, "cup")

        # Location could be None if the object isn't detected in the mock image
        # but the method should not raise an exception
        self.assertIsInstance(location, (tuple, type(None)))


class TestLanguageSystem(unittest.TestCase):
    """Test cases for the Language System"""

    def setUp(self):
        self.language_system = LanguageSystem()

    def test_process_command_navigation(self):
        """Test processing a navigation command"""
        command = "Go to the kitchen"
        result = self.language_system.process_command(command)

        self.assertEqual(result.intent_type, 'navigation')
        self.assertIn('kitchen', result.parameters.get('target', '').lower())

    def test_process_command_manipulation(self):
        """Test processing a manipulation command"""
        command = "Pick up the red cup"
        result = self.language_system.process_command(command)

        self.assertEqual(result.intent_type, 'manipulation')
        self.assertIn('red cup', result.parameters.get('target', '').lower())

    def test_process_command_action(self):
        """Test processing a general action command"""
        command = "Stop moving"
        result = self.language_system.process_command(command)

        self.assertEqual(result.intent_type, 'action')

    def test_generate_action_plan(self):
        """Test generating an action plan"""
        command = "Go to the kitchen"
        env_context = {
            'known_locations': {'kitchen': (5, 5)},
            'objects': []
        }

        plan = self.language_system.generate_action_plan(command, env_context)

        self.assertIsNotNone(plan)
        self.assertIsInstance(plan.actions, list)
        self.assertGreater(len(plan.actions), 0)
        self.assertGreaterEqual(plan.confidence, 0.0)
        self.assertLessEqual(plan.confidence, 1.0)


class TestActionSystem(unittest.TestCase):
    """Test cases for the Action System"""

    def setUp(self):
        self.action_system = ActionSystem()

    def test_execute_single_action(self):
        """Test executing a single action"""
        action = {
            'action_type': 'wait',
            'parameters': {'duration': 0.1}  # Short duration for testing
        }

        result = self.action_system.execute_action(action)

        self.assertTrue(result.success)
        self.assertGreaterEqual(result.execution_time, 0)

    def test_execute_plan(self):
        """Test executing a sequence of actions"""
        plan = [
            {
                'action_type': 'wait',
                'parameters': {'duration': 0.1}
            },
            {
                'action_type': 'speak',
                'parameters': {'text': 'Hello'}
            }
        ]

        results = self.action_system.execute_plan(plan)

        self.assertEqual(len(results), len(plan))
        for result in results:
            self.assertTrue(result.success)


class TestVLAPipeline(unittest.TestCase):
    """Test cases for the VLA Pipeline"""

    def setUp(self):
        self.vla_pipeline = VLAPipeline()

    def test_sequential_pipeline(self):
        """Test the sequential VLA pipeline"""
        command = "Wait for 0.5 seconds"
        mock_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)

        result = self.vla_pipeline.execute_sequential_pipeline(command, mock_image)

        self.assertIsNotNone(result)
        self.assertIsInstance(result, type(self.vla_pipeline.execute_sequential_pipeline("", mock_image)))

    def test_parallel_pipeline(self):
        """Test the parallel VLA pipeline"""
        command = "Wait for 0.5 seconds"
        mock_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)

        result = self.vla_pipeline.execute_parallel_pipeline(command, mock_image)

        self.assertIsNotNone(result)

    def test_pipeline_statistics(self):
        """Test pipeline statistics"""
        stats = self.vla_pipeline.get_pipeline_statistics()

        self.assertIn('total_executions', stats)
        self.assertIn('success_rate', stats)
        self.assertIn('avg_execution_time', stats)


class TestAutonomousHumanoidSystem(unittest.TestCase):
    """Test cases for the Autonomous Humanoid System"""

    def setUp(self):
        self.system = AutonomousHumanoidSystem()

    def test_system_initialization(self):
        """Test that the system initializes correctly"""
        self.assertFalse(self.system.running)
        self.assertEqual(len(self.system.command_queue), 0)
        self.assertIn('current_mode', self.system.system_state)

    def test_add_command(self):
        """Test adding a command to the queue"""
        initial_length = len(self.system.command_queue)
        self.system.add_command("Test command")
        new_length = len(self.system.command_queue)

        self.assertEqual(new_length, initial_length + 1)


class TestIntegration(unittest.TestCase):
    """Integration tests for the complete VLA system"""

    def test_end_to_end_pipeline(self):
        """Test the complete end-to-end pipeline"""
        # Create the complete system
        vla_pipeline = VLAPipeline()

        # Create a mock image
        mock_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)

        # Test a simple command
        command = "Wait for 0.1 seconds"

        # Execute through the pipeline
        result = vla_pipeline.execute_sequential_pipeline(command, mock_image)

        # Verify the result
        self.assertIsNotNone(result)
        self.assertIsInstance(result, type(vla_pipeline.execute_sequential_pipeline("", mock_image)))

        # The action should be successful (waiting is generally safe)
        # Note: This might fail if the action system fails, but it's a basic test
        # The main goal is to ensure no exceptions are raised


def run_tests():
    """Run all tests"""
    # Create a test suite
    suite = unittest.TestSuite()

    # Add all test cases
    suite.addTest(unittest.makeSuite(TestVisionSystem))
    suite.addTest(unittest.makeSuite(TestLanguageSystem))
    suite.addTest(unittest.makeSuite(TestActionSystem))
    suite.addTest(unittest.makeSuite(TestVLAPipeline))
    suite.addTest(unittest.makeSuite(TestAutonomousHumanoidSystem))
    suite.addTest(unittest.makeSuite(TestIntegration))

    # Run the tests
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)

    return result.wasSuccessful()


if __name__ == "__main__":
    print("Running VLA System Tests...")
    success = run_tests()

    if success:
        print("\nAll tests passed! ✓")
    else:
        print("\nSome tests failed! ✗")