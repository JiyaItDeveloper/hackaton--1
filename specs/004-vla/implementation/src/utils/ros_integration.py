"""
ROS 2 integration utilities for the VLA system.
Provides interfaces for communication with ROS 2 nodes and services.
"""
from typing import Dict, Any, Optional, Callable
import threading
import time
from dataclasses import dataclass


@dataclass
class ROSMessage:
    """Base class for ROS messages"""
    msg_type: str
    data: Dict[str, Any]
    timestamp: float


class ROSPublisher:
    """Simulated ROS publisher for VLA system messages"""

    def __init__(self, topic_name: str, msg_type: str):
        self.topic_name = topic_name
        self.msg_type = msg_type
        self.message_queue = []
        self.subscribers = []
        self.lock = threading.Lock()

    def publish(self, data: Dict[str, Any]):
        """Publish a message to the topic"""
        msg = ROSMessage(
            msg_type=self.msg_type,
            data=data,
            timestamp=time.time()
        )

        with self.lock:
            self.message_queue.append(msg)

        # Notify subscribers (in a real system, this would send to actual subscribers)
        for callback in self.subscribers:
            try:
                callback(msg)
            except Exception as e:
                print(f"Error in subscriber callback: {e}")

    def add_subscriber(self, callback: Callable[[ROSMessage], None]):
        """Add a subscriber to this publisher"""
        self.subscribers.append(callback)


class ROSSubscriber:
    """Simulated ROS subscriber for VLA system messages"""

    def __init__(self, topic_name: str, msg_type: str, callback: Callable[[Dict[str, Any]], None]):
        self.topic_name = topic_name
        self.msg_type = msg_type
        self.callback = callback
        self.message_buffer = []
        self.lock = threading.Lock()

    def receive_message(self, msg: ROSMessage):
        """Receive a message from a publisher"""
        if msg.msg_type == self.msg_type:
            with self.lock:
                self.message_buffer.append(msg)

            # Process the message
            self.callback(msg.data)


class ROSNode:
    """Base class for ROS nodes in the VLA system"""

    def __init__(self, node_name: str):
        self.node_name = node_name
        self.publishers = {}
        self.subscribers = {}
        self.services = {}
        self.action_servers = {}
        self.action_clients = {}

    def create_publisher(self, topic_name: str, msg_type: str):
        """Create a publisher for the node"""
        publisher = ROSPublisher(topic_name, msg_type)
        self.publishers[topic_name] = publisher
        return publisher

    def create_subscriber(self, topic_name: str, msg_type: str, callback: Callable[[Dict[str, Any]], None]):
        """Create a subscriber for the node"""
        subscriber = ROSSubscriber(topic_name, msg_type, callback)
        self.subscribers[topic_name] = subscriber
        return subscriber

    def get_publisher(self, topic_name: str) -> Optional[ROSPublisher]:
        """Get a publisher by topic name"""
        return self.publishers.get(topic_name)

    def get_subscriber(self, topic_name: str) -> Optional[ROSSubscriber]:
        """Get a subscriber by topic name"""
        return self.subscribers.get(topic_name)


class VoiceCommandNode(ROSNode):
    """ROS node for processing voice commands"""

    def __init__(self, node_name: str = "voice_command_node"):
        super().__init__(node_name)
        self.voice_processor = None  # Will be set externally
        self.command_queue = []
        self.is_running = False

        # Create publishers and subscribers
        self.stt_publisher = self.create_publisher('/stt_output', 'String')
        self.nav_goal_publisher = self.create_publisher('/goal_pose', 'PoseStamped')
        self.action_publisher = self.create_publisher('/robot_action', 'String')
        self.status_publisher = self.create_publisher('/voice_status', 'String')

        # Subscribe to audio input
        self.audio_subscriber = self.create_subscriber('/audio', 'AudioData', self.audio_callback)

    def set_voice_processor(self, processor):
        """Set the voice command processor"""
        self.voice_processor = processor

    def audio_callback(self, audio_data: Dict[str, Any]):
        """Callback for audio input"""
        if self.voice_processor:
            # Process the audio command
            result = self.voice_processor.process_voice_command(audio_data.get('data', []))

            if result['success']:
                # Publish the transcription
                self.stt_publisher.publish({
                    'data': result['transcription']
                })

                # Publish status
                self.status_publisher.publish({
                    'status': 'command_processed',
                    'command': result['transcription'],
                    'confidence': result['confidence']
                })

                # Process the intent and publish appropriate action
                self._process_intent(result['parsed_command'])
            else:
                self.status_publisher.publish({
                    'status': 'error',
                    'error': result['error']
                })

    def _process_intent(self, parsed_command: Dict[str, Any]):
        """Process the parsed command and publish appropriate actions"""
        cmd_type = parsed_command['type']
        action = parsed_command['action']
        params = parsed_command['parameters']

        if cmd_type == 'navigation':
            self._publish_navigation_command(params)
        elif cmd_type == 'manipulation':
            self._publish_manipulation_command(params)
        elif cmd_type == 'action':
            self._publish_action_command(params)
        elif cmd_type == 'query':
            self._publish_query_command(params)

    def _publish_navigation_command(self, params: Dict[str, Any]):
        """Publish navigation command"""
        target_location = params.get('target_location', '')
        # In a real system, this would publish to the navigation stack
        self.nav_goal_publisher.publish({
            'target_location': target_location,
            'frame_id': 'map',
            'goal': self._get_location_coordinates(target_location)
        })

    def _publish_manipulation_command(self, params: Dict[str, Any]):
        """Publish manipulation command"""
        target_object = params.get('target_object', '')
        self.action_publisher.publish({
            'action_type': 'manipulation',
            'target_object': target_object
        })

    def _publish_action_command(self, params: Dict[str, Any]):
        """Publish general action command"""
        self.action_publisher.publish({
            'action_type': 'general',
            'command': params.get('original_command', '')
        })

    def _publish_query_command(self, params: Dict[str, Any]):
        """Publish query command"""
        self.action_publisher.publish({
            'action_type': 'query_response',
            'query_target': params.get('target', '')
        })

    def _get_location_coordinates(self, location_name: str) -> Dict[str, float]:
        """Get coordinates for a named location (mock implementation)"""
        locations = {
            'kitchen': {'x': 5.0, 'y': 5.0, 'theta': 0.0},
            'living room': {'x': 0.0, 'y': 0.0, 'theta': 0.0},
            'bedroom': {'x': -3.0, 'y': 4.0, 'theta': 0.0},
            'office': {'x': 2.0, 'y': -3.0, 'theta': 0.0}
        }

        return locations.get(location_name.lower(), {'x': 0.0, 'y': 0.0, 'theta': 0.0})


class NavigationNode(ROSNode):
    """ROS node for navigation functionality"""

    def __init__(self, node_name: str = "navigation_node"):
        super().__init__(node_name)
        self.current_position = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.is_navigating = False

        # Subscribe to navigation goals
        self.nav_goal_subscriber = self.create_subscriber('/goal_pose', 'PoseStamped', self.nav_goal_callback)

        # Publish navigation status
        self.nav_status_publisher = self.create_publisher('/nav_status', 'String')

    def nav_goal_callback(self, goal_data: Dict[str, Any]):
        """Callback for navigation goals"""
        target = goal_data.get('goal', {})
        location_name = goal_data.get('target_location', 'unknown')

        self.nav_status_publisher.publish({
            'status': 'navigating',
            'target': location_name,
            'coordinates': target
        })

        # Simulate navigation
        self._simulate_navigation(target)

        self.nav_status_publisher.publish({
            'status': 'arrived',
            'target': location_name,
            'coordinates': target
        })

    def _simulate_navigation(self, target: Dict[str, float]):
        """Simulate navigation to target coordinates"""
        print(f"Navigating to {target}")
        # Simulate navigation time
        time.sleep(2)  # 2 seconds for simulation
        self.current_position = target


class ActionNode(ROSNode):
    """ROS node for action execution"""

    def __init__(self, node_name: str = "action_node"):
        super().__init__(node_name)
        self.action_subscriber = self.create_subscriber('/robot_action', 'String', self.action_callback)
        self.action_status_publisher = self.create_publisher('/action_status', 'String')

    def action_callback(self, action_data: Dict[str, Any]):
        """Callback for action commands"""
        action_type = action_data.get('action_type', 'unknown')
        target_object = action_data.get('target_object')
        command = action_data.get('command')

        self.action_status_publisher.publish({
            'status': 'executing',
            'action_type': action_type
        })

        # Execute the action based on type
        if action_type == 'manipulation' and target_object:
            self._execute_manipulation(target_object)
        elif action_type == 'general' and command:
            self._execute_general_command(command)
        elif action_type == 'query_response':
            self._execute_query_response(action_data)

        self.action_status_publisher.publish({
            'status': 'completed',
            'action_type': action_type
        })

    def _execute_manipulation(self, target_object: str):
        """Execute manipulation action"""
        print(f"Manipulating object: {target_object}")
        time.sleep(1)  # Simulate manipulation time

    def _execute_general_command(self, command: str):
        """Execute general command"""
        print(f"Executing command: {command}")
        time.sleep(0.5)  # Simulate execution time

    def _execute_query_response(self, action_data: Dict[str, Any]):
        """Execute query response"""
        query_target = action_data.get('query_target', 'unknown')
        print(f"Responding to query about: {query_target}")
        time.sleep(0.5)  # Simulate response time


# Example usage
if __name__ == "__main__":
    print("Testing ROS Integration...")

    # Create nodes
    voice_node = VoiceCommandNode()
    nav_node = NavigationNode()
    action_node = ActionNode()

    # Create a mock voice processor (using the Whisper STT node from the language module)
    try:
        from ..language.whisper_stt import WhisperSTTNode, VoiceCommandProcessor
        stt_node = WhisperSTTNode()
        voice_processor = VoiceCommandProcessor(stt_node)
        voice_node.set_voice_processor(voice_processor)

        # Simulate receiving an audio command
        mock_audio_data = {
            'data': [0.1, -0.2, 0.3, -0.1, 0.4]  # Mock audio data
        }

        # Process the audio command
        voice_node.audio_callback(mock_audio_data)
    except ImportError:
        print("Could not import language modules, skipping voice processor test")

    print("ROS integration test completed!")