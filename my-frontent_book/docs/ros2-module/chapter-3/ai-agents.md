---
sidebar_position: 4
---

# Chapter 3: Python AI Agents with ROS 2

## Learning Goals
- Use rclpy (ROS 2 Python client library) effectively
- Bridge AI logic to ROS controllers
- Implement message flow and design patterns

## Introduction to rclpy

rclpy is the Python client library for ROS 2, providing the interface between Python code and the ROS 2 middleware. It allows you to create nodes, publish and subscribe to topics, provide and use services, and create and use actions.

### Basic Node Structure

```python
import rclpy
from rclpy.node import Node

class AIAgentNode(Node):
    def __init__(self):
        super().__init__('ai_agent_node')

        # Initialize AI components
        self.initialize_ai_components()

        # Set up ROS 2 communication
        self.setup_ros_communication()

        # Start timers for AI processing
        self.ai_timer = self.create_timer(0.1, self.ai_processing_callback)

    def initialize_ai_components(self):
        # Initialize your AI models, decision trees, etc.
        pass

    def setup_ros_communication(self):
        # Set up publishers, subscribers, services, etc.
        pass

    def ai_processing_callback(self):
        # Main AI processing loop
        pass
```

## Integration of AI Algorithms with ROS 2

### Sensor Data Processing

AI agents need to process sensor data to make informed decisions. Here's how to integrate sensor data processing:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu, Image
import numpy as np

class SensorProcessingAIAgent(Node):
    def __init__(self):
        super().__init__('sensor_ai_agent')

        # Subscribe to various sensor topics
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10
        )
        self.camera_sub = self.create_subscription(
            Image, '/camera/image_raw', self.camera_callback, 10
        )

        # Publisher for AI decisions
        self.decision_pub = self.create_publisher(
            String, '/ai_decisions', 10
        )

        # Store sensor data
        self.joint_data = None
        self.imu_data = None
        self.camera_data = None

        # AI processing timer
        self.ai_timer = self.create_timer(0.05, self.process_sensors_and_decide)

    def joint_callback(self, msg):
        self.joint_data = msg

    def imu_callback(self, msg):
        self.imu_data = msg

    def camera_callback(self, msg):
        self.camera_data = msg

    def process_sensors_and_decide(self):
        if self.joint_data and self.imu_data:
            # Process sensor data with AI algorithms
            decision = self.ai_decision_algorithm()

            # Publish decision
            decision_msg = String()
            decision_msg.data = decision
            self.decision_pub.publish(decision_msg)

    def ai_decision_algorithm(self):
        # Implement your AI decision logic here
        # This could be a neural network, decision tree, etc.
        return "move_forward"
```

## Asynchronous Programming Patterns in ROS 2

ROS 2 is designed to work well with Python's asynchronous programming model:

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import asyncio

class AsyncAIAgent(Node):
    def __init__(self):
        super().__init__('async_ai_agent')

        # Setup communication
        self.setup_communication()

        # Create async tasks
        self.async_tasks = []

    def setup_communication(self):
        self.subscriber = self.create_subscription(
            String, 'sensor_data', self.async_sensor_callback, 10
        )

        self.publisher = self.create_publisher(String, 'ai_output', 10)

    def async_sensor_callback(self, msg):
        # Schedule async processing
        future = asyncio.run_coroutine_threadsafe(
            self.process_sensor_async(msg),
            self.async_loop
        )
        future.add_done_callback(self.on_processing_complete)

    async def process_sensor_async(self, sensor_msg):
        # Simulate async AI processing
        await asyncio.sleep(0.01)  # Simulated processing time
        result = self.run_ai_model(sensor_msg.data)
        return result

    def on_processing_complete(self, future):
        try:
            result = future.result()
            # Publish the result
            output_msg = String()
            output_msg.data = result
            self.publisher.publish(output_msg)
        except Exception as e:
            self.get_logger().error(f'Async processing failed: {e}')
```

## Error Handling and Node Recovery

Robust AI agents need proper error handling and recovery mechanisms:

```python
import rclpy
from rclpy.node import Node
from rclpy.exceptions import ParameterNotDeclaredException
from std_msgs.msg import String
import traceback

class RobustAIAgent(Node):
    def __init__(self):
        super().__init__('robust_ai_agent')

        # Setup error handling
        self.setup_error_handling()

        # Initialize AI components with error handling
        self.ai_model = None
        self.initialize_ai_model_with_recovery()

        # Setup communication with error handling
        self.setup_communication_with_validation()

    def setup_error_handling(self):
        # Set up logging
        self.get_logger().info('Initializing error handling...')

        # Set up parameter validation
        self.declare_parameter('model_path', '/default/model/path')

    def initialize_ai_model_with_recovery(self):
        try:
            model_path = self.get_parameter('model_path').value
            self.ai_model = self.load_ai_model(model_path)
            self.get_logger().info('AI model loaded successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to load AI model: {e}')
            self.get_logger().info('Initializing fallback model...')
            self.ai_model = self.create_fallback_model()

    def load_ai_model(self, path):
        # Load your actual AI model
        # This is a placeholder - implement based on your AI framework
        return {"type": "neural_network", "path": path}

    def create_fallback_model(self):
        # Create a simple fallback model
        return {"type": "fallback", "behavior": "safe_mode"}

    def setup_communication_with_validation(self):
        try:
            self.sensor_sub = self.create_subscription(
                String, 'sensor_input', self.safe_sensor_callback, 10
            )
            self.command_pub = self.create_publisher(String, 'ai_commands', 10)
        except Exception as e:
            self.get_logger().error(f'Setup communication failed: {e}')
            raise

    def safe_sensor_callback(self, msg):
        try:
            # Process message with error handling
            result = self.process_with_ai_model(msg)
            self.publish_command(result)
        except Exception as e:
            self.get_logger().error(f'Error in sensor callback: {e}')
            self.publish_emergency_stop()
            traceback.print_exc()

    def process_with_ai_model(self, sensor_msg):
        if self.ai_model is None:
            return "safe_mode"

        # Apply AI logic
        if self.ai_model['type'] == 'fallback':
            return self.fallback_decision_logic(sensor_msg)
        else:
            return self.neural_network_decision(sensor_msg)

    def fallback_decision_logic(self, sensor_msg):
        # Simple decision logic for fallback mode
        return "safe_move"

    def neural_network_decision(self, sensor_msg):
        # Complex decision logic using neural network
        return "calculated_move"

    def publish_command(self, command):
        cmd_msg = String()
        cmd_msg.data = command
        self.command_pub.publish(cmd_msg)

    def publish_emergency_stop(self):
        cmd_msg = String()
        cmd_msg.data = "emergency_stop"
        self.command_pub.publish(cmd_msg)
```

## Performance Considerations for Real-time AI

For real-time humanoid control, performance is critical:

```python
import rclpy
from rclpy.node import Node
import time
import threading
from collections import deque
import numpy as np

class HighPerformanceAIAgent(Node):
    def __init__(self):
        super().__init__('high_performance_ai_agent')

        # Optimize for real-time performance
        self.setup_real_time_optimizations()

        # Efficient data structures
        self.sensor_buffer = deque(maxlen=10)  # Circular buffer
        self.processing_times = deque(maxlen=100)

        # Threading for heavy computations
        self.ai_thread = None
        self.ai_thread_lock = threading.Lock()

        # Setup communication
        self.setup_communication()

    def setup_real_time_optimizations(self):
        # Use efficient QoS profiles for real-time
        from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

        self.real_time_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST
        )

    def setup_communication(self):
        # High-frequency sensor subscription
        self.sensor_sub = self.create_subscription(
            String, 'high_freq_sensor', self.high_freq_callback,
            self.real_time_qos
        )

        # High-frequency command publication
        self.cmd_pub = self.create_publisher(
            String, 'high_freq_commands', self.real_time_qos
        )

        # Real-time processing timer (1kHz)
        self.real_time_timer = self.create_timer(0.001, self.real_time_processing)

    def high_freq_callback(self, msg):
        # Add to buffer quickly
        self.sensor_buffer.append({
            'timestamp': time.time(),
            'data': msg.data
        })

    def real_time_processing(self):
        start_time = time.time()

        # Process sensor data
        if self.sensor_buffer:
            latest_data = self.sensor_buffer[-1]
            command = self.fast_ai_decision(latest_data)

            # Publish command
            cmd_msg = String()
            cmd_msg.data = command
            self.cmd_pub.publish(cmd_msg)

        # Track processing time
        end_time = time.time()
        processing_time = (end_time - start_time) * 1000  # in milliseconds
        self.processing_times.append(processing_time)

        # Log if processing is too slow
        avg_processing_time = np.mean(self.processing_times) if self.processing_times else 0
        if avg_processing_time > 1.0:  # More than 1ms average
            self.get_logger().warn(f'AI processing is slow: {avg_processing_time:.2f}ms avg')

    def fast_ai_decision(self, sensor_data):
        # Optimized AI decision making
        # Use lightweight models or precomputed decisions
        # Avoid heavy computations in real-time loop
        return "fast_command"
```

## Simple AI Integration Examples

### Decision Tree Example

```python
from sklearn.tree import DecisionTreeClassifier
import numpy as np

class DecisionTreeAIAgent(Node):
    def __init__(self):
        super().__init__('decision_tree_agent')

        # Train a simple decision tree
        self.train_decision_tree()

    def train_decision_tree(self):
        # Example: train on joint positions to determine movement type
        X = np.array([
            [0.0, 0.0, 0.0],  # joint positions: standing
            [0.5, -0.5, 0.0], # joint positions: walking
            [1.0, -1.0, 0.5], # joint positions: reaching
        ])
        y = np.array(['stand', 'walk', 'reach'])

        self.decision_tree = DecisionTreeClassifier()
        self.decision_tree.fit(X, y)

    def make_decision(self, joint_positions):
        # Reshape for prediction
        features = np.array(joint_positions).reshape(1, -1)
        prediction = self.decision_tree.predict(features)[0]
        confidence = max(self.decision_tree.predict_proba(features)[0])

        return prediction, confidence
```

### Neural Network Example (using PyTorch)

```python
# Note: You'll need to install torch: pip install torch
try:
    import torch
    import torch.nn as nn
    import torch.nn.functional as F

    class SimpleNN(nn.Module):
        def __init__(self, input_size, hidden_size, output_size):
            super(SimpleNN, self).__init__()
            self.fc1 = nn.Linear(input_size, hidden_size)
            self.fc2 = nn.Linear(hidden_size, hidden_size)
            self.fc3 = nn.Linear(hidden_size, output_size)

        def forward(self, x):
            x = F.relu(self.fc1(x))
            x = F.relu(self.fc2(x))
            x = self.fc3(x)
            return x

    class NeuralNetworkAIAgent(Node):
        def __init__(self):
            super().__init__('nn_agent')

            # Initialize neural network
            self.nn = SimpleNN(input_size=12, hidden_size=64, output_size=6)  # Example sizes

            # Load pre-trained weights if available
            # self.nn.load_state_dict(torch.load('model_weights.pth'))

        def process_sensor_data(self, sensor_data):
            # Convert ROS message to tensor
            input_tensor = torch.tensor(sensor_data, dtype=torch.float32)

            # Get prediction
            with torch.no_grad():
                output = self.nn(input_tensor)
                probabilities = torch.softmax(output, dim=0)
                predicted_class = torch.argmax(probabilities).item()

            return predicted_class, probabilities.numpy()

except ImportError:
    print("PyTorch not available, skipping neural network example")
```

## Summary

This chapter covered how to integrate AI algorithms with ROS 2 using Python. You've learned about rclpy, sensor data processing, asynchronous programming patterns, error handling, and performance considerations for real-time AI. In the next chapter, we'll explore humanoid modeling with URDF.