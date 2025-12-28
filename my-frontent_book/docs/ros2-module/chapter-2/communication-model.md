---
sidebar_position: 3
---

# Chapter 2: ROS 2 Communication Model

## Learning Goals
- Master ROS 2 communication primitives (Nodes, Topics, Services, Actions)
- Understand sensor-actuator data flow patterns
- Apply communication patterns to humanoid control use-cases

## Core Communication Primitives

ROS 2 provides four main communication patterns that form the backbone of robot systems:

### 1. Nodes
Nodes are the fundamental building blocks of a ROS 2 system. Each node is a process that performs computation and communicates with other nodes through messages.

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node_name')
        # Node initialization code here
```

### 2. Topics (Publish/Subscribe)
Topics enable asynchronous, one-way communication through a publish/subscribe pattern. This is ideal for sensor data streams and continuous actuator commands.

```python
# Publisher example
self.publisher = self.create_publisher(String, 'sensor_data', 10)

# Subscriber example
self.subscriber = self.create_subscription(
    String,
    'actuator_commands',
    self.listener_callback,
    10
)
```

### 3. Services (Request/Response)
Services provide synchronous, two-way communication for request/response interactions. This is useful for configuration changes or commands that require confirmation.

```python
# Service server
self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

# Service client
self.cli = self.create_client(AddTwoInts, 'add_two_ints')
```

### 4. Actions (Goal/Cancel/Feedback/Result)
Actions are used for long-running tasks that require feedback and the ability to cancel. Perfect for complex movements or navigation tasks.

```python
# Action server
self._action_server = ActionServer(
    self,
    Fibonacci,
    'fibonacci',
    self.execute_callback
)

# Action client
self._action_client = ActionClient(self, Fibonacci, 'fibonacci')
```

## Quality of Service (QoS) Settings

QoS profiles allow you to fine-tune the communication behavior based on your application's requirements:

```python
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# For sensor data requiring reliability
qos_profile = QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST
)

# For real-time actuator commands
realtime_qos = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST
)
```

## Custom Message Types

For humanoid control applications, you'll often need custom message types. Create them in a `msg` directory within your package:

```python
# JointState.msg
float64[] positions
float64[] velocities
float64[] efforts
string[] joint_names

# HumanoidCommand.msg
string command_type  # "move", "grip", "balance", etc.
float64[] joint_targets
float64[] joint_velocities
bool emergency_stop
```

## Sensor-Actuator Data Flow Patterns

### Pattern 1: Direct Control Loop
```
Sensors → AI Decision Node → Actuators
```
Ideal for real-time control where sensor data directly influences actuator commands.

### Pattern 2: State Estimation
```
Sensors → State Estimator → AI Decision → Actuators
```
For complex systems requiring state estimation before decision making.

### Pattern 3: Hierarchical Control
```
High-level Planner → Mid-level Controller → Low-level Drivers → Actuators
```
For complex humanoid robots with multiple control layers.

## Humanoid Control Use-cases

### Joint Control
- Publishing joint position/velocity/effort commands
- Subscribing to joint state feedback
- Implementing PID controllers for precise movement

### Sensor Fusion
- Combining data from multiple sensors (IMU, cameras, LIDAR)
- Creating unified state representations
- Handling sensor failures gracefully

### Safety Systems
- Emergency stop mechanisms
- Joint limit enforcement
- Collision avoidance

## Implementation Example: Sensor-Actuator Communication

Here's a complete example of a simple sensor-actuator communication pattern for humanoid control:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from builtin_interfaces.msg import Duration

class HumanoidController(Node):
    def __init__(self):
        super().__init__('humanoid_controller')

        # Subscribe to joint states from robot
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Publish commands to joints
        self.joint_cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/joint_commands',
            10
        )

        # Timer for control loop
        self.control_timer = self.create_timer(0.01, self.control_loop)  # 100Hz

        self.current_joint_states = JointState()

    def joint_state_callback(self, msg):
        self.current_joint_states = msg

    def control_loop(self):
        # Simple PD controller example
        cmd_msg = Float64MultiArray()

        # Calculate desired joint positions based on some logic
        # This would typically come from an AI decision node
        desired_positions = self.calculate_desired_positions()

        # Apply control algorithm
        cmd_msg.data = self.apply_pd_control(desired_positions)

        self.joint_cmd_pub.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    controller = HumanoidController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Summary

This chapter covered the essential communication patterns in ROS 2. You've learned about nodes, topics, services, and actions, and how to apply them to humanoid control systems. In the next chapter, we'll explore how to integrate Python AI agents with these communication patterns.