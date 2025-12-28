---
sidebar_position: 6
---

# Integration Project: Complete ROS 2 Humanoid Control System

## Learning Goals
- Combine all concepts from previous chapters
- Create end-to-end system with AI agent controlling humanoid
- Test sensor-actuator data flow patterns
- Debug ROS 2 systems effectively

## Project Overview

In this integration project, we'll build a complete system that combines all the concepts learned in the previous chapters:

1. **ROS 2 Communication**: Using nodes, topics, services, and actions
2. **AI Integration**: Python AI agents processing sensor data
3. **Humanoid Modeling**: URDF model with proper joint control
4. **Real-time Control**: Sensor-actuator loops with safety mechanisms

## System Architecture

The complete system will have the following architecture:

```
[Humanoid Robot Model (URDF)]
         ↓
[Robot State Publisher] ← [Joint State Publisher GUI]
         ↓
[Sensor Data] → [AI Decision Node] → [Actuator Commands]
    ↓              ↓                    ↓
[IMU Data]    [AI Processing]      [Joint Controllers]
[Camera Data] [Decision Trees]     [Position/Vel/Effort]
[Joint States] [Neural Networks]    [Safety Controllers]
```

## Implementation: Complete Control System

### 1. Main Control Node

Let's create the main control node that coordinates all system components:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu, Image
from std_msgs.msg import String, Float64MultiArray
from geometry_msgs.msg import Twist
import numpy as np
import threading
import time

class HumanoidControlSystem(Node):
    def __init__(self):
        super().__init__('humanoid_control_system')

        # Initialize all system components
        self.initialize_sensors()
        self.initialize_ai_agent()
        self.initialize_actuators()
        self.initialize_safety_system()

        # Control loop timer
        self.control_timer = self.create_timer(0.01, self.control_loop)  # 100Hz

        # Data storage
        self.robot_state = {
            'joint_states': None,
            'imu_data': None,
            'camera_data': None,
            'ai_decision': None
        }

        self.get_logger().info('Humanoid Control System initialized')

    def initialize_sensors(self):
        """Initialize all sensor subscriptions"""
        # Joint states from robot
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10
        )

        # IMU data for balance
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10
        )

        # Camera data for vision processing
        self.camera_sub = self.create_subscription(
            Image, '/camera/image_raw', self.camera_callback, 10
        )

    def initialize_ai_agent(self):
        """Initialize AI decision making components"""
        # For this example, we'll use a simple decision maker
        # In practice, this could be a neural network, decision tree, etc.
        self.ai_decision_publisher = self.create_publisher(
            String, '/ai_decisions', 10
        )

    def initialize_actuators(self):
        """Initialize actuator publishers"""
        # Joint command publisher
        self.joint_cmd_pub = self.create_publisher(
            Float64MultiArray, '/joint_commands', 10
        )

        # Twist command for base movement (if applicable)
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/cmd_vel', 10
        )

    def initialize_safety_system(self):
        """Initialize safety monitoring"""
        self.safety_enabled = True
        self.emergency_stop_active = False
        self.safety_timer = self.create_timer(0.1, self.safety_check)

    def joint_callback(self, msg):
        """Handle joint state updates"""
        self.robot_state['joint_states'] = msg
        self.get_logger().debug(f'Received joint states for {len(msg.name)} joints')

    def imu_callback(self, msg):
        """Handle IMU data for balance"""
        self.robot_state['imu_data'] = msg
        self.get_logger().debug(f'IMU: linear_acceleration=({msg.linear_acceleration.x:.2f}, {msg.linear_acceleration.y:.2f}, {msg.linear_acceleration.z:.2f})')

    def camera_callback(self, msg):
        """Handle camera data"""
        self.robot_state['camera_data'] = msg
        self.get_logger().debug(f'Camera data received: {msg.height}x{msg.width}')

    def control_loop(self):
        """Main control loop running at 100Hz"""
        if not self.safety_check_ok():
            self.emergency_stop()
            return

        # Process sensor data
        sensor_data = self.aggregate_sensor_data()

        # Make AI decision based on sensor data
        ai_decision = self.make_ai_decision(sensor_data)

        # Convert decision to actuator commands
        commands = self.decision_to_commands(ai_decision, sensor_data)

        # Publish commands to robot
        self.publish_commands(commands)

    def aggregate_sensor_data(self):
        """Aggregate all sensor data into a unified format"""
        sensor_data = {}

        if self.robot_state['joint_states']:
            sensor_data['joint_positions'] = list(self.robot_state['joint_states'].position)
            sensor_data['joint_velocities'] = list(self.robot_state['joint_states'].velocity)
            sensor_data['joint_names'] = list(self.robot_state['joint_states'].name)

        if self.robot_state['imu_data']:
            sensor_data['imu'] = {
                'linear_acceleration': [
                    self.robot_state['imu_data'].linear_acceleration.x,
                    self.robot_state['imu_data'].linear_acceleration.y,
                    self.robot_state['imu_data'].linear_acceleration.z
                ],
                'angular_velocity': [
                    self.robot_state['imu_data'].angular_velocity.x,
                    self.robot_state['imu_data'].angular_velocity.y,
                    self.robot_state['imu_data'].angular_velocity.z
                ]
            }

        return sensor_data

    def make_ai_decision(self, sensor_data):
        """Make AI decision based on sensor data"""
        if not sensor_data:
            return "stand_still"

        # Simple balance control example
        if 'imu' in sensor_data:
            # Check if robot is tilted too much
            tilt_threshold = 0.5
            linear_acc = sensor_data['imu']['linear_acceleration']
            tilt_magnitude = np.sqrt(linear_acc[0]**2 + linear_acc[1]**2)

            if tilt_magnitude > tilt_threshold:
                return "balance_correction"

        # Check joint positions for safety
        if 'joint_positions' in sensor_data:
            joint_positions = sensor_data['joint_positions']
            for pos in joint_positions:
                if abs(pos) > 3.0:  # Joint limit exceeded
                    return "joint_safety_stop"

        # Default behavior based on some logic
        return "normal_operation"

    def decision_to_commands(self, decision, sensor_data):
        """Convert AI decision to actuator commands"""
        commands = {
            'joint_commands': [],
            'cmd_vel': None
        }

        if decision == "balance_correction":
            # Implement balance correction logic
            commands['joint_commands'] = self.balance_correction_commands(sensor_data)
        elif decision == "joint_safety_stop":
            # Stop all joint movement
            commands['joint_commands'] = [0.0] * len(sensor_data.get('joint_positions', [0]*12))
        elif decision == "normal_operation":
            # Implement normal operation (walking, etc.)
            commands['joint_commands'] = self.normal_operation_commands(sensor_data)
        else:
            # Default: hold current position
            if 'joint_positions' in sensor_data:
                commands['joint_commands'] = sensor_data['joint_positions']
            else:
                commands['joint_commands'] = [0.0] * 12  # Default 12 joints

        return commands

    def balance_correction_commands(self, sensor_data):
        """Generate commands to correct robot balance"""
        # Simple PD controller for balance
        if 'imu' in sensor_data:
            linear_acc = sensor_data['imu']['linear_acceleration']
            # Simple balance correction based on tilt
            correction = [-0.1 * linear_acc[0], -0.1 * linear_acc[1]] + [0.0] * 10
            return correction
        return [0.0] * 12

    def normal_operation_commands(self, sensor_data):
        """Generate normal operation commands"""
        # For this example, we'll generate a simple walking gait
        # In practice, this would come from a more sophisticated AI system
        current_time = self.get_clock().now().nanoseconds / 1e9
        phase = current_time * 2  # 2 rad/s oscillation

        # Generate simple walking pattern
        commands = []
        for i in range(12):  # 12 joints
            # Different joints have different patterns
            if i < 4:  # Legs
                cmd = 0.5 * np.sin(phase + i * 0.5)
            elif i < 8:  # Arms
                cmd = 0.3 * np.sin(phase * 0.7 + i * 0.3)
            else:  # Head, etc.
                cmd = 0.1 * np.sin(phase * 0.3)
            commands.append(cmd)

        return commands

    def publish_commands(self, commands):
        """Publish actuator commands to robot"""
        if commands['joint_commands']:
            # Publish joint commands
            joint_cmd_msg = Float64MultiArray()
            joint_cmd_msg.data = commands['joint_commands']
            self.joint_cmd_pub.publish(joint_cmd_msg)

        if commands['cmd_vel']:
            # Publish velocity commands if applicable
            self.cmd_vel_pub.publish(commands['cmd_vel'])

    def safety_check_ok(self):
        """Check if system is in safe state"""
        if self.emergency_stop_active:
            return False

        # Check if we have recent sensor data
        # (Implement timeout checks as needed)

        return True

    def safety_check(self):
        """Periodic safety check"""
        if not self.safety_check_ok():
            self.emergency_stop()

    def emergency_stop(self):
        """Emergency stop procedure"""
        if not self.emergency_stop_active:
            self.get_logger().warn('EMERGENCY STOP ACTIVATED')
            self.emergency_stop_active = True

            # Send zero commands to all actuators
            zero_commands = Float64MultiArray()
            zero_commands.data = [0.0] * 12  # 12 joints
            self.joint_cmd_pub.publish(zero_commands)

def main(args=None):
    rclpy.init(args=args)
    control_system = HumanoidControlSystem()

    try:
        rclpy.spin(control_system)
    except KeyboardInterrupt:
        control_system.get_logger().info('Interrupted by user')
    finally:
        control_system.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2. AI Agent Node

Now let's create a dedicated AI agent node that processes sensor data and makes decisions:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import String, Float64MultiArray
import numpy as np
from collections import deque
import pickle

class AdvancedAIAgent(Node):
    def __init__(self):
        super().__init__('advanced_ai_agent')

        # Initialize AI components
        self.initialize_ai_components()

        # Setup communication
        self.setup_communication()

        # Data buffers for temporal processing
        self.joint_buffer = deque(maxlen=10)
        self.imu_buffer = deque(maxlen=10)

    def initialize_ai_components(self):
        """Initialize AI decision components"""
        # Simple state machine for behavior selection
        self.current_behavior = "idle"
        self.behavior_states = {
            "idle": self.idle_behavior,
            "walking": self.walking_behavior,
            "balancing": self.balancing_behavior,
            "reaching": self.reaching_behavior
        }

        # AI models (simplified for this example)
        self.balance_model = self.create_balance_model()
        self.walk_model = self.create_walk_model()

    def setup_communication(self):
        """Setup ROS 2 communication"""
        # Subscriptions
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10
        )

        # Publications
        self.behavior_pub = self.create_publisher(
            String, '/ai_behavior', 10
        )
        self.command_pub = self.create_publisher(
            Float64MultiArray, '/ai_joint_commands', 10
        )

        # Processing timer
        self.ai_timer = self.create_timer(0.05, self.ai_processing_loop)  # 20Hz

    def joint_callback(self, msg):
        """Process joint state data"""
        joint_data = {
            'positions': list(msg.position),
            'velocities': list(msg.velocity),
            'efforts': list(msg.effort),
            'names': list(msg.name),
            'timestamp': self.get_clock().now().nanoseconds
        }
        self.joint_buffer.append(joint_data)

    def imu_callback(self, msg):
        """Process IMU data"""
        imu_data = {
            'linear_acceleration': [
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z
            ],
            'angular_velocity': [
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z
            ],
            'orientation': [
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w
            ],
            'timestamp': self.get_clock().now().nanoseconds
        }
        self.imu_buffer.append(imu_data)

    def ai_processing_loop(self):
        """Main AI processing loop"""
        # Get latest sensor data
        latest_joint = self.joint_buffer[-1] if self.joint_buffer else None
        latest_imu = self.imu_buffer[-1] if self.imu_buffer else None

        if not latest_joint or not latest_imu:
            return  # Wait for data

        # Determine current state and select behavior
        current_state = self.extract_state_features(latest_joint, latest_imu)
        new_behavior = self.select_behavior(current_state, latest_joint, latest_imu)

        # Update behavior if changed
        if new_behavior != self.current_behavior:
            self.get_logger().info(f'Switching behavior: {self.current_behavior} -> {new_behavior}')
            self.current_behavior = new_behavior

            # Publish behavior change
            behavior_msg = String()
            behavior_msg.data = new_behavior
            self.behavior_pub.publish(behavior_msg)

        # Generate commands based on current behavior
        commands = self.generate_behavior_commands(self.current_behavior, current_state)

        # Publish commands
        if commands:
            cmd_msg = Float64MultiArray()
            cmd_msg.data = commands
            self.command_pub.publish(cmd_msg)

    def extract_state_features(self, joint_data, imu_data):
        """Extract features from sensor data for AI processing"""
        features = {}

        # Joint-based features
        if joint_data:
            features['joint_positions'] = joint_data['positions']
            features['joint_velocities'] = joint_data['velocities']
            features['joint_efforts'] = joint_data['efforts']

            # Calculate derived features
            features['center_of_mass'] = self.calculate_com(joint_data)
            features['joint_ranges'] = self.calculate_joint_ranges(joint_data)

        # IMU-based features
        if imu_data:
            features['linear_acceleration'] = imu_data['linear_acceleration']
            features['angular_velocity'] = imu_data['angular_velocity']
            features['orientation'] = imu_data['orientation']

            # Calculate derived features
            features['tilt_angle'] = self.calculate_tilt(imu_data)
            features['balance_state'] = self.assess_balance(imu_data)

        return features

    def calculate_com(self, joint_data):
        """Calculate center of mass approximation"""
        # Simplified CoM calculation
        # In reality, this would use mass properties from URDF
        if not joint_data['positions']:
            return [0.0, 0.0, 0.0]

        # For simplicity, return torso position as CoM approximation
        return [0.0, 0.0, 0.8]  # Approximate height

    def calculate_joint_ranges(self, joint_data):
        """Calculate how close joints are to their limits"""
        # For this example, assume joint limits of ±2.5 radians
        limits = 2.5
        ranges = []
        for pos in joint_data['positions']:
            # Calculate normalized distance to limit (0 = at limit, 1 = at center)
            range_val = 1.0 - min(abs(pos) / limits, 1.0)
            ranges.append(range_val)
        return ranges

    def calculate_tilt(self, imu_data):
        """Calculate robot tilt from IMU data"""
        acc = imu_data['linear_acceleration']
        tilt_x = np.arctan2(acc[1], acc[2])  # Tilt around X axis
        tilt_y = np.arctan2(-acc[0], acc[2])  # Tilt around Y axis
        return [tilt_x, tilt_y]

    def assess_balance(self, imu_data):
        """Assess robot balance state"""
        tilt = self.calculate_tilt(imu_data)
        tilt_magnitude = np.sqrt(tilt[0]**2 + tilt[1]**2)

        if tilt_magnitude > 0.5:  # High tilt
            return "unstable"
        elif tilt_magnitude > 0.2:  # Moderate tilt
            return "caution"
        else:  # Low tilt
            return "stable"

    def select_behavior(self, state, joint_data, imu_data):
        """Select appropriate behavior based on current state"""
        balance_state = state.get('balance_state', 'stable')

        # Emergency behavior if robot is falling
        if balance_state == "unstable":
            return "balancing"

        # Check if we're in a safe position to change behaviors
        joint_ranges = state.get('joint_ranges', [1.0]*12)
        if min(joint_ranges) < 0.1:  # Too close to joint limits
            return "balancing"  # Need to correct position first

        # Select behavior based on other factors
        # This could be based on high-level commands, goals, etc.
        # For this example, we'll use a simple state machine
        if self.current_behavior == "idle":
            # If stable and idle, maybe start walking
            if balance_state == "stable":
                return "walking"
        elif self.current_behavior == "walking":
            # If unstable while walking, go to balancing
            if balance_state != "stable":
                return "balancing"
            # Otherwise, continue walking
        elif self.current_behavior == "balancing":
            # If balanced, return to normal behavior
            if balance_state == "stable":
                return "walking"

        return self.current_behavior  # Maintain current behavior

    def generate_behavior_commands(self, behavior, state):
        """Generate commands for the specified behavior"""
        if behavior in self.behavior_states:
            return self.behavior_states[behavior](state)
        else:
            # Default to idle behavior
            return self.idle_behavior(state)

    def idle_behavior(self, state):
        """Idle behavior - maintain current position"""
        # Return current joint positions as commands (position control)
        # For this example, return neutral positions
        neutral_positions = [0.0] * 12  # 12 joints
        return neutral_positions

    def walking_behavior(self, state):
        """Walking behavior - generate walking gait pattern"""
        # Simple walking pattern based on time
        current_time = self.get_clock().now().nanoseconds / 1e9
        phase = current_time * 2  # 2 rad/s walking frequency

        commands = []
        for i in range(12):
            if i < 4:  # Leg joints - walking gait
                # Generate walking pattern for legs
                leg_phase = phase + (i % 2) * np.pi  # Alternate legs
                cmd = 0.4 * np.sin(leg_phase) + (0.1 if i < 2 else -0.1)  # Hip offset
            elif i < 8:  # Arm joints - counterbalance
                cmd = -0.3 * np.sin(phase + np.pi/2)  # Counterbalance arms
            else:  # Head, etc.
                cmd = 0.0
            commands.append(cmd)

        return commands

    def balancing_behavior(self, state):
        """Balancing behavior - correct for tilt and instability"""
        tilt = state.get('tilt_angle', [0.0, 0.0])

        # Generate correction commands based on tilt
        correction_commands = []
        for i in range(12):
            if i < 4:  # Leg joints for balance
                # Apply tilt correction to hip and knee joints
                tilt_corr = -0.5 * (tilt[0] if i % 2 == 0 else tilt[1])
                correction_commands.append(tilt_corr)
            elif i < 8:  # Arm joints for additional balance
                # Use arms for balance correction
                arm_corr = 0.3 * (tilt[1] if i % 2 == 0 else -tilt[0])
                correction_commands.append(arm_corr)
            else:  # Other joints
                correction_commands.append(0.0)

        return correction_commands

    def reaching_behavior(self, state):
        """Reaching behavior - move arm to target position"""
        # This would implement inverse kinematics for reaching
        # For this example, return a simple reaching pattern
        reaching_commands = []
        for i in range(12):
            if 4 <= i < 8:  # Arm joints
                # Simple reaching motion
                reaching_commands.append(0.5 if i % 2 == 0 else -0.3)
            else:
                reaching_commands.append(0.0)

        return reaching_commands

    def create_balance_model(self):
        """Create or load balance control model"""
        # In practice, this might load a trained neural network
        # For this example, return a simple function
        return lambda state: self.balance_correction_from_state(state)

    def create_walk_model(self):
        """Create or load walking gait model"""
        # For this example, return the walking behavior function
        return self.walking_behavior

    def balance_correction_from_state(self, state):
        """Generate balance correction from state"""
        tilt = state.get('tilt_angle', [0.0, 0.0])
        return [-tilt[0] * 0.5, -tilt[1] * 0.5]  # Simple PD-like correction

def main(args=None):
    rclpy.init(args=args)
    ai_agent = AdvancedAIAgent()

    try:
        rclpy.spin(ai_agent)
    except KeyboardInterrupt:
        ai_agent.get_logger().info('AI Agent interrupted by user')
    finally:
        ai_agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 3. Safety Monitor Node

Let's create a safety monitoring node that ensures safe operation:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import String, Bool
from builtin_interfaces.msg import Time
import numpy as np

class SafetyMonitor(Node):
    def __init__(self):
        super().__init__('safety_monitor')

        # Safety parameters
        self.joint_limit_threshold = 2.5  # radians
        self.imu_tilt_threshold = 0.8     # radians
        self.emergency_stop_active = False
        self.safety_violations = []

        # Setup monitoring
        self.setup_monitoring()

    def setup_monitoring(self):
        """Setup safety monitoring subscriptions"""
        # Monitor all critical sensor data
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_monitor_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_monitor_callback, 10
        )

        # Emergency stop publisher
        self.emergency_stop_pub = self.create_publisher(
            Bool, '/emergency_stop', 10
        )

        # Safety status publisher
        self.safety_status_pub = self.create_publisher(
            String, '/safety_status', 10
        )

        # Regular safety check timer
        self.safety_check_timer = self.create_timer(0.1, self.safety_periodic_check)

    def joint_monitor_callback(self, msg):
        """Monitor joint states for safety violations"""
        if self.emergency_stop_active:
            return

        violations = []

        for i, position in enumerate(msg.position):
            if abs(position) > self.joint_limit_threshold:
                violation = f"Joint {i} ({msg.name[i] if i < len(msg.name) else 'unknown'}) exceeded limit: {position:.2f}"
                violations.append(violation)
                self.get_logger().error(violation)

        if violations:
            self.safety_violations.extend(violations)
            self.trigger_emergency_stop("Joint limit exceeded")

    def imu_monitor_callback(self, msg):
        """Monitor IMU data for safety violations"""
        if self.emergency_stop_active:
            return

        # Calculate tilt from IMU data
        linear_acc = msg.linear_acceleration
        tilt_magnitude = np.sqrt(linear_acc.x**2 + linear_acc.y**2)

        if tilt_magnitude > self.imu_tilt_threshold:
            violation = f"Robot tilt exceeded threshold: {tilt_magnitude:.2f} > {self.imu_tilt_threshold:.2f}"
            self.get_logger().error(violation)
            self.safety_violations.append(violation)
            self.trigger_emergency_stop("Excessive tilt detected")

    def safety_periodic_check(self):
        """Periodic safety checks"""
        if self.emergency_stop_active:
            # Check if it's safe to resume (this is a simplified check)
            # In practice, you'd have a more sophisticated recovery procedure
            pass

        # Publish current safety status
        status_msg = String()
        if self.emergency_stop_active:
            status_msg.data = "EMERGENCY_STOP_ACTIVE"
        elif self.safety_violations:
            status_msg.data = f"WARNING: {len(self.safety_violations)} violations"
        else:
            status_msg.data = "SAFE"

        self.safety_status_pub.publish(status_msg)

    def trigger_emergency_stop(self, reason):
        """Trigger emergency stop procedure"""
        if not self.emergency_stop_active:
            self.get_logger().fatal(f"EMERGENCY STOP: {reason}")
            self.emergency_stop_active = True

            # Publish emergency stop command
            stop_msg = Bool()
            stop_msg.data = True
            self.emergency_stop_pub.publish(stop_msg)

            # Log the incident
            self.log_safety_incident(reason)

    def log_safety_incident(self, reason):
        """Log safety incidents for analysis"""
        timestamp = self.get_clock().now().to_msg()
        log_entry = f"[{timestamp.sec}.{timestamp.nanosec:09d}] SAFETY INCIDENT: {reason}"

        # In practice, you might save this to a file or database
        self.get_logger().info(f"SAFETY LOG: {log_entry}")

def main(args=None):
    rclpy.init(args=args)
    safety_monitor = SafetyMonitor()

    try:
        rclpy.spin(safety_monitor)
    except KeyboardInterrupt:
        safety_monitor.get_logger().info('Safety monitor interrupted by user')
    finally:
        safety_monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Launch File for the Complete System

Let's create a launch file to bring up the complete system:

```python
# humanoid_control_system.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare launch arguments
    urdf_arg = DeclareLaunchArgument(
        'urdf',
        default_value=os.path.join(
            get_package_share_directory('your_robot_description'),
            'urdf',
            'simple_humanoid.urdf'
        ),
        description='URDF file path'
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': open(LaunchConfiguration('urdf')).read()
        }]
    )

    # Joint state publisher (GUI for manual control)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'rate': 50,
            'use_gui': True
        }]
    )

    # Main control system
    control_system = Node(
        package='humanoid_control',
        executable='control_system',
        name='humanoid_control_system',
        parameters=[{
            'robot_name': 'simple_humanoid',
            'control_frequency': 100
        }]
    )

    # AI agent
    ai_agent = Node(
        package='humanoid_control',
        executable='ai_agent',
        name='advanced_ai_agent',
        parameters=[{
            'behavior_frequency': 20,
            'safety_thresholds': {
                'balance': 0.5,
                'joint_limits': 2.5
            }
        }]
    )

    # Safety monitor
    safety_monitor = Node(
        package='humanoid_control',
        executable='safety_monitor',
        name='safety_monitor',
        parameters=[{
            'joint_limit_threshold': 2.5,
            'tilt_threshold': 0.8
        }]
    )

    return LaunchDescription([
        urdf_arg,
        robot_state_publisher,
        joint_state_publisher,
        control_system,
        ai_agent,
        safety_monitor
    ])
```

## Testing and Debugging

### 1. Basic System Test

```bash
# Launch the complete system
ros2 launch humanoid_control_system.launch.py

# Monitor topics
ros2 topic echo /joint_states
ros2 topic echo /ai_decisions
ros2 topic echo /safety_status

# Send commands to test
ros2 topic pub /ai_behavior std_msgs/String "data: 'walking'"
```

### 2. Debugging Tools

```python
# Debugging node to monitor system state
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState

class SystemDebugger(Node):
    def __init__(self):
        super().__init__('system_debugger')

        self.subscription = self.create_subscription(
            String, '/debug_info', self.listener_callback, 10
        )

        self.joint_subscription = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10
        )

        self.debug_publisher = self.create_publisher(
            String, '/debug_output', 10
        )

        self.get_logger().info('System Debugger initialized')

    def listener_callback(self, msg):
        self.get_logger().info(f'Debug: {msg.data}')

    def joint_callback(self, msg):
        # Monitor joint states for debugging
        position_str = ', '.join([f'{p:.2f}' for p in msg.position[:5]])  # First 5 joints
        self.get_logger().debug(f'Joint positions: [{position_str}...]')

def main(args=None):
    rclpy.init(args=args)
    debugger = SystemDebugger()
    rclpy.spin(debugger)
    debugger.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Performance Optimization Tips

### 1. Threading for Heavy Computations

```python
import threading
import queue

class OptimizedAIAgent(Node):
    def __init__(self):
        super().__init__('optimized_ai_agent')

        # Thread-safe queues for data processing
        self.sensor_queue = queue.Queue(maxsize=10)
        self.command_queue = queue.Queue(maxsize=10)

        # Processing thread for heavy AI computations
        self.ai_thread = threading.Thread(target=self.ai_processing_thread)
        self.ai_thread.daemon = True
        self.ai_thread.start()

    def sensor_callback(self, msg):
        """Non-blocking sensor data handling"""
        try:
            self.sensor_queue.put_nowait(msg)
        except queue.Full:
            self.get_logger().warn('Sensor queue full, dropping data')

    def ai_processing_thread(self):
        """Dedicated thread for AI processing"""
        while rclpy.ok():
            try:
                # Get sensor data
                sensor_data = self.sensor_queue.get(timeout=0.1)

                # Heavy AI processing here
                commands = self.process_with_ai_model(sensor_data)

                # Put commands in queue for main thread
                try:
                    self.command_queue.put_nowait(commands)
                except queue.Full:
                    self.get_logger().warn('Command queue full')

            except queue.Empty:
                continue  # Check again
```

### 2. Efficient Data Structures

```python
from collections import deque
import numpy as np

class EfficientDataHandler:
    def __init__(self):
        # Use deques for time-series data
        self.joint_history = deque(maxlen=100)
        self.imu_history = deque(maxlen=100)

        # Use numpy arrays for numerical computations
        self.joint_positions = np.zeros(12, dtype=np.float32)
        self.joint_velocities = np.zeros(12, dtype=np.float32)
```

## Summary

In this integration project, we've built a complete ROS 2 humanoid control system that combines all the concepts from previous chapters:

1. **Communication**: We used nodes, topics, services, and proper QoS settings
2. **AI Integration**: We created sophisticated AI agents with behavior selection
3. **Humanoid Modeling**: We referenced URDF models and coordinated with robot state publishing
4. **Real-time Control**: We implemented 100Hz control loops with safety systems

The system includes:
- Main control system coordinating all components
- Advanced AI agent with behavior selection
- Safety monitoring with emergency stop capabilities
- Proper launch files for system integration
- Debugging and performance optimization techniques

This complete system demonstrates how all the individual components learned in this module work together to create a functional humanoid robot control system.