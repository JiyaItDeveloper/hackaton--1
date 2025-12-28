# Chapter 3: Physics Simulation & Humanoid Motion

## Introduction

In this chapter, we'll implement realistic physics simulation for our humanoid robot, focusing on stable motion and proper joint control. We'll configure the physics properties in both Gazebo and Unity to ensure accurate simulation of humanoid dynamics.

## Understanding Humanoid Physics

Humanoid robots present unique challenges in physics simulation due to their:
- **Complex kinematic structure** with multiple degrees of freedom
- **Balance requirements** for stable standing and walking
- **Dynamic interactions** with the environment
- **Real-time control requirements** for responsive behavior

## Physics Configuration in Gazebo

### Inertial Properties
Proper inertial properties are crucial for realistic physics simulation:

```xml
<link name="torso">
  <inertial>
    <mass value="5.0" />
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.2" />
  </inertial>
  <!-- visual and collision elements -->
</link>
```

### Joint Dynamics
Configure joint limits and dynamics for realistic movement:

```xml
<joint name="hip_joint" type="revolute">
  <parent link="torso"/>
  <child link="thigh"/>
  <origin xyz="0 0 -0.1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="3.14"/>
  <dynamics damping="0.5" friction="0.1"/>
</joint>
```

### Contact Properties
Set appropriate friction and contact parameters:

```xml
<gazebo reference="foot_link">
  <mu1>0.8</mu1>
  <mu2>0.8</mu2>
  <kp>1000000.0</kp>
  <kd>100.0</kd>
  <max_vel>100.0</max_vel>
  <min_depth>0.001</min_depth>
</gazebo>
```

## ros2_control Integration

### Controller Configuration
Set up ros2_control for precise joint control:

```yaml
# config/humanoid_controllers.yaml
controller_manager:
  ros__parameters:
    update_rate: 1000  # Hz

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    humanoid_joint_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController

humanoid_joint_trajectory_controller:
  ros__parameters:
    joints:
      - left_hip_joint
      - left_knee_joint
      - left_ankle_joint
      - right_hip_joint
      - right_knee_joint
      - right_ankle_joint
      - left_shoulder_joint
      - left_elbow_joint
      - right_shoulder_joint
      - right_elbow_joint
      - torso_joint
      - head_joint

    command_interfaces:
      - position

    state_interfaces:
      - position
      - velocity
```

### URDF ros2_control Integration
Add ros2_control interface to your URDF:

```xml
<ros2_control name="GazeboSystem" type="system">
  <hardware>
    <plugin>gazebo_ros2_control/GazeboSystem</plugin>
  </hardware>
  <joint name="left_hip_joint">
    <command_interface name="position">
      <param name="min">-1.57</param>
      <param name="max">1.57</param>
    </command_interface>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
  <!-- Add other joints similarly -->
</ros2_control>
```

## Physics Engine Configuration

### Gazebo Physics Parameters
Optimize physics parameters for humanoid simulation:

```xml
<physics type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000.0</real_time_update_rate>
  <gravity>0 0 -9.8</gravity>
  <ode>
    <solver>
      <type>quick</type>
      <iters>100</iters>
      <sor>1.3</sor>
    </solver>
    <constraints>
      <cfm>0.0</cfm>
      <erp>0.2</erp>
      <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
      <contact_surface_layer>0.001</contact_surface_layer>
    </constraints>
  </ode>
</physics>
```

### Stability Considerations
For stable humanoid simulation:

- Use smaller step sizes (0.001s) for better accuracy
- Increase solver iterations for more stable constraints
- Adjust contact parameters to prevent interpenetration
- Balance real-time factor for performance vs accuracy

## Implementing Stable Humanoid Motion

### Balance Control
Implement basic balance control for standing:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray

class BalanceController(Node):
    def __init__(self):
        super().__init__('balance_controller')

        # Publisher for joint trajectory commands
        self.joint_pub = self.create_publisher(
            JointTrajectory,
            '/humanoid_joint_trajectory_controller/joint_trajectory',
            10
        )

        # Timer for control loop
        self.timer = self.create_timer(0.01, self.balance_callback)  # 100Hz

    def balance_callback(self):
        msg = JointTrajectory()
        msg.joint_names = [
            'left_hip_joint', 'left_knee_joint', 'left_ankle_joint',
            'right_hip_joint', 'right_knee_joint', 'right_ankle_joint'
        ]

        point = JointTrajectoryPoint()
        # Set desired joint positions for stable standing
        point.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Example positions
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 10000000  # 10ms

        msg.points.append(point)
        self.joint_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    controller = BalanceController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Walking Pattern Generation
Create basic walking patterns:

```python
import math

class WalkingPatternGenerator:
    def __init__(self):
        self.step_height = 0.1
        self.step_length = 0.2
        self.cycle_time = 2.0  # seconds per step

    def generate_step_trajectory(self, time, leg='left'):
        """Generate trajectory for a single leg during walking"""
        phase = (time % self.cycle_time) / self.cycle_time

        # Calculate position based on phase
        x = self.step_length * phase if phase < 0.5 else self.step_length * (1 - phase)
        z = self.step_height * math.sin(math.pi * phase) if phase < 0.5 else 0

        return x, 0, z  # x, y, z position
```

## Sensor Integration for Physics

### IMU Simulation
Add IMU sensors for balance feedback:

```xml
<gazebo reference="torso">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </z>
      </angular_velocity>
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </z>
      </linear_acceleration>
    </imu>
  </sensor>
</gazebo>
```

### Force/Torque Sensors
Add force/torque sensors to feet for ground contact detection:

```xml
<gazebo reference="left_foot">
  <sensor name="left_foot_ft_sensor" type="force_torque">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <force_torque>
      <frame>child</frame>
      <measure_direction>child_to_parent</measure_direction>
    </force_torque>
  </sensor>
</gazebo>
```

## Validation and Testing

### Stability Tests
Create tests to validate stable motion:

```python
import rclpy
from sensor_msgs.msg import Imu, JointState
import numpy as np

class StabilityValidator:
    def __init__(self):
        self.imu_data = None
        self.joint_data = None
        self.stability_threshold = 0.5  # meters

    def validate_stability(self):
        """Validate that the humanoid maintains stable position"""
        if self.imu_data and self.joint_data:
            # Check if orientation is within acceptable bounds
            roll, pitch = self.get_orientation_angles(self.imu_data)

            if abs(roll) > 0.5 or abs(pitch) > 0.5:
                return False  # Robot is tilting too much

            # Check if position is stable
            # Implementation would check position over time

            return True
        return False
```

### Performance Metrics
Monitor simulation performance:

- **Real-time Factor**: Should maintain close to 1.0
- **Update Rate**: Should match configured rate (1000Hz)
- **Joint Position Errors**: Should be minimal with proper control
- **Balance Metrics**: Orientation and position stability

## Exercise: Implement Basic Humanoid Controller

1. **Set up ros2_control configuration** for your humanoid model
2. **Create a basic balance controller** that maintains stable standing
3. **Test the controller** in Gazebo simulation
4. **Monitor sensor data** to validate stable motion

## Troubleshooting Physics Issues

### Robot Falling Through Ground
- Check inertial properties for proper mass distribution
- Verify collision geometries are properly defined
- Adjust contact parameters (kp, kd values)

### Joint Oscillations
- Increase damping values in joint dynamics
- Check controller gains if using PID control
- Verify joint limits are properly set

### Unstable Balance
- Review center of mass calculation
- Check that control update rate is appropriate
- Validate sensor data accuracy

## Summary

In this chapter, we've covered the implementation of physics simulation and humanoid motion:
- Configured proper inertial properties and joint dynamics
- Set up ros2_control for precise joint control
- Implemented basic balance control algorithms
- Integrated sensors for feedback and validation
- Validated stable motion through testing

In the next chapter, we'll explore how to synchronize both simulation environments to create a complete digital twin system.