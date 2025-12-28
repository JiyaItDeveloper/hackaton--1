---
sidebar_position: 5
---

# Chapter 4: Humanoid Modeling with URDF

## Learning Goals
- Create robot models using URDF
- Understand links, joints, and coordinate frames
- Prepare robot models for simulation

## Introduction to URDF

URDF (Unified Robot Description Format) is an XML format used to describe robot models in ROS. It defines the physical and visual properties of a robot, including its links (rigid parts), joints (connections between links), and coordinate frames.

### Basic URDF Structure

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Links define rigid parts of the robot -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.5"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.5 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Joints connect links -->
  <joint name="base_to_head" type="fixed">
    <parent link="base_link"/>
    <child link="head"/>
    <origin xyz="0 0 0.5"/>
  </joint>

  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </visual>
  </link>
</robot>
```

## URDF XML Structure and Syntax

### Links

Links represent rigid bodies in the robot. Each link can have multiple properties:

1. **Visual**: How the link appears in visualization tools
2. **Collision**: How the link interacts in collision detection
3. **Inertial**: Physical properties for dynamics simulation

```xml
<link name="example_link">
  <!-- Visual properties for rendering -->
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <!-- Can be box, cylinder, sphere, or mesh -->
      <box size="0.1 0.1 0.1"/>
    </geometry>
    <material name="blue">
      <color rgba="0 0 1 1"/>
    </material>
  </visual>

  <!-- Collision properties for physics simulation -->
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="0.1 0.1 0.1"/>
    </geometry>
  </collision>

  <!-- Inertial properties for dynamics -->
  <inertial>
    <mass value="0.1"/>
    <origin xyz="0 0 0"/>
    <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
  </inertial>
</link>
```

### Joints

Joints define the connection between links and specify how they can move relative to each other:

```xml
<!-- Revolute joint (rotational) -->
<joint name="elbow_joint" type="revolute">
  <parent link="upper_arm"/>
  <child link="forearm"/>
  <origin xyz="0 0 -0.3" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
</joint>

<!-- Prismatic joint (linear) -->
<joint name="slide_joint" type="prismatic">
  <parent link="base"/>
  <child link="slider"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="0" upper="0.5" effort="100" velocity="1"/>
</joint>

<!-- Fixed joint (no movement) -->
<joint name="fixed_joint" type="fixed">
  <parent link="link1"/>
  <child link="link2"/>
  <origin xyz="0.1 0 0" rpy="0 0 0"/>
</joint>
```

## Complete Humanoid URDF Model

Here's a more complete example of a simple humanoid robot:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">

  <!-- Base link (torso) -->
  <link name="torso">
    <visual>
      <origin xyz="0 0 0.4" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.2 0.8"/>
      </geometry>
      <material name="light_gray">
        <color rgba="0.7 0.7 0.7 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.4" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.2 0.8"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <origin xyz="0 0 0.4"/>
      <inertia ixx="0.5" ixy="0" ixz="0" iyy="0.6" iyz="0" izz="0.2"/>
    </inertial>
  </link>

  <!-- Head -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.8" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.78" upper="0.78" effort="10" velocity="1"/>
  </joint>

  <link name="head">
    <visual>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
      <material name="skin">
        <color rgba="0.9 0.8 0.7 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 0.1"/>
      <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.02"/>
    </inertial>
  </link>

  <!-- Left Arm -->
  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.2 0 0.6" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="1"/>
  </joint>

  <link name="left_upper_arm">
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 -0.15"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.005"/>
    </inertial>
  </link>

  <joint name="left_elbow_joint" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_forearm"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="30" velocity="1"/>
  </joint>

  <link name="left_forearm">
    <visual>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.2" radius="0.04"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.2" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 -0.1"/>
      <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.002"/>
    </inertial>
  </link>

  <!-- Right Arm (similar to left, mirrored) -->
  <joint name="right_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="right_upper_arm"/>
    <origin xyz="-0.2 0 0.6" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="1"/>
  </joint>

  <link name="right_upper_arm">
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 -0.15"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.005"/>
    </inertial>
  </link>

  <joint name="right_elbow_joint" type="revolute">
    <parent link="right_upper_arm"/>
    <child link="right_forearm"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="30" velocity="1"/>
  </joint>

  <link name="right_forearm">
    <visual>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.2" radius="0.04"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.2" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 -0.1"/>
      <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.002"/>
    </inertial>
  </link>

  <!-- Left Leg -->
  <joint name="left_hip_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_thigh"/>
    <origin xyz="0.1 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <link name="left_thigh">
    <visual>
      <origin xyz="0 0 -0.25" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.5" radius="0.06"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 -0.25" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.5" radius="0.06"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 -0.25"/>
      <inertia ixx="0.05" ixy="0" ixz="0" iyy="0.05" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="left_knee_joint" type="revolute">
    <parent link="left_thigh"/>
    <child link="left_shin"/>
    <origin xyz="0 0 -0.5" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="0" effort="80" velocity="1"/>
  </joint>

  <link name="left_shin">
    <visual>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <origin xyz="0 0 -0.2"/>
      <inertia ixx="0.03" ixy="0" ixz="0" iyy="0.03" iyz="0" izz="0.008"/>
    </inertial>
  </link>

  <!-- Right Leg (similar to left, mirrored) -->
  <joint name="right_hip_joint" type="revolute">
    <parent link="torso"/>
    <child link="right_thigh"/>
    <origin xyz="-0.1 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <link name="right_thigh">
    <visual>
      <origin xyz="0 0 -0.25" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.5" radius="0.06"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 -0.25" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.5" radius="0.06"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 -0.25"/>
      <inertia ixx="0.05" ixy="0" ixz="0" iyy="0.05" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="right_knee_joint" type="revolute">
    <parent link="right_thigh"/>
    <child link="right_shin"/>
    <origin xyz="0 0 -0.5" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="0" effort="80" velocity="1"/>
  </joint>

  <link name="right_shin">
    <visual>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <origin xyz="0 0 -0.2"/>
      <inertia ixx="0.03" ixy="0" ixz="0" iyy="0.03" iyz="0" izz="0.008"/>
    </inertial>
  </link>
</robot>
```

## Coordinate Frames and Transformations

ROS uses the tf (transform) system to keep track of coordinate frames in a robot system. For URDF models, each link has its own coordinate frame, and joints define transformations between frames.

### Understanding Coordinate Frames

Each link in URDF has its own coordinate frame with:
- X: Forward (typically)
- Y: Left (typically)
- Z: Up (typically)

The transformation between frames is defined by the joint's origin element:

```xml
<joint name="example_joint" type="revolute">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <!-- Translation and rotation from parent to child frame -->
  <origin xyz="0.1 0.0 0.0" rpy="0.0 0.0 0.0"/>
  <axis xyz="0 0 1"/>
</joint>
```

## Robot State Publisher Integration

To visualize the robot in RViz and use it in simulations, you need to publish the robot state:

### ROS 2 Node for Robot State Publishing

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math

class RobotStatePublisher(Node):
    def __init__(self):
        super().__init__('robot_state_publisher')

        # Create publisher for joint states
        self.joint_state_publisher = self.create_publisher(
            JointState, 'joint_states', 10
        )

        # Create transform broadcaster for tf
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer to publish states at regular intervals
        self.timer = self.create_timer(0.05, self.publish_joint_states)  # 20Hz

        # Initialize joint positions
        self.joint_positions = {
            'neck_joint': 0.0,
            'left_shoulder_joint': 0.0,
            'left_elbow_joint': 0.0,
            'right_shoulder_joint': 0.0,
            'right_elbow_joint': 0.0,
            'left_hip_joint': 0.0,
            'left_knee_joint': 0.0,
            'right_hip_joint': 0.0,
            'right_knee_joint': 0.0
        }

    def publish_joint_states(self):
        # Create joint state message
        msg = JointState()
        msg.name = list(self.joint_positions.keys())
        msg.position = list(self.joint_positions.values())
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # Publish joint states
        self.joint_state_publisher.publish(msg)

        # Broadcast transforms
        self.broadcast_transforms()

    def broadcast_transforms(self):
        # For each joint, broadcast the transform
        for joint_name, position in self.joint_positions.items():
            # Create transform for this joint
            t = TransformStamped()

            # Time stamp
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'base_link'  # This would be the parent link
            t.child_frame_id = joint_name.replace('_joint', '_link')  # Child link

            # Set transform based on joint position
            # This is simplified - in reality you'd calculate based on URDF
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0

            # Convert joint position to quaternion (simplified)
            # In practice, you'd use forward kinematics
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = math.sin(position / 2.0)
            t.transform.rotation.w = math.cos(position / 2.0)

            # Send the transform
            self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = RobotStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Visualization in RViz

To visualize your URDF model in RViz:

1. Launch your robot state publisher
2. Add RobotModel display in RViz
3. Set the Robot Description parameter to your URDF parameter name
4. Set Fixed Frame to your base link (e.g., "torso")

### Launch File Example

```xml
# robot_visualization.launch.py
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('your_robot_description')
    urdf_file = os.path.join(pkg_share, 'urdf', 'simple_humanoid.urdf')

    return LaunchDescription([
        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[
                {'robot_description': Command(['xacro ', urdf_file])}
            ]
        ),

        # Joint state publisher (for GUI control)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher'
        )
    ])
```

## Best Practices for URDF Modeling

### 1. Proper Inertial Properties
- Calculate realistic mass and inertia values
- Use CAD software to compute inertial properties
- Ensure stability in physics simulations

### 2. Collision vs Visual Geometry
- Use simpler geometry for collision detection (improves performance)
- Use detailed geometry for visual representation
- Ensure collision geometry is at least as large as visual geometry

### 3. Joint Limits
- Set realistic joint limits based on physical constraints
- Include effort and velocity limits for safety
- Consider using safety margins in limits

### 4. Mass Distribution
- Properly distribute mass throughout the robot
- Consider center of gravity for stability
- Balance computational complexity with accuracy

## Troubleshooting Common Issues

### 1. Invalid URDF
- Use `check_urdf` command to validate: `check_urdf /path/to/robot.urdf`
- Common issues: missing parent links, invalid joint types, malformed XML

### 2. TF Tree Issues
- Use `tf2_tools` to visualize the transform tree: `ros2 run tf2_tools view_frames`
- Ensure all links are connected in a proper tree structure

### 3. Physics Simulation Issues
- Check inertial properties are properly defined
- Ensure mass values are realistic
- Verify joint limits and safety controllers

## Summary

In this chapter, you've learned how to create complete humanoid robot models using URDF. You've seen how to define links with visual, collision, and inertial properties, configure different types of joints, and set up coordinate frames and transformations. You've also learned about robot state publishing for visualization and best practices for URDF modeling. This completes the core content of the ROS 2 module. In the next section, we'll combine all these concepts in an integration project.