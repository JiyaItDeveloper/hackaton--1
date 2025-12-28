# Gazebo Physics Layer Configuration for Humanoid Robot

## Physics Engine Configuration

### Main Physics Parameters
```xml
<!-- In the Gazebo world file or physics configuration -->
<physics type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000.0</real_time_update_rate>
  <gravity>0 0 -9.8</gravity>
  <ode>
    <solver>
      <type>quick</type>
      <iters>10</iters>
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

### Physics Engine Options
- **ODE (Open Dynamics Engine)**: Default choice, good for humanoid robots
- **Bullet**: Alternative with better multi-body dynamics
- **DART**: Advanced dynamics engine with better stability

## URDF Extensions for Gazebo Simulation

### Gazebo Plugin Configuration
```xml
<!-- In the humanoid URDF file -->
<gazebo>
  <!-- Robot state publisher plugin -->
  <plugin name="robot_state_publisher" filename="libgazebo_ros_robot_state_publisher.so">
    <tf_topic>tf</tf_topic>
    <topic_name>robot_description</topic_name>
  </plugin>
</gazebo>

<!-- Joint state publisher plugin -->
<gazebo>
  <plugin name="joint_state_publisher" filename="libgazebo_ros_joint_state_publisher.so">
    <joint_name>joint1, joint2, joint3</joint_name>
    <update_rate>30</update_rate>
    <topic_name>joint_states</topic_name>
  </plugin>
</gazebo>
```

### Inertial Properties Configuration
For each link in the URDF, ensure proper inertial properties:

```xml
<link name="link_name">
  <inertial>
    <mass value="1.0" />
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01" />
  </inertial>
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <mesh filename="package://humanoid_description/meshes/link_name.dae"/>
    </geometry>
  </visual>
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <mesh filename="package://humanoid_description/meshes/link_name_collision.stl"/>
    </geometry>
  </collision>
</link>
```

## Joint Configuration for Humanoid Robot

### Joint Limits and Dynamics
```xml
<joint name="joint_name" type="revolute">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="3.14"/>
  <dynamics damping="0.1" friction="0.0"/>
</joint>
```

### Gazebo Joint Plugins
```xml
<gazebo reference="joint_name">
  <implicitSpringDamper>1</implicitSpringDamper>
  <provideFeedback>1</provideFeedback>
</gazebo>
```

## ros2_control Integration

### Controller Configuration File (YAML)
```yaml
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

    state_publish_rate: 50.0
    action_monitor_rate: 20.0

    allow_partial_joints_goal: false
    open_loop_control: true
    allow_integration_in_goal_trajectories: true
    constraints:
      stopped_velocity_tolerance: 0.01
      goal_time: 0.0
```

### URDF ros2_control Integration
```xml
<!-- Add to the end of the URDF file -->
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

## Collision Detection Configuration

### Collision Properties
```xml
<gazebo reference="link_name">
  <mu1>0.3</mu1>
  <mu2>0.3</mu2>
  <kp>1000000.0</kp>
  <kd>100.0</kd>
  <max_vel>100.0</max_vel>
  <min_depth>0.001</min_depth>
</gazebo>
```

### Contact Sensors
```xml
<gazebo reference="foot_link">
  <sensor name="foot_contact_sensor" type="contact">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <contact>
      <collision>foot_link_collision</collision>
    </contact>
    <plugin name="contact_plugin" filename="libgazebo_ros_contact.so">
      <topic_name>foot_contact</topic_name>
    </plugin>
  </sensor>
</gazebo>
```

## World File Configuration

### Basic World with Physics Parameters
```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="humanoid_world">
    <!-- Physics Engine -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000.0</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
    </physics>

    <!-- Ground Plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Lighting -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Humanoid Robot Model -->
    <include>
      <uri>model://humanoid_robot</uri>
      <pose>0 0 1 0 0 0</pose>
    </include>

    <!-- Optional: Additional environment models -->
  </world>
</sdf>
```

## Gravity and Environmental Configuration

### Gravity Settings
- **Standard Gravity**: 9.8 m/s² (can be modified for different environments)
- **Direction**: Negative Z-axis (downward)
- **Adjustable**: Can be modified for different gravitational environments

### Environmental Parameters
- **Air Resistance**: Configured via damping parameters in links
- **Friction Coefficients**: Set per link based on material properties
- **Contact Properties**: Defined for realistic interaction between surfaces

## Performance Optimization

### Physics Parameters for Performance
- **Max Step Size**: 0.001s for stable humanoid simulation
- **Real-time Update Rate**: 1000Hz for real-time control
- **Solver Iterations**: Balanced between accuracy and performance
- **Collision Margin**: Adjusted for performance vs precision trade-off

### Optimization Tips
1. Use simplified collision meshes where high precision isn't needed
2. Adjust solver parameters based on simulation requirements
3. Use compound collision shapes for complex links
4. Set appropriate damping and friction values to prevent oscillation
5. Monitor simulation performance and adjust parameters accordingly

## Validation Tests

### Physics Validation Checklist
- [ ] Robot maintains stable pose when static
- [ ] Joint limits are properly enforced
- [ ] Collision detection works correctly
- [ ] Gravity affects robot appropriately
- [ ] Joint controllers respond to commands
- [ ] No unexpected oscillations or instabilities
- [ ] Real-time factor maintains 1.0 under load
- [ ] Sensor data is published at expected rates