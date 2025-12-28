# Digital Twin Simulation Stack - Specification

## 1. Overview
This specification defines the requirements for Module 2: The Digital Twin (Gazebo & Unity) that builds upon the ROS 2 infrastructure established in Module 1. The digital twin will provide a comprehensive simulation environment for the humanoid robot with both physics-accurate simulation in Gazebo and real-time visualization in Unity.

## 2. Requirements

### 2.1 Functional Requirements
- **R1.0**: The system shall integrate Gazebo physics simulation with ROS 2 communication infrastructure
- **R1.1**: The system shall integrate Unity 3D visualization with ROS 2 communication infrastructure
- **R1.2**: The system shall import the humanoid URDF model from Module 1 into both simulation environments
- **R1.3**: The system shall maintain synchronized state between Gazebo physics and Unity visualization
- **R1.4**: The system shall support real-time joint control of the humanoid robot through ROS 2
- **R1.5**: The system shall simulate sensors (IMU, cameras, force/torque) and publish realistic data
- **R1.6**: The system shall validate stable humanoid motion in the physics simulation environment
- **R1.7**: The system shall provide a unified interface for controlling both simulation environments

### 2.2 Non-Functional Requirements
- **R2.0**: The Gazebo physics simulation shall run at 1000 Hz for stable humanoid control
- **R2.1**: The Unity visualization shall maintain 30+ FPS for smooth user experience
- **R2.2**: The latency between Gazebo physics and Unity visualization shall be less than 50ms
- **R2.3**: The system shall handle up to 20 degrees of freedom for the humanoid robot
- **R2.4**: The system shall consume less than 80% of CPU resources during normal operation
- **R2.5**: The system shall maintain stable operation for 8+ hours of continuous simulation

## 3. Architecture

### 3.1 System Architecture
```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   ROS 2 Nodes   │◄──►│  ROS TCP Bridge  │◄──►│   Unity 3D      │
│                 │    │                  │    │                 │
│  Joint Control  │    │  gazebo_ros_pkgs │    │  Visualization  │
│  State Publish  │    │  ros2_control    │    │  User Interface │
└─────────────────┘    └──────────────────┘    └─────────────────┘
        ▲                       ▲
        │                       │
        └───────────────────────┘
                Gazebo
           Physics Simulation
```

### 3.2 Component Descriptions
- **ROS 2 Core**: Provides communication infrastructure and control interfaces
- **Gazebo Physics Engine**: Handles realistic physics simulation and collision detection
- **Unity Visualization**: Provides real-time 3D visualization and user interaction
- **Synchronization Layer**: Maintains state consistency between simulators
- **Sensor Simulation**: Emulates real sensor data for development and testing

## 4. Interface Specifications

### 4.1 ROS 2 Topics
- **/joint_states** (sensor_msgs/JointState): Current joint positions, velocities, efforts
- **/joint_trajectory** (trajectory_msgs/JointTrajectory): Desired joint trajectories
- **/imu/data** (sensor_msgs/Imu): Inertial measurement unit data
- **/camera/image_raw** (sensor_msgs/Image): Camera sensor data
- **/tf** and **/tf_static** (tf2_msgs/TFMessage): Transform relationships

### 4.2 ROS 2 Services
- **/spawn_entity** (gazebo_msgs/SpawnEntity): Spawn robot model in Gazebo
- **/set_joint_trajectory** (control_msgs/SetJointTrajectory): Set joint trajectory
- **/reset_simulation** (std_srvs/Empty): Reset simulation to initial state

### 4.3 Unity Components
- **ROS TCP Connector**: Establishes connection to ROS network
- **Joint Controller**: Synchronizes joint positions from ROS
- **Sensor Visualizer**: Displays sensor data in Unity scene
- **User Interface**: Provides controls for teleoperation

## 5. Data Models

### 5.1 Humanoid Robot Model
- **Base**: Torso with head attachment
- **Arms**: 2 arms with shoulder, elbow joints (4 DOF total)
- **Legs**: 2 legs with hip, knee joints (4 DOF total)
- **Total**: Minimum 8 DOF with expandability for wrists, ankles, etc.

### 5.2 Physical Properties
- **Mass Distribution**: Realistic mass properties for each link
- **Inertial Tensors**: Properly defined for stable simulation
- **Joint Limits**: Realistic range of motion constraints
- **Friction Coefficients**: Appropriate values for contact simulation

## 6. Validation Criteria

### 6.1 Physics Validation
- **V1.0**: Humanoid maintains stable standing position under gravity
- **V1.1**: Joint movements stay within defined limits
- **V1.2**: Collision detection prevents interpenetration
- **V1.3**: Sensor data reflects realistic physical interactions

### 6.2 Integration Validation
- **V2.0**: Unity visualization accurately reflects Gazebo physics state
- **V2.1**: ROS 2 communication maintains real-time performance
- **V2.2**: Control commands result in expected robot behavior
- **V2.3**: System handles error conditions gracefully

## 7. Acceptance Tests

### 7.1 Basic Functionality Tests
- **T1.0**: Verify URDF model loads in both Gazebo and Unity
- **T1.1**: Verify joint control commands move robot in both simulators
- **T1.2**: Verify sensor data publication from both simulators
- **T1.3**: Verify state synchronization between simulators

### 7.2 Performance Tests
- **T2.0**: Measure Gazebo simulation update rate (target: 1000 Hz)
- **T2.1**: Measure Unity frame rate (target: 30+ FPS)
- **T2.2**: Measure synchronization latency (target: <50ms)
- **T2.3**: Measure CPU/memory usage under load

### 7.3 Stress Tests
- **T3.0**: Run simulation continuously for 8 hours
- **T3.1**: Test with maximum joint velocities
- **T3.2**: Test with complex interaction scenarios
- **T3.3**: Test network interruption and recovery

## 8. Deliverables
- Complete Gazebo simulation environment with humanoid robot
- Unity visualization environment with synchronized robot model
- ROS 2 integration with proper control and sensor interfaces
- Documentation for setup, operation, and troubleshooting
- Validation test results and performance benchmarks