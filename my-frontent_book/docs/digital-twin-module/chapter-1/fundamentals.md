# Chapter 1: Gazebo Simulation Fundamentals

## Introduction

Gazebo is a powerful 3D simulation environment that provides accurate physics simulation, high-quality graphics, and convenient programmatic interfaces. In this chapter, we'll set up Gazebo with ROS 2 integration and learn how to create realistic simulation environments for our humanoid robot.

## What is Gazebo?

Gazebo is a 3D dynamic simulator with the ability to accurately and efficiently simulate populations of robots in complex indoor and outdoor environments. It provides:

- **Physics Simulation**: Realistic physics using ODE, Bullet, or DART engines
- **Sensor Simulation**: Cameras, LiDAR, IMU, and other sensors
- **3D Visualization**: High-quality rendering for real-time visualization
- **ROS Integration**: Seamless integration with ROS and ROS 2

## Installing Gazebo with ROS 2

For ROS 2 Humble Hawksbill, install Gazebo Fortress with ROS 2 packages:

```bash
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers
```

## Basic Gazebo Concepts

### Worlds
A world file defines the environment where robots operate, including:
- Physical properties (gravity, physics engine parameters)
- Models and their initial poses
- Lighting and environmental settings

### Models
Models represent objects in the simulation, including:
- Robots with joints and links
- Static objects and obstacles
- Sensors and actuators

### Plugins
Plugins extend Gazebo functionality:
- Control plugins for robot interfaces
- Sensor plugins for data generation
- Physics plugins for custom behaviors

## Creating Your First Gazebo World

Let's create a simple world file for our humanoid robot:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="humanoid_world">
    <!-- Physics Engine -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
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
    <!-- We'll add our robot here in the next section -->
  </world>
</sdf>
```

## Gazebo with ROS 2 Integration

### Spawning Models
Use the `spawn_entity.py` script to spawn models in Gazebo:

```bash
ros2 run gazebo_ros spawn_entity.py -entity my_robot -file path/to/robot.urdf -x 0 -y 0 -z 1
```

### ROS 2 Topics and Services
Gazebo publishes and subscribes to various ROS 2 topics:
- `/clock` - Simulation time
- `/gazebo/model_states` - States of all models
- `/gazebo/link_states` - States of all links
- Services for model manipulation

## Gazebo Plugins for ROS 2

### Robot State Publisher Plugin
This plugin publishes TF transforms for the robot:

```xml
<gazebo>
  <plugin name="robot_state_publisher" filename="libgazebo_ros_robot_state_publisher.so">
    <tf_topic>tf</tf_topic>
    <topic_name>robot_description</topic_name>
  </plugin>
</gazebo>
```

### Joint State Publisher Plugin
This plugin publishes joint states:

```xml
<gazebo>
  <plugin name="joint_state_publisher" filename="libgazebo_ros_joint_state_publisher.so">
    <joint_name>joint1, joint2, joint3</joint_name>
    <update_rate>30</update_rate>
    <topic_name>joint_states</topic_name>
  </plugin>
</gazebo>
```

## Exercise: Launch Basic Gazebo Environment

1. Launch Gazebo:
   ```bash
   gz sim
   ```

2. Verify that Gazebo starts correctly with the default empty world

3. Try spawning a simple model to test the environment

## Summary

In this chapter, we've covered the fundamentals of Gazebo simulation and its integration with ROS 2. We've learned about:
- Installing Gazebo with ROS 2 packages
- Basic concepts like worlds, models, and plugins
- How to create basic world files
- Essential ROS 2 integration patterns

In the next chapter, we'll explore Unity integration with ROS 2, which will complement our Gazebo physics simulation with real-time visualization capabilities.