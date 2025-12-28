# URDF Import and Validation Process for Digital Twin

## Overview
This document outlines the process for importing the existing humanoid URDF model from Module 1 into both Gazebo and Unity simulation environments, including validation procedures to ensure proper functionality.

## Current URDF Model Analysis

### Existing Model Structure (from Module 1)
Based on the existing ROS 2 module documentation, the humanoid model includes:
- **Torso**: Main body with appropriate inertial properties
- **Head**: With neck joint for orientation
- **Arms**: Shoulders and elbows with proper joint limits
- **Legs**: Hips and knees with realistic movement constraints
- **Joint Configuration**: Properly defined joint types, limits, and dynamics

### URDF File Structure Requirements
- Valid XML syntax
- Proper parent-child link relationships
- Correct joint definitions with appropriate types
- Inertial properties for each link
- Visual and collision geometries
- Material definitions for visualization

## Gazebo URDF Import Process

### Step 1: URDF Enhancement for Gazebo
1. **Add Gazebo-specific tags** to the existing URDF:
   ```xml
   <gazebo reference="link_name">
     <material>Gazebo/Blue</material>
     <mu1>0.3</mu1>
     <mu2>0.3</mu2>
   </gazebo>
   ```

2. **Add plugin configurations**:
   - Robot state publisher plugin
   - Joint state publisher plugin
   - ros2_control hardware interface

3. **Verify inertial properties** for realistic physics simulation

### Step 2: Gazebo Model Preparation
1. **Create model.config file**:
   ```xml
   <?xml version="1.0"?>
   <model>
     <name>humanoid_robot</name>
     <version>1.0</version>
     <sdf version="1.7">model.sdf</sdf>
     <author>
       <name>Your Name</name>
       <email>your.email@example.com</email>
     </author>
     <description>Humanoid robot model for simulation</description>
   </model>
   ```

2. **Organize mesh files** in proper directory structure:
   ```
   models/humanoid_robot/
   ├── meshes/
   │   ├── torso.dae
   │   ├── head.dae
   │   ├── arm.dae
   │   └── leg.dae
   ├── model.urdf
   └── model.config
   ```

### Step 3: Gazebo Import and Testing
1. **Launch Gazebo with the model**:
   ```bash
   # Set environment variable
   export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/path/to/models

   # Launch Gazebo
   gz sim -v 4
   ```

2. **Spawn the model**:
   ```bash
   ros2 run gazebo_ros spawn_entity.py -entity humanoid_robot -file $(find ~/path/to/models -name "model.urdf") -x 0 -y 0 -z 1
   ```

## Unity URDF Import Process

### Step 1: Unity Environment Setup
1. **Install URDF Importer package** in Unity:
   - Open Unity Package Manager
   - Install "URDF Importer" from Unity Robotics repository
   - Verify installation in Project window

2. **Prepare URDF files for Unity**:
   - Ensure all mesh files are in compatible formats (FBX, OBJ, DAE)
   - Verify all file paths are accessible from Unity project
   - Check that URDF uses absolute paths or proper package:// references

### Step 2: URDF Import into Unity
1. **Import process**:
   - In Unity, go to Assets → Import Robot from URDF
   - Select the URDF file location
   - Configure import settings:
     - Base Path: Root directory containing meshes
     - Collision Handling: Use convex hulls for performance
     - Joint Configuration: Preserve joint limits and types

2. **Post-import processing**:
   - Verify joint hierarchy matches URDF structure
   - Check that visual meshes align with collision meshes
   - Validate material assignments and colors

### Step 3: ROS Communication Setup in Unity
1. **Add ROS TCP Connector** to Unity scene:
   - Create empty GameObject
   - Add "ROS TCP Connector" component
   - Configure IP address and port (default: 10000)

2. **Implement joint synchronization**:
   - Create scripts to subscribe to joint_states topic
   - Map ROS joint names to Unity joint hierarchy
   - Implement interpolation for smooth motion

## Validation Procedures

### Gazebo Validation Checklist
- [ ] **Model Spawning**: Robot model successfully spawns in Gazebo environment
- [ ] **Visual Representation**: All links display with correct geometry and materials
- [ ] **Collision Detection**: Collision shapes are properly defined and functional
- [ ] **Joint Movement**: All joints move within defined limits
- [ ] **Physics Behavior**: Robot responds correctly to gravity and forces
- [ ] **Inertial Properties**: Mass and center of mass are correctly defined
- [ ] **Sensor Integration**: Any defined sensors publish data correctly
- [ ] **ros2_control Interface**: Joint controllers respond to ROS commands

### Unity Validation Checklist
- [ ] **Model Import**: Robot model imports without errors
- [ ] **Joint Hierarchy**: Correct parent-child relationships maintained
- [ ] **Visual Fidelity**: Meshes and materials match intended appearance
- [ ] **Joint Limits**: Joint constraints properly enforced
- [ ] **ROS Connection**: Successful connection to ROS TCP Endpoint
- [ ] **State Synchronization**: Joint positions synchronize between ROS and Unity
- [ ] **Performance**: Smooth operation at target frame rate
- [ ] **Coordinate System**: Proper conversion between ROS and Unity coordinate systems

## Troubleshooting Common Issues

### Gazebo Import Issues
- **Model not appearing**: Check GAZEBO_MODEL_PATH and file permissions
- **Missing textures**: Verify mesh file paths and material definitions
- **Physics instability**: Review inertial properties and joint dynamics
- **Joint errors**: Check joint limits and types in URDF

### Unity Import Issues
- **Import errors**: Verify URDF syntax and file paths
- **Missing meshes**: Ensure all referenced mesh files exist and are accessible
- **Coordinate system mismatch**: Implement proper ROS-Unity coordinate conversion
- **Joint mapping issues**: Verify joint names match between URDF and ROS topics

## Automated Validation Scripts

### Gazebo Validation Script
```bash
#!/bin/bash
# validate_gazebo_import.sh

echo "Starting Gazebo URDF validation..."

# Launch Gazebo with empty world
gz sim -v 4 --headless-rendering &

# Wait for Gazebo to start
sleep 5

# Try to spawn the model
ros2 run gazebo_ros spawn_entity.py -entity test_robot -file /path/to/humanoid.urdf

# Check if model spawned successfully
if gz model -m test_robot --info; then
    echo "✓ Model spawned successfully"
else
    echo "✗ Model failed to spawn"
    exit 1
fi

# Check joint states
timeout 10 ros2 topic echo /joint_states --field name | head -n 10

echo "Gazebo validation completed"
pkill -f gz
```

### Unity Validation Script
```csharp
// Unity validation script for joint synchronization
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Sensor;

public class JointValidation : MonoBehaviour
{
    private ROSConnection ros;
    private ArticulationBody[] joints;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterSubscriber<JointStateMsg>("joint_states");

        // Initialize joint array based on robot structure
        InitializeJoints();
    }

    void InitializeJoints()
    {
        // Find and store references to all robot joints
        // This would be customized based on the imported model structure
    }

    public void OnJointStateMessage(JointStateMsg jointState)
    {
        // Validate that joint positions are being received
        // and match the expected joint names
    }
}
```

## Quality Assurance Process

### Pre-Import Verification
1. **URDF Validation**: Use check_urdf tool to verify URDF syntax
   ```bash
   check_urdf /path/to/humanoid.urdf
   ```

2. **Xacro Processing**: Verify xacro files compile correctly
   ```bash
   xacro /path/to/humanoid.xacro > output.urdf
   ```

3. **RViz Visualization**: Test URDF in RViz before simulation import
   ```bash
   ros2 run rviz2 rviz2
   # Add RobotModel display and set robot_description
   ```

### Post-Import Verification
1. **Kinematic Validation**: Verify forward and inverse kinematics work correctly
2. **Dynamic Validation**: Test with various joint commands and verify realistic motion
3. **Sensor Validation**: Confirm all simulated sensors publish correct data
4. **Performance Validation**: Measure simulation performance under various conditions

## Success Criteria

### Gazebo Success Metrics
- Model spawns without errors
- All joints move within specified limits
- Physics simulation runs at 1000 Hz update rate
- Joint controllers respond to ROS commands
- Sensor data published at expected rates

### Unity Success Metrics
- Model imports without errors
- All joints properly mapped to Unity hierarchy
- Smooth visualization at 30+ FPS
- Successful ROS communication established
- Joint positions synchronize correctly

### Overall Integration Success
- Both simulators maintain consistent robot state
- Minimal latency between Gazebo physics and Unity visualization
- All ROS topics and services function correctly
- Humanoid robot demonstrates stable, realistic motion