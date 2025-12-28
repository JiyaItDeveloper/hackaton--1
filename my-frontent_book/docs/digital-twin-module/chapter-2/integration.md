# Chapter 2: Unity Integration with ROS 2

## Introduction

Unity is a powerful 3D game engine that can be integrated with ROS 2 to provide real-time visualization and user interaction capabilities for our digital twin. In this chapter, we'll set up Unity with ROS 2 communication and learn how to create compelling visualizations for our humanoid robot.

## Unity-ROS 2 Integration Overview

The Unity-ROS 2 integration is facilitated through the Unity Robotics Hub, which provides:

- **ROS TCP Connector**: Establishes communication between Unity and ROS 2
- **Message Serialization**: Handles ROS message types in Unity C#
- **URDF Importer**: Imports URDF models directly into Unity
- **Sensor Visualization**: Visualizes sensor data in the Unity environment

## Setting Up Unity for ROS 2

### Prerequisites
- Unity 2021.3 LTS or later
- ROS 2 Humble Hawksbill
- Unity Robotics Hub package
- URDF Importer package

### Installation Process

1. **Install Unity Hub and Unity 2021.3 LTS+**
2. **Install ROS TCP Endpoint**:
   ```bash
   pip3 install ros-tcp-endpoint
   ```

3. **In Unity, install required packages**:
   - URDF Importer from Unity Asset Store or Package Manager
   - ROS TCP Connector from Unity Robotics Hub

## ROS TCP Communication Architecture

The communication between Unity and ROS 2 uses TCP/IP protocol:

```
ROS 2 Nodes ←→ ROS TCP Endpoint ←→ Unity ROS TCP Connector
```

The ROS TCP Endpoint acts as a bridge between ROS 2 and Unity, handling message serialization and deserialization.

## Setting Up ROS TCP Connector in Unity

1. **Create an empty GameObject** in your Unity scene
2. **Add ROS TCP Connector component** to the GameObject
3. **Configure the IP address and port** (default: 10000)

```csharp
using Unity.Robotics.ROSTCPConnector;

public class RobotController : MonoBehaviour
{
    private ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Connect("127.0.0.1", 10000); // Connect to local ROS endpoint
    }
}
```

## Importing URDF Models into Unity

The URDF Importer allows you to directly import your robot model:

1. **Go to Assets → Import Robot from URDF**
2. **Select your URDF file location**
3. **Configure import settings**:
   - Base Path: Root directory containing meshes
   - Collision Handling: Choose appropriate collision settings
   - Joint Configuration: Preserve joint limits and types

### Coordinate System Conversion

Unity uses a left-handed coordinate system (Z-forward), while ROS uses a right-handed system (Z-up). The URDF Importer handles this conversion automatically.

## Synchronizing Robot State in Unity

To synchronize the robot's joint states from ROS 2 to Unity:

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Sensor;

public class JointSynchronizer : MonoBehaviour
{
    [SerializeField] private ArticulationBody[] joints;
    private ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<JointStateMsg>("joint_states", OnJointStateReceived);
    }

    void OnJointStateReceived(JointStateMsg jointState)
    {
        for (int i = 0; i < jointState.name.Count; i++)
        {
            string jointName = jointState.name[i];
            float jointPosition = jointState.position[i];

            // Find and update the corresponding Unity joint
            ArticulationBody joint = FindJointByName(jointName);
            if (joint != null)
            {
                joint.jointPosition = new Vector3(jointPosition, 0, 0);
            }
        }
    }

    private ArticulationBody FindJointByName(string name)
    {
        // Implementation to find joint by name
        return transform.Find(name)?.GetComponent<ArticulationBody>();
    }
}
```

## Creating Sensor Visualizations

Unity can visualize sensor data from ROS 2:

```csharp
using Unity.Robotics.ROSTCPConnector.MessageTypes.Sensor;

public class SensorVisualizer : MonoBehaviour
{
    private void Start()
    {
        ROSConnection.GetOrCreateInstance()
            .Subscribe<ImuMsg>("imu/data", OnImuDataReceived);
    }

    void OnImuDataReceived(ImuMsg imuData)
    {
        // Visualize IMU data in Unity
        Debug.Log($"Orientation: {imuData.orientation}");
        Debug.Log($"Angular Velocity: {imuData.angular_velocity}");
        Debug.Log($"Linear Acceleration: {imuData.linear_acceleration}");
    }
}
```

## Unity Scene Setup for Digital Twin

### Basic Scene Structure
```
Main Camera
├── ROS TCP Connector (empty GameObject with ROS TCP Connector component)
├── Humanoid Robot (imported from URDF)
├── Lighting (Directional light, etc.)
└── UI Canvas (for controls and information display)
```

### Performance Optimization Tips
- Use appropriate Level of Detail (LOD) for complex models
- Optimize mesh complexity for real-time rendering
- Use occlusion culling for large environments
- Implement object pooling for frequently instantiated objects

## Exercise: Import and Visualize Robot Model

1. **Import your humanoid URDF model** into Unity using the URDF Importer
2. **Set up ROS TCP Connector** in a new Unity scene
3. **Create a simple joint synchronizer** script to visualize joint positions
4. **Test the connection** with a basic ROS 2 publisher

## Troubleshooting Common Issues

### Connection Issues
- Verify IP addresses and port numbers match between Unity and ROS TCP Endpoint
- Check firewall settings that might block TCP connections
- Ensure ROS TCP Endpoint is running before starting Unity

### Coordinate System Issues
- Verify that joint orientations match between ROS and Unity
- Check that visual and collision meshes align properly

### Performance Issues
- Reduce polygon count of imported meshes
- Use appropriate texture resolutions
- Consider using Unity's Profiler to identify bottlenecks

## Summary

In this chapter, we've covered the fundamentals of integrating Unity with ROS 2:
- Setting up the Unity-ROS 2 communication infrastructure
- Importing URDF models into Unity
- Synchronizing robot state between ROS 2 and Unity
- Creating sensor visualizations
- Optimizing performance for real-time applications

In the next chapter, we'll dive into implementing physics simulation and realistic humanoid motion in our digital twin environment.