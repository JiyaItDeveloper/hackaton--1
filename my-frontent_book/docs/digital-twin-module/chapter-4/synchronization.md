# Chapter 4: Digital Twin Synchronization

## Introduction

In this chapter, we'll implement synchronization between Gazebo physics simulation and Unity visualization to create a cohesive digital twin system. This synchronization ensures that both environments maintain consistent robot states, providing accurate physics simulation with real-time visualization.

## Digital Twin Architecture

The digital twin system consists of:

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

## Synchronization Challenges

### Latency Management
- **Network Latency**: Communication delay between components
- **Simulation Step Timing**: Different update rates between simulators
- **Visualization Refresh**: Unity frame rate vs physics update rate

### State Consistency
- **Joint Position Synchronization**: Ensuring both simulators show the same pose
- **Sensor Data Consistency**: Matching sensor readings between environments
- **Timing Alignment**: Synchronizing timestamps across systems

## Implementing State Synchronization

### Timestamp Handling
Use ROS time for synchronization:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import time

class SynchronizationNode(Node):
    def __init__(self):
        super().__init__('synchronization_node')

        # Publishers and subscribers
        self.joint_pub = self.create_publisher(JointState, '/sync_joint_states', 10)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)

        # Timing management
        self.last_sync_time = self.get_clock().now()
        self.sync_timer = self.create_timer(0.01, self.synchronization_callback)  # 100Hz sync

    def joint_callback(self, msg):
        """Receive joint states from Gazebo"""
        # Store received state with timestamp
        self.gazebo_joint_state = msg
        self.gazebo_timestamp = self.get_clock().now()

    def synchronization_callback(self):
        """Synchronize states between simulators"""
        if hasattr(self, 'gazebo_joint_state'):
            # Create synchronized message
            sync_msg = self.gazebo_joint_state
            sync_msg.header.stamp = self.get_clock().now().to_msg()

            # Publish synchronized state
            self.joint_pub.publish(sync_msg)
```

### Interpolation for Smooth Motion
Implement interpolation to handle timing differences:

```csharp
using UnityEngine;
using System.Collections.Generic;

public class JointInterpolator : MonoBehaviour
{
    [System.Serializable]
    public class JointState
    {
        public string name;
        public float position;
        public float velocity;
        public double timestamp;
    }

    private Dictionary<string, JointState> currentStates = new Dictionary<string, JointState>();
    private Dictionary<string, JointState> targetStates = new Dictionary<string, JointState>();

    public void UpdateJointStates(List<JointState> newStates, double newTimestamp)
    {
        foreach (var newState in newStates)
        {
            if (targetStates.ContainsKey(newState.name))
            {
                // Store previous state for interpolation
                currentStates[newState.name] = targetStates[newState.name];
            }
            targetStates[newState.name] = newState;
        }
    }

    void Update()
    {
        double currentTime = Time.timeAsDouble;
        float interpolationFactor = 0.1f; // Adjust for smoothness

        foreach (var kvp in targetStates)
        {
            string jointName = kvp.Key;
            JointState targetState = kvp.Value;

            if (currentStates.ContainsKey(jointName))
            {
                JointState currentState = currentStates[jointName];
                // Interpolate between current and target states
                float interpolatedPosition = Mathf.Lerp(
                    (float)currentState.position,
                    (float)targetState.position,
                    interpolationFactor
                );

                // Apply to Unity joint
                ApplyJointPosition(jointName, interpolatedPosition);
            }
            else
            {
                // Direct application if no previous state
                ApplyJointPosition(jointName, (float)targetState.position);
            }
        }
    }

    private void ApplyJointPosition(string jointName, float position)
    {
        Transform jointTransform = transform.Find(jointName);
        if (jointTransform != null)
        {
            // Apply position to Unity joint (implementation depends on joint type)
            ArticulationBody joint = jointTransform.GetComponent<ArticulationBody>();
            if (joint != null)
            {
                joint.jointPosition = new Vector3(position, 0, 0);
            }
        }
    }
}
```

## Unified Control Interface

### Joint Command Distribution
Create a unified interface that sends commands to both simulators:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
import threading

class UnifiedController(Node):
    def __init__(self):
        super().__init__('unified_controller')

        # Publishers for both simulators
        self.gazebo_pub = self.create_publisher(
            JointTrajectory,
            '/humanoid_joint_trajectory_controller/joint_trajectory',
            10
        )

        # Subscriber for joint commands
        self.command_sub = self.create_subscription(
            JointTrajectory,
            '/joint_trajectory_commands',
            self.command_callback,
            10
        )

        # Timer for state monitoring
        self.monitor_timer = self.create_timer(0.01, self.monitor_states)

    def command_callback(self, msg):
        """Forward commands to both simulators"""
        # Send to Gazebo physics simulation
        self.gazebo_pub.publish(msg)

        # In a complete system, this would also send to Unity via TCP
        self.send_to_unity(msg)

    def send_to_unity(self, trajectory_msg):
        """Send trajectory to Unity via TCP"""
        # Implementation would serialize and send via ROS TCP Connector
        pass

    def monitor_states(self):
        """Monitor and validate synchronization"""
        # Implementation would compare states between simulators
        pass
```

### State Validation
Monitor synchronization quality:

```python
class SynchronizationValidator(Node):
    def __init__(self):
        super().__init__('sync_validator')
        self.state_diff_threshold = 0.01  # 1cm/1deg threshold
        self.max_latency = 0.05  # 50ms max latency

    def validate_synchronization(self, gazebo_state, unity_state):
        """Validate that both simulators are synchronized"""
        if len(gazebo_state.position) != len(unity_state.position):
            return False, "Joint count mismatch"

        max_diff = 0
        for i in range(len(gazebo_state.position)):
            diff = abs(gazebo_state.position[i] - unity_state.position[i])
            max_diff = max(max_diff, diff)

        if max_diff > self.state_diff_threshold:
            return False, f"State difference too large: {max_diff}"

        return True, "Synchronized"
```

## Fallback Mechanisms

### Connection Monitoring
Implement robust connection handling:

```csharp
using UnityEngine;
using System.Collections;

public class ConnectionMonitor : MonoBehaviour
{
    private bool gazeboConnected = true;
    private bool unityConnected = true;
    private float lastHeartbeat = 0f;
    private float heartbeatInterval = 1f;

    void Update()
    {
        CheckConnections();

        if (Time.time - lastHeartbeat > heartbeatInterval)
        {
            SendHeartbeat();
            lastHeartbeat = Time.time;
        }
    }

    void CheckConnections()
    {
        // Check if we're receiving joint states
        if (!IsReceivingData())
        {
            HandleConnectionLoss();
        }
    }

    void SendHeartbeat()
    {
        // Send heartbeat message to ROS
    }

    void HandleConnectionLoss()
    {
        if (!gazeboConnected)
        {
            // Implement fallback behavior
            Debug.LogWarning("Gazebo connection lost, using Unity-only mode");
            // Switch to Unity-only simulation if needed
        }
    }

    bool IsReceivingData()
    {
        // Check if joint states are being received
        return Time.time - lastDataReceivedTime < 2f; // 2 second timeout
    }
}
```

### Error Recovery
Implement recovery procedures:

```python
class ErrorHandler(Node):
    def __init__(self):
        super().__init__('error_handler')
        self.recovery_attempts = 0
        self.max_recovery_attempts = 3

    def handle_gazebo_disconnect(self):
        """Handle Gazebo disconnection"""
        self.get_logger().info("Gazebo disconnected, attempting recovery...")

        if self.recovery_attempts < self.max_recovery_attempts:
            # Attempt to reconnect
            self.attempt_reconnection()
            self.recovery_attempts += 1
        else:
            # Switch to safe mode
            self.activate_safe_mode()

    def attempt_reconnection(self):
        """Attempt to reconnect to Gazebo"""
        # Implementation would try to reconnect to Gazebo
        pass

    def activate_safe_mode(self):
        """Activate safe mode with Unity-only simulation"""
        # Implementation would switch to Unity-only mode
        self.get_logger().info("Activated safe mode with Unity-only simulation")
```

## Performance Optimization

### Network Optimization
Minimize network overhead:

```python
class OptimizedSyncNode(Node):
    def __init__(self):
        super().__init__('optimized_sync')
        self.compression_enabled = True
        self.data_rate_limit = 100  # Hz max
        self.last_publish_time = 0.0

    def optimized_publish(self, msg):
        """Optimized publishing with rate limiting"""
        current_time = self.get_clock().now().nanoseconds / 1e9

        if (current_time - self.last_publish_time) >= (1.0 / self.data_rate_limit):
            # Compress and publish message
            compressed_msg = self.compress_message(msg)
            self.sync_publisher.publish(compressed_msg)
            self.last_publish_time = current_time

    def compress_message(self, msg):
        """Compress message to reduce network load"""
        # Implementation would compress joint states
        # Remove redundant data, use delta encoding, etc.
        return msg
```

### Unity Performance
Optimize Unity for real-time visualization:

```csharp
using UnityEngine;
using System.Collections.Generic;

public class OptimizedVisualization : MonoBehaviour
{
    [Range(1, 60)] public int targetFrameRate = 30;
    private List<Renderer> robotRenderers = new List<Renderer>();

    void Start()
    {
        Application.targetFrameRate = targetFrameRate;
        QualitySettings.vSyncCount = 0;

        // Cache robot renderers for optimization
        robotRenderers.AddRange(GetComponentsInChildren<Renderer>());
    }

    void Update()
    {
        // Only update visualization if needed
        if (ShouldUpdateVisualization())
        {
            UpdateRobotVisualization();
        }
    }

    bool ShouldUpdateVisualization()
    {
        // Check if significant movement occurred since last update
        return Vector3.Distance(lastPosition, transform.position) > 0.01f;
    }

    void UpdateRobotVisualization()
    {
        // Update only changed components
        foreach (var renderer in robotRenderers)
        {
            // Update only if necessary
            if (renderer.material.HasProperty("_EmissionColor"))
            {
                // Update material properties as needed
            }
        }
    }
}
```

## Testing Synchronization

### Latency Testing
Test synchronization performance:

```python
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class LatencyTester(Node):
    def __init__(self):
        super().__init__('latency_tester')
        self.test_pub = self.create_publisher(Float64, '/latency_test', 10)
        self.test_sub = self.create_subscription(Float64, '/latency_result', self.result_callback, 10)
        self.start_times = {}

    def send_latency_test(self):
        """Send timestamp to measure round-trip time"""
        msg = Float64()
        msg.data = time.time()
        self.start_times[msg.data] = time.time()
        self.test_pub.publish(msg)

    def result_callback(self, msg):
        """Receive result and calculate latency"""
        start_time = self.start_times.get(msg.data)
        if start_time:
            latency = time.time() - start_time
            self.get_logger().info(f'Round-trip latency: {latency:.3f}s')
```

### Consistency Testing
Validate state consistency:

```python
class ConsistencyTester(Node):
    def __init__(self):
        super().__init__('consistency_tester')
        self.state_history = []
        self.max_state_diff = 0.01

    def test_consistency(self, gazebo_state, unity_state):
        """Test consistency between simulators"""
        # Store state history for trend analysis
        self.state_history.append((gazebo_state, unity_state))

        # Remove old entries (keep last 100)
        if len(self.state_history) > 100:
            self.state_history.pop(0)

        # Calculate consistency metrics
        diff = self.calculate_state_difference(gazebo_state, unity_state)

        if diff > self.max_state_diff:
            self.get_logger().warn(f'High state difference detected: {diff}')

        return diff <= self.max_state_diff
```

## Exercise: Implement Synchronization System

1. **Create a synchronization node** that monitors states from both simulators
2. **Implement interpolation** to smooth transitions between states
3. **Test latency** between Gazebo and Unity
4. **Validate consistency** of joint positions between simulators
5. **Implement fallback mechanisms** for connection failures

## Troubleshooting Synchronization Issues

### High Latency
- Check network configuration and bandwidth
- Verify ROS TCP Endpoint performance
- Optimize message size and frequency

### State Drift
- Verify timestamp synchronization
- Check interpolation algorithms
- Validate update rates match between systems

### Connection Failures
- Implement proper error handling
- Add reconnection logic
- Create fallback visualization modes

## Summary

In this chapter, we've covered the implementation of digital twin synchronization:
- Implemented state synchronization between Gazebo and Unity
- Created unified control interfaces
- Developed fallback mechanisms for robust operation
- Optimized performance for real-time operation
- Validated synchronization quality through testing

In the next section, we'll integrate all components into a complete digital twin system.