---
id: unity-visualization
title: Interaction & Visualization with Unity
---

# Chapter 3: Interaction & Visualization with Unity

## Visual Realism with Unity

### Why Photorealistic Visualization Matters

While Gazebo excels at physics simulation, its graphics are basic. **Unity** provides photorealistic rendering that's critical for:

**1. Stakeholder Demonstrations**
- Investors and executives respond better to beautiful visuals
- Realistic environments make robot capabilities tangible
- Professional presentations require high-quality graphics

**2. Perception Testing**
- AI vision models need realistic images for training
- Test object detection in photorealistic homes/offices
- Validate perception in varied lighting conditions

**3. Human-Robot Interaction (HRI) Research**
- Study how humans react to robot movements
- Test social navigation in realistic crowds
- Validate safety systems with realistic human models

**4. Marketing and Public Engagement**
- Create compelling videos for product launches
- Generate promotional materials without physical robots
- Enable virtual try-before-you-buy experiences

---

### Unity's Visual Capabilities

**Real-Time Rendering Features**:
- **Physically-Based Rendering (PBR)**: Realistic materials (metal, plastic, fabric)
- **Global Illumination**: Accurate light bouncing and shadows
- **Post-Processing**: Bloom, ambient occlusion, depth of field
- **Particle Systems**: Dust, smoke, water effects

**Example Comparison**:

| Gazebo Rendering | Unity Rendering |
|------------------|-----------------|
| Flat shading | Realistic lighting and shadows |
| Simple textures | High-resolution PBR materials |
| Basic colors | Accurate metal/plastic/fabric appearance |
| Functional | Photorealistic |

**When visual quality matters**: Demos, perception testing, HRI research, marketing

---

## Human-Robot Interaction (HRI) Scenarios

### Why Test HRI in Simulation?

Testing how humans and robots interact safely is critical before deployment, but difficult with real hardware:

- **Safety risks**: Physical robot could harm people if buggy
- **Repeatability**: Hard to get same human behavior twice in real tests
- **Cost**: Requires human participants, lab space, safety equipment
- **Edge cases**: Can't ethically test dangerous scenarios (collision, fall)

**Unity solution**: Simulate realistic human models and test interactions safely.

---

### HRI Scenarios Testable in Unity

**1. Social Navigation**
- Robot navigates crowded office/hallway
- Maintains safe distances from humans (social comfort zones)
- Yields right-of-way appropriately
- Avoids sudden movements that startle people

**2. Gesture and Gaze**
- Robot makes eye contact during interaction
- Uses hand gestures to communicate
- Points to objects or directions
- Nods/shakes head for yes/no

**3. Speech and Audio Interaction**
- Robot speaks with appropriate volume and tone
- Responds to voice commands from humans
- Provides audio feedback (beeps, spoken words)

**4. Collaborative Tasks**
- Robot hands object to human
- Follows human leader in navigation
- Responds to pointing or guiding gestures
- Maintains safe distance during collaboration

**5. Safety Scenarios**
- Human suddenly walks in front of robot
- Robot detects and stops to avoid collision
- Emergency stop triggers on unexpected contact
- Recovery behaviors after near-miss

---

### Simulated Humans in Unity

**Unity Asset Store** provides realistic human models:
- Animated walking/sitting/standing behaviors
- Varied ages, body types, clothing
- Scripted paths and interaction behaviors

**AI-Driven Humans**: Use Unity's ML-Agents to create humans that:
- Navigate environment autonomously
- React to robot presence
- Simulate realistic crowd behaviors

---

## State Synchronization: Gazebo ↔ Unity

### The Synchronization Challenge

For a true digital twin, **Gazebo (physics)** and **Unity (visualization)** must show the **same robot state** at the **same time**.

**Key state to synchronize**:
- Joint angles (shoulder, elbow, hip, knee, etc.)
- Link poses (position and orientation of each body part)
- Sensor data (camera images, point clouds)
- Contact forces (feet on ground)

---

### ROS 2 Bridge Architecture

**Approach**: Use ROS 2 as the communication layer between Gazebo and Unity

```
┌──────────────────┐         ┌──────────────────┐
│  Gazebo (Linux)  │         │  Unity (Win/Mac) │
│                  │         │                  │
│  Physics @ 1kHz  │         │  Render @ 60Hz   │
│                  │         │                  │
│  Publishes:      │  ROS 2  │  Subscribes:     │
│  /joint_states ──┼────────>│  /joint_states   │
│  /tf            ──┼────────>│  /tf             │
│  /odom          ──┼────────>│  /odom           │
│                  │         │                  │
└──────────────────┘         └──────────────────┘
```

**ROS 2 Topics for Synchronization**:
- `/joint_states`: All joint positions and velocities
- `/tf`: Transform tree (coordinate frames)
- `/odom`: Robot's global position and orientation
- `/robot_description`: URDF model (loaded once at startup)

---

### Synchronization Implementation Concepts

**In Gazebo**:
1. Physics simulation runs at high frequency (1000 Hz)
2. Gazebo ROS plugin publishes joint states at 60-100 Hz
3. TF broadcaster publishes transform tree

**In Unity**:
1. ROS# or ROS-TCP-Connector subscribes to ROS 2 topics
2. Unity receives joint state messages
3. Unity updates robot's visual model to match joint angles
4. Renders at 30-60 FPS

**Latency Considerations**:
- Network delay: 1-10ms (local network) to 50-200ms (internet)
- Unity aims to render state that's 16-33ms old (acceptable lag)
- Prediction can smooth motion if delay is noticeable

---

### Data Flow Example

**Gazebo publishes** (60 Hz):
```python
# Pseudocode: Gazebo ROS plugin
joint_state_msg = JointState()
joint_state_msg.name = ["left_hip", "left_knee", "left_ankle", ...]
joint_state_msg.position = [0.3, -0.5, 0.2, ...]  # radians
joint_state_msg.velocity = [0.1, -0.2, 0.15, ...]  # rad/s

publish("/joint_states", joint_state_msg)
```

**Unity receives** (60 Hz):
```csharp
// Pseudocode: Unity ROS subscriber
void OnJointStatesReceived(JointState msg) {
    for (int i = 0; i < msg.name.Length; i++) {
        string jointName = msg.name[i];
        float angle = msg.position[i];

        // Update Unity's robot model
        ArticulationBody joint = FindJoint(jointName);
        joint.SetDriveTarget(angle * Mathf.Rad2Deg);  // Convert to degrees
    }
}
```

---

## Benefits of Gazebo-Unity Integration

### Complementary Strengths

**Gazebo provides**:
- Accurate physics for controller validation
- Fast simulation without GPU requirements
- Easy ROS 2 integration

**Unity provides**:
- Beautiful visuals for demos and HRI
- Rich environments (offices, homes, outdoors)
- Human models and crowd simulation

**Together**:
- **Physics** (Gazebo) + **Graphics** (Unity) = Complete digital twin
- Same ROS 2 code runs on both simulation and real robot
- Validate mechanics in Gazebo, then show stakeholders in Unity

---

### Use Case: Humanoid Demo Pipeline

**Development Workflow**:
1. Design walking gait in code
2. Test in **Gazebo**: Verify balance, joint limits, stability (fast iteration)
3. Validate in **Unity**: Show robot walking in photorealistic office (stakeholder demo)
4. Deploy to **real robot**: Same ROS 2 code works on hardware

**Why this works**:
- Gazebo catches mechanical issues quickly
- Unity validates perception and HRI safely
- Real robot deployment has high confidence (tested both physics and perception)

---

## Chapter Summary

✅ **Unity provides** photorealistic rendering for demos, HRI research, and perception testing
✅ **HRI scenarios** (navigation, gestures, collaboration) can be tested safely in Unity
✅ **Synchronization** uses ROS 2 topics to share robot state between Gazebo and Unity
✅ **Complementary tools**: Gazebo for physics, Unity for visuals = complete digital twin
✅ **Integrated workflow**: Test mechanics in Gazebo, demonstrate in Unity, deploy to hardware

---

## Up Next

In [Chapter 4: Simulated Sensors](../chapter-4/simulated-sensors.md), you'll learn how to simulate LiDAR, depth cameras, and IMUs with realistic noise for training AI perception systems.

**Key Concepts to Remember**:
- Unity = photorealistic visualization and HRI testing
- ROS 2 bridge = synchronizes Gazebo physics with Unity rendering
- HRI scenarios = safe testing of human-robot interaction
- Complementary tools = use both for complete digital twin
