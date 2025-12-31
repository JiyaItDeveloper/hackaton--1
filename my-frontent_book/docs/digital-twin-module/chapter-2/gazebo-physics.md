---
id: gazebo-physics-simulation
title: Physics Simulation with Gazebo
---

# Chapter 2: Physics Simulation with Gazebo

## Physics Engine Basics

### Gazebo's Physics Engines

Gazebo supports multiple physics engines, each with different trade-offs:

| Engine | Strengths | Best For | Typical Use |
|--------|-----------|----------|-------------|
| **ODE** (Open Dynamics Engine) | Fast, stable, well-tested | General robotics | Default choice for most projects |
| **Bullet** | Fast collision detection, soft bodies | Complex collisions, deformable objects | Grasping, manipulation |
| **DART** (Dynamic Animation and Robotics Toolkit) | Accurate, handles complex constraints | Research, precise simulations | Humanoid locomotion, legged robots |

**For humanoid robots**: DART is often preferred due to its accurate contact modeling and constraint handling, critical for bipedal balance.

---

### How Physics Engines Work

Physics engines simulate the real world by:

**1. Computing Forces**
- Gravity pulls objects downward
- Contact forces prevent objects from interpenetrating
- Joint forces from motors and springs

**2. Solving Constraints**
- Joint limits (e.g., elbow can't bend backwards)
- Contact constraints (feet on ground don't penetrate)
- Velocity/position constraints (fixed joints stay fixed)

**3. Updating Positions**
- Integrate forces → accelerations → velocities → positions
- Typically run at 1000 Hz (1ms timestep) for stability
- Rendered visuals update at 30-60 Hz for display

---

### Key Physics Parameters

**Gravity**:
```xml
<gravity>0 0 -9.81</gravity>
```
- Standard Earth gravity: 9.81 m/s² downward (negative Z)
- Can be modified to simulate Moon (1.62 m/s²), Mars (3.71 m/s²), or zero-G

**Time Step**:
- Smaller timestep = more accurate but slower
- Typical: 0.001s (1000 Hz) for humanoids
- Too large → unstable simulation, robots "explode"

**Collision Properties**:
- **Friction** (mu): Surface grip (0 = ice, 1+ = rubber)
- **Restitution**: Bounciness (0 = no bounce, 1 = perfectly elastic)
- **Contact stiffness**: How rigid contacts are

---

## Loading URDF Models

### What Happens When Loading a URDF

When you load a URDF humanoid model into Gazebo:

**Step 1: Parse URDF XML**
- Read robot structure (links, joints, sensors)
- Extract visual meshes (.stl, .dae files)
- Extract collision meshes (simplified geometry)
- Read inertial properties (mass, center of mass, inertia tensor)

**Step 2: Create Physics Objects**
- Physics engine creates rigid bodies for each link
- Joints are implemented as constraints between bodies
- Collision geometries are registered with collision detector

**Step 3: Spawn in World**
- Place robot at specified position and orientation
- Initialize joint angles to default or specified values
- Activate gravity and contact forces

**Step 4: Start Simulation**
- Physics engine begins computing forces and updating state
- ROS 2 plugins publish joint states, sensor data
- Control commands from ROS 2 applied as joint torques

---

### URDF to Gazebo Mapping

**Links** (URDF) → **Rigid Bodies** (Gazebo):
- Mass and inertia from `<inertial>` tag
- Visual appearance from `<visual>` meshes
- Collision shapes from `<collision>` geometries

**Joints** (URDF) → **Constraints** (Gazebo):
- Revolute joint → hinge with limits
- Continuous joint → hinge without limits
- Fixed joint → weld constraint
- Prismatic joint → slider constraint

**Sensors** (URDF/Gazebo) → **Simulated Data** (ROS 2):
- Camera → RGB images on `/camera/image_raw`
- LiDAR → Point clouds on `/scan`
- IMU → Orientation, angular velocity, linear acceleration on `/imu/data`

---

### Example: Conceptual URDF Snippet

```xml
<!-- Simplified humanoid torso link -->
<link name="torso">
  <inertial>
    <mass value="15.0"/>  <!-- 15 kg torso -->
    <inertia ixx="0.5" ixy="0" ixz="0" iyy="0.5" iyz="0" izz="0.3"/>
  </inertial>

  <visual>
    <geometry>
      <mesh filename="torso.stl"/>  <!-- 3D model for rendering -->
    </geometry>
  </visual>

  <collision>
    <geometry>
      <box size="0.4 0.3 0.6"/>  <!-- Simplified collision box -->
    </geometry>
  </collision>
</link>

<!-- Hip joint connecting pelvis to left thigh -->
<joint name="left_hip" type="revolute">
  <parent link="pelvis"/>
  <child link="left_thigh"/>
  <axis xyz="0 1 0"/>  <!-- Rotation around Y-axis -->
  <limit effort="100" velocity="2.0" lower="-1.57" upper="1.57"/>
</joint>
```

**What Gazebo does with this**:
- Creates a 15kg rigid body for the torso
- Applies gravity force (15kg × 9.81 m/s² = 147N downward)
- Creates hinge joint with ±90° range and 100Nm max torque
- Detects collisions using simplified box shape (faster than mesh)

---

## Motion Validation

### Why Validate Motion in Simulation?

Before deploying a walking controller to a real $100,000 humanoid, you want to verify:
- ✅ Robot doesn't fall over
- ✅ Joints stay within safe limits
- ✅ No self-collisions (e.g., knees don't hit chest)
- ✅ Foot contacts are stable
- ✅ Balance is maintained during gait

**Simulation lets you catch these issues** before risking hardware damage.

---

### Validation Techniques

**1. Visual Inspection**
- Watch robot walk in Gazebo 3D view
- Check for unnatural motions, vibrations, or instability
- Verify feet make proper ground contact

**2. Joint Limit Monitoring**
- Monitor joint angles via ROS 2 `/joint_states` topic
- Ensure values stay within `<limit lower="..." upper="..."/>` bounds
- Plot joint trajectories to identify violations

**3. Stability Metrics**
- **Center of Mass (COM)**: Should stay within support polygon (footprint)
- **Zero Moment Point (ZMP)**: Should be inside foot contact area
- **Tip-over risk**: If COM moves outside support, robot will fall

**4. Contact Force Analysis**
- Monitor foot-ground contact forces
- Verify forces are reasonable (not too high = impact, not zero = floating)
- Check for even weight distribution between feet during double support

**5. Energy Consumption**
- Measure total joint torques and velocities
- Identify inefficient motions that waste energy
- Optimize gait for real robot battery life

---

### Validation Checklist

Before deploying walking gait to real robot:

- [ ] Robot maintains upright posture for entire gait cycle
- [ ] All joint angles remain within URDF-specified limits
- [ ] No self-collisions detected (check Gazebo collision plugin output)
- [ ] Foot contacts occur at expected times in gait
- [ ] Center of mass remains above support polygon (no tip-over)
- [ ] No sudden jumps or discontinuities in motion
- [ ] Forward velocity matches commanded speed (±10%)
- [ ] Robot can start, stop, and turn without falling

**If any item fails**: Debug in simulation, fix controller, re-test. Repeat until all pass.

---

## Physics Simulation for Humanoid Challenges

### Specific Challenges for Bipedal Robots

**1. Balance and Stability**
- Humanoids have high center of mass, narrow base of support
- Small errors → tipping over
- Physics engine must accurately model dynamic balance

**2. Contact Modeling**
- Foot-ground contact is critical for walking
- Need accurate friction to prevent slipping
- Contact forces must be stable (no jitter)

**3. Computational Cost**
- Humanoids have 20-50+ degrees of freedom
- Many joints = expensive constraint solving
- Must run faster than real-time (1000 Hz physics, 30 Hz display)

**4. Singularities and Numerical Issues**
- Fully extended limbs can cause solver issues
- Joint limits at boundaries require careful handling
- Physics engines use various tricks to maintain stability

---

## Gazebo + ROS 2 Integration

### Data Flow

```
┌──────────────────┐
│  Gazebo Physics  │
│  - Simulates     │
│  - Computes      │
│    forces        │
└────────┬─────────┘
         │ ROS 2 topics
         ↓
┌──────────────────┐
│  Your Code       │
│  - Controllers   │
│  - Planners      │
│  - AI models     │
└────────┬─────────┘
         │ Commands
         ↓
┌──────────────────┐
│  Gazebo Actuators│
│  - Apply torques │
│  - Move joints   │
└──────────────────┘
```

**Key ROS 2 Topics**:
- `/joint_states` (sensor_msgs/JointState): Current joint positions/velocities
- `/cmd_vel` (geometry_msgs/Twist): Velocity commands for mobile base
- `/joint_trajectory` (trajectory_msgs/JointTrajectory): Joint position commands

---

## Chapter Summary

✅ **Physics engines** simulate gravity, collisions, and joint dynamics
✅ **Gazebo supports** ODE, Bullet, and DART engines (DART best for humanoids)
✅ **URDF models** are converted to physics objects when loaded into Gazebo
✅ **Motion validation** catches control bugs before hardware deployment
✅ **Simulation-first** saves time and money by testing virtually first

---

## Up Next

In [Chapter 3: Interaction & Visualization with Unity](../chapter-3/unity-visualization.md), you'll learn how Unity provides photorealistic rendering and human-robot interaction testing capabilities.

**Key Concepts to Remember**:
- Physics engine = simulates forces, constraints, collisions
- URDF → Gazebo = links become rigid bodies, joints become constraints
- Validation = verify motion is safe before hardware deployment
- DART engine = best for humanoid locomotion
