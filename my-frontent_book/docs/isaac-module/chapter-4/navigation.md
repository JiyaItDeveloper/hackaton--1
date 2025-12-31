---
id: navigation
title: Nav2 Navigation for Humanoids
---

# Chapter 4: Nav2 Navigation for Humanoids

## Nav2 Architecture

### What is Nav2?

**Nav2** (Navigation2) is the standard ROS 2 navigation framework that enables mobile robots to autonomously navigate from point A to point B while avoiding obstacles.

Originally designed for wheeled robots, Nav2 can be adapted for bipedal humanoid robots with special considerations for stability and locomotion constraints.

### Core Nav2 Components

**1. Global Planner**
- Computes high-level path from start to goal
- Uses global costmap (static map + known obstacles)
- Plans collision-free route at low frequency (1-5 Hz)

**2. Local Planner (Controller)**
- Generates velocity commands to follow global path
- Uses local costmap (recent sensor data)
- Reacts to dynamic obstacles at high frequency (10-30 Hz)

**3. Costmap Generator**
- Creates 2D occupancy grids showing obstacle locations
- Combines map data, sensor data, and inflation layers
- Provides safe/unsafe regions for planning

**4. Recovery Behaviors**
- Actions to take when stuck (back up, rotate, clear costmap)
- Prevents robot from getting permanently stuck

**5. Behavior Tree (BT) Navigator**
- Coordinates all navigation components
- Manages navigation states and error handling

---

## Costmap Generation

### What are Costmaps?

A **costmap** is a 2D grid where each cell has a value representing navigation cost:
- **0**: Free space (safe to traverse)
- **100-253**: Inflated obstacle zone (discouraged but traversable)
- **254**: Inscribed obstacle (robot footprint would collide)
- **255**: Lethal obstacle (certain collision)

### Global vs Local Costmaps

| Global Costmap | Local Costmap |
|----------------|---------------|
| Large area (10-100m) | Small area around robot (5-10m) |
| Static map + sensors | Recent sensor data only |
| Updated slowly (1 Hz) | Updated frequently (5-10 Hz) |
| Long-term planning | Reactive control |

### Costmap Layers

Nav2 costmaps consist of multiple layers combined together:

**1. Static Layer**
- Pre-built map from SLAM or manual creation
- Represents known walls, furniture, fixed obstacles

**2. Obstacle Layer**
- Real-time sensor observations (LiDAR, depth cameras)
- Detects dynamic obstacles (people, moving objects)

**3. Inflation Layer**
- Expands obstacles by robot radius
- Creates safety margin around obstacles

**4. Voxel Layer (3D)**
- For humanoid robots with height considerations
- Tracks obstacles at multiple heights (tables, shelves)

---

## Humanoid Navigation Challenges

### Why Humanoids Are Different

Wheeled robots can:
- Turn in place (differential drive)
- Move instantly in any direction (omni wheels)
- Stop and start quickly

Humanoid robots:
- Must maintain dynamic balance while walking
- Take discrete footsteps (can't move smoothly)
- Need time to shift weight and change direction
- Risk falling if commanded to stop/turn too quickly

### Key Differences from Wheeled Robots

| Constraint | Wheeled Robot | Humanoid Robot |
|------------|---------------|----------------|
| **Minimum turn radius** | 0m (can spin in place) | 0.5-1.0m (needs space to pivot) |
| **Acceleration** | Instant | Gradual (balance constraints) |
| **Stopping distance** | Near-zero | 0.3-1.0m (momentum + stability) |
| **Step constraints** | Continuous motion | Discrete footsteps |
| **Vertical clearance** | Fixed height | Variable (can duck, reach) |

---

## Humanoid-Specific Planning

### Footstep Planning Integration

For true humanoid navigation, Nav2's velocity commands should be translated to **footstep plans**:

**Nav2 Output**: Linear velocity (m/s) + Angular velocity (rad/s)
**Humanoid Requirement**: Sequence of foot placements

**Footstep Planner**:
1. Receives Nav2 velocity command
2. Computes feasible foot placements
3. Checks balance stability for each step
4. Sends foot poses to locomotion controller

### Balance-Aware Local Planning

Standard Nav2 local planners (DWA, TEB) assume instant velocity changes. For humanoids, we need:

**Modified cost function**:
```
Total Cost = Path Cost + Obstacle Cost + Balance Cost + Stability Cost

Balance Cost = Penalty for rapid direction changes
Stability Cost = Penalty for paths requiring single-leg stance duration > threshold
```

### Gait-Aware Replanning

**Challenge**: Humanoid takes 0.5-1.0 seconds per step

**Solution**: Local planner must predict future robot pose accounting for current gait phase:
- If mid-stride: Cannot change direction until foot lands
- If double-support: Can initiate turn or stop
- If single-leg stance: Limited maneuverability

---

## Isaac Perception to Nav2 Integration

### Data Flow Pipeline

```
┌──────────────────────────────────────────────────────────┐
│                    Isaac ROS Perception                  │
├──────────────────────────────────────────────────────────┤
│                                                          │
│  Visual SLAM ──> Odometry (/odom → /base_link)         │
│       │                                                  │
│       └──> Point Cloud (/visual_slam/vis/map_points)    │
│                                                          │
│  Depth Camera ──> Obstacle Points (/camera/depth/points)│
│                                                          │
└──────────────────────────────────────────────────────────┘
                           │
                           v
┌──────────────────────────────────────────────────────────┐
│                     Nav2 Stack                           │
├──────────────────────────────────────────────────────────┤
│                                                          │
│  Costmap 2D ──> Combines odometry + point clouds        │
│       │                                                  │
│       ├──> Global Planner ──> Path to Goal              │
│       │                                                  │
│       └──> Local Planner ──> Velocity Commands          │
│                                                          │
└──────────────────────────────────────────────────────────┘
                           │
                           v
                  Humanoid Locomotion Controller
                  (Footstep Planning + Balance)
```

### ROS 2 Topic Mapping

**Isaac ROS Outputs → Nav2 Inputs**:
- `/visual_slam/tracking/odometry` → `/odom` (Odometry)
- `/camera/depth/points` → `/point_cloud` (PointCloud2)
- `/visual_slam/vis/map_points` → `/map` (if using VSLAM map)

**Nav2 Outputs → Humanoid Controller**:
- `/cmd_vel` (geometry_msgs/Twist) → Footstep planner

---

## Nav2 Configuration for Humanoids

### Example Configuration Snippet

```yaml
# humanoid_nav2_params.yaml

controller_server:
  ros__parameters:
    controller_frequency: 20.0  # Higher for humanoid reactivity
    min_x_velocity_threshold: 0.1  # Slower than wheeled robots
    min_y_velocity_threshold: 0.0  # No lateral motion (bipedal)
    min_theta_velocity_threshold: 0.05  # Gradual turns

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 10.0
      publish_frequency: 5.0
      width: 5  # Smaller window for humanoids
      height: 5
      resolution: 0.05
      robot_radius: 0.3  # Humanoid footprint

      # Inflation for safety margin
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        inflation_radius: 0.8  # Larger for humanoid stability
        cost_scaling_factor: 5.0

global_costmap:
  global_costmap:
    ros__parameters:
      robot_radius: 0.3
      resolution: 0.1
      track_unknown_space: true

planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5  # Looser for humanoids
      use_astar: true
```

---

## Dynamic Replanning for Obstacle Avoidance

### Real-Time Replanning

**Scenario**: Humanoid walking toward goal, person steps in path

**Nav2 Response**:
1. Local costmap detects new obstacle (from Isaac ROS depth camera)
2. Local planner computes new velocity to avoid obstacle
3. If path blocked: Global planner computes new route
4. Behavior tree manages replanning vs recovery

**Humanoid consideration**: Replanning mid-gait requires:
- Checking if current footstep can be aborted safely
- If not: Complete current step, then replan
- Smooth velocity transitions to maintain balance

---

## Chapter Summary

✅ **Nav2** provides global and local planning for autonomous navigation
✅ **Costmaps** represent safe and unsafe navigation areas from sensor data
✅ **Humanoids** require balance-aware planning with discrete footsteps
✅ **Isaac perception** provides odometry and obstacle data to Nav2
✅ **Dynamic replanning** enables humanoids to avoid obstacles while walking

---

## Module Complete!

Congratulations! You've completed Module 03 (Isaac Module). You now understand:
- The NVIDIA Isaac ecosystem (Isaac Sim, Isaac ROS)
- Synthetic data generation for AI training
- GPU-accelerated perception with Visual SLAM
- Autonomous navigation with Nav2 for humanoids

**Next Steps**: Apply these concepts in the Integration Project to build a complete autonomous humanoid system!
