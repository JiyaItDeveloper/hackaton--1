---
id: simulated-sensors
title: Simulated Sensors
---

# Chapter 4: Simulated Sensors

## Sensor Types in Digital Twins

### Why Simulate Sensors?

Sensors are how robots perceive their environment. For AI development, you need:
- **Thousands of labeled images** for vision models
- **Diverse scenarios** (different lighting, obstacles, environments)
- **Ground truth data** (perfect labels for supervised learning)

**Problem**: Collecting this data on real robots is expensive and time-consuming.
**Solution**: Simulate sensors in Gazebo/Unity to generate unlimited training data.

---

### 1. LiDAR (Light Detection and Ranging)

**What it measures**: Distance to obstacles by measuring laser pulse travel time

**Simulated output**: 2D array of distances (e.g., 360 measurements in a circle)

**Use cases**:
- Obstacle detection for navigation
- 3D mapping (SLAM)
- Collision avoidance

**Gazebo LiDAR configuration example**:
```xml
<!-- Conceptual Gazebo sensor definition -->
<sensor name="lidar" type="ray">
  <ray>
    <scan>
      <horizontal>
        <samples>360</samples>  <!-- 360 degree scan -->
        <min_angle>-3.14</min_angle>
        <max_angle>3.14</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.1</min>  <!-- Minimum range: 10cm -->
      <max>30.0</max>  <!-- Maximum range: 30m -->
    </range>
  </ray>
</sensor>
```

**ROS 2 Output**: `/scan` topic with `sensor_msgs/LaserScan` messages

---

### 2. Depth Cameras

**What they measure**: Distance to every pixel (creates 3D point cloud)

**Simulated output**: Image where each pixel's value = distance to surface

**Use cases**:
- 3D object detection
- Grasping pose estimation
- Dense mapping
- Obstacle avoidance

**Types**:
- **Stereo cameras**: Two cameras compute depth from disparity
- **Structured light**: Projects pattern, measures distortion
- **Time-of-Flight**: Measures light travel time per pixel

**ROS 2 Output**:
- `/camera/depth/image_raw` (Depth image)
- `/camera/depth/points` (PointCloud2 - 3D points)

---

### 3. RGB Cameras

**What they measure**: Standard color images

**Simulated output**: Image like a regular camera photo

**Use cases**:
- Object recognition
- Visual SLAM (from Module 03)
- Human detection
- Scene understanding

**Unity advantage**: Photorealistic images better for training vision AI

**ROS 2 Output**: `/camera/image_raw` (sensor_msgs/Image)

---

### 4. IMU (Inertial Measurement Unit)

**What it measures**:
- **Accelerometer**: Linear acceleration (including gravity)
- **Gyroscope**: Angular velocity (rotation rates)
- **Magnetometer**: Magnetic field direction (compass)

**Simulated output**: 3D vectors for acceleration, angular velocity, orientation

**Use cases**:
- Balance control (critical for humanoids)
- Sensor fusion with cameras (Visual-Inertial Odometry)
- Detecting falls or impacts

**ROS 2 Output**: `/imu/data` (sensor_msgs/Imu)

---

### 5. Force/Torque Sensors

**What they measure**: Forces and torques at joints or contacts

**Simulated output**: 6D wrench (force XYZ, torque XYZ)

**Use cases**:
- Foot-ground contact detection (is foot touching floor?)
- Grasping force control
- Collision detection

**Example**: Foot force sensors detect when humanoid's foot contacts ground during walking.

---

## Sensor Realism and Noise

### Why Add Noise?

**Perfect sensors** in simulation create a **sim-to-real gap** — AI trained on perfect data fails on noisy real sensors.

**Sources of real sensor noise:**
- **Measurement noise**: Random fluctuations in readings
- **Bias/drift**: Systematic errors over time
- **Occlusions**: Missing data (LiDAR blocked by obstacles)
- **Environmental factors**: Lighting, reflections, vibrations

### Gaussian Noise Model

Most sensors use **Gaussian (normal) distribution** noise:

```
measurement = true_value + N(mean, stddev)
```

**Parameters**:
- **Mean**: Average error (often 0 for unbiased sensors)
- **Standard deviation**: Spread of noise

**Example: IMU Accelerometer**
- True acceleration: `[0.0, 0.0, -9.81]` m/s² (gravity)
- Noise: `stddev = 0.01` m/s²
- Measured: `[0.003, -0.007, -9.815]` m/s² (slightly noisy)

---

### Noise Parameters for Common Sensors

| Sensor | Typical Noise Level | Impact |
|--------|---------------------|--------|
| **LiDAR** | ±2cm @ 10m | Small position errors |
| **Depth Camera** | ±5mm @ 1m | Affects grasping accuracy |
| **RGB Camera** | ±5% pixel intensity | Slight color variations |
| **IMU Accel** | ±0.01 m/s² | Affects dead reckoning |
| **IMU Gyro** | ±0.01 rad/s | Orientation drift over time |

**Tuning noise levels**: Start with values from real sensor datasheets, adjust if sim-to-real gap persists.

---

### Adding Noise in Gazebo

```xml
<!-- Conceptual: Add Gaussian noise to IMU -->
<sensor name="imu" type="imu">
  <imu>
    <noise>
      <type>gaussian</type>
      <rate>
        <mean>0.0</mean>
        <stddev>0.01</stddev>  <!-- Gyro noise: 0.01 rad/s -->
      </rate>
      <accel>
        <mean>0.0</mean>
        <stddev>0.01</stddev>  <!-- Accel noise: 0.01 m/s² -->
      </accel>
    </noise>
  </imu>
</sensor>
```

**Effect**: IMU data now includes realistic noise, making AI models robust to real sensor imperfections.

---

## Data Flow to AI Systems

### From Simulation to AI Training

**Pipeline**:
```
┌────────────────┐
│ Gazebo/Unity   │
│ Simulated      │
│ Sensors        │
└───────┬────────┘
        │ ROS 2 topics
        ↓
┌────────────────┐
│ ROS 2 Nodes    │
│ - Record bags  │
│ - Save images  │
└───────┬────────┘
        │ Dataset files
        ↓
┌────────────────┐
│ AI Training    │
│ Framework      │
│ (PyTorch, TF)  │
└───────┬────────┘
        │ Trained model
        ↓
┌────────────────┐
│ Deploy to      │
│ Real Robot     │
└────────────────┘
```

---

### Collecting Training Data

**ROS 2 Bag Recording**:
```bash
# Conceptual command to record sensor data
ros2 bag record /camera/image_raw /camera/depth/points /scan /imu/data
```

**Result**: `.db3` file containing all sensor messages with timestamps

**Convert to AI framework format**:
- Extract images from bag → PNG files
- Extract labels (bounding boxes, segmentation) → JSON/XML
- Organize into train/val/test splits
- Load into PyTorch DataLoader or TensorFlow Dataset

---

### Sensor Data for Different AI Tasks

**Object Detection**:
- **Input**: RGB images from simulated camera
- **Labels**: Bounding boxes (automatically from simulation)
- **Model**: YOLO, Faster R-CNN
- **Goal**: Detect "person", "chair", "door" in camera view

**Visual SLAM** (from Module 03):
- **Input**: Stereo camera images + IMU
- **Labels**: Ground truth robot poses from Gazebo
- **Model**: ORB-SLAM, LIO-SAM
- **Goal**: Localize robot in environment

**Terrain Classification**:
- **Input**: Point clouds from LiDAR
- **Labels**: Ground segmentation (floor, obstacle, wall)
- **Model**: PointNet, VoxelNet
- **Goal**: Identify safe walking surfaces

---

## Sim-to-Real Transfer

### The Challenge

AI trained purely in simulation often fails on real robots:
- **Visual domain shift**: Simulated images look different from real camera
- **Physics mismatch**: Real dynamics differ slightly from simulation
- **Sensor characteristics**: Real sensors have unique noise patterns

### Techniques to Improve Transfer

**1. Add Realistic Noise**
- Match noise parameters to real sensor datasheets
- Include outliers, occlusions, bias drift

**2. Domain Randomization** (from Module 03)
- Randomize lighting, textures, object positions
- Train on diverse simulated scenarios
- Forces AI to learn general features, not specific scenes

**3. Sim-to-Real Fine-Tuning**
- Pre-train on millions of simulated examples
- Fine-tune on small dataset of real examples (100-1000 images)
- Combines simulation's scale with real data's accuracy

**4. Sim-to-Real Validation**
- Always test on real robot after simulation training
- Measure performance degradation (sim: 95% accuracy, real: 85% acceptable)
- Iterate if gap is too large

---

## Chapter Summary

✅ **Simulated sensors** (LiDAR, depth, RGB, IMU) generate unlimited training data
✅ **Realistic noise** models improve AI robustness and sim-to-real transfer
✅ **ROS 2 topics** deliver sensor data from simulation to AI training frameworks
✅ **Data collection** in simulation is 100-1000x faster than real-world collection
✅ **Sim-to-real gap** is reduced through noise modeling, domain randomization, and fine-tuning

---

## Module Complete!

Congratulations! You've completed Module 02 (Digital Twin). You now understand:
- What digital twins are and their benefits for robotics
- How Gazebo simulates physics for robot validation
- How Unity provides visual realism for HRI and demos
- How to simulate sensors for AI training data generation

**Next Steps**: Apply these concepts in Module 03 (Isaac) to learn about NVIDIA's advanced simulation and perception tools!

**Key Concepts to Remember**:
- Simulated sensors = unlimited training data for AI
- Add noise = improve real-world transfer
- ROS 2 = common interface for simulation and real robots
- Simulation-first = safe, fast, cost-effective development
