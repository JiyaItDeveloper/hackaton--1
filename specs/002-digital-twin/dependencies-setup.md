# Digital Twin Simulation Stack - Dependencies and Setup Requirements

## System Requirements

### Hardware Requirements
- **CPU**: Multi-core processor (Intel i7 or equivalent AMD)
- **RAM**: 16GB minimum, 32GB recommended
- **GPU**: Dedicated graphics card with OpenGL 3.3+ support (NVIDIA/AMD)
- **Storage**: 20GB free space for all components
- **Network**: Stable network connection for Unity-ROS communication

### Software Requirements
- **Operating System**: Ubuntu 22.04 LTS (recommended) or Windows 10/11 with WSL2
- **Docker**: Version 20.10+ (optional, for containerized setup)
- **Git**: Version control for repository management

## ROS 2 Environment Setup

### ROS 2 Distribution
- **Distribution**: ROS 2 Humble Hawksbill (LTS)
- **Installation Method**: Debian packages via apt
- **Required Components**:
  - ros-humble-desktop-full
  - ros-humble-gazebo-ros-pkgs
  - ros-humble-gazebo-ros2-control
  - ros-humble-ros2-control
  - ros-humble-ros2-controllers
  - ros-humble-joint-state-publisher
  - ros-humble-robot-state-publisher
  - ros-humble-xacro

### Installation Commands
```bash
# Add ROS 2 repository and setup keys
sudo apt update && sudo apt install -y curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -
echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list

# Install ROS 2 Humble
sudo apt update
sudo apt install ros-humble-desktop-full

# Install Gazebo integration packages
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers

# Install additional ROS 2 packages
sudo apt install ros-humble-joint-state-publisher ros-humble-robot-state-publisher ros-humble-xacro

# Setup environment
source /opt/ros/humble/setup.bash
```

## Gazebo Simulation Setup

### Gazebo Version
- **Version**: Gazebo Fortress (gz-sim) or compatible version
- **Alternative**: Gazebo Classic (gazebo-11) with ROS 2 bridge

### Required Gazebo Packages
- **Physics Engine**: ODE, Bullet, or DART
- **ROS Integration**: gazebo_ros_pkgs
- **Control Interface**: gazebo_ros2_control
- **Sensors**: Various sensor plugins for IMU, cameras, etc.

### Installation Commands
```bash
# Install Gazebo Fortress
sudo apt install ros-humble-gazebo-ros ros-humble-gazebo-plugins ros-humble-gazebo-dev

# Verify installation
gz sim --version
```

## Unity Environment Setup

### Unity Version
- **Version**: Unity 2021.3 LTS or later
- **Module**: Unity Personal (free) or Unity Pro
- **Build Support**: Linux Build Support (if using Ubuntu)

### Unity Packages Required
- **ROS# Package**: Unity Robotics Hub package for ROS communication
- **URDF Importer**: Package for importing URDF models into Unity
- **Mathematics Package**: Unity Mathematics package for optimized math operations

### Installation Process
1. Download Unity Hub from Unity website
2. Install Unity 2021.3 LTS or later through Unity Hub
3. Create new 3D project
4. Import ROS# package via Unity Package Manager or Asset Store
5. Import URDF Importer package
6. Install additional required packages

### ROS TCP Endpoint Setup
```bash
# Clone and install ROS TCP Endpoint
git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git
cd ROS-TCP-Endpoint
pip3 install -e .

# Alternative: Install via pip
pip3 install ros-tcp-endpoint
```

## Project-Specific Dependencies

### Existing Module Dependencies
- **Module 1 (ROS 2)**: Humanoid URDF model and ROS 2 infrastructure
- **URDF Files**: Located in existing ROS 2 module documentation
- **Configuration Files**: Robot state publisher and joint controllers

### Additional Dependencies for Digital Twin
- **Python Libraries**:
  - rospy (ROS 2 Python client)
  - numpy (numerical computations)
  - transforms3d (3D transformations)
  - cv2 (OpenCV for camera simulation)

- **System Libraries**:
  - libgazebo-dev (Gazebo development files)
  - libignition-math6-dev (Ignition math libraries)
  - libbullet-dev (Bullet physics development files)

### Installation Commands for Additional Dependencies
```bash
# Install system dependencies
sudo apt update
sudo apt install -y python3-pip python3-numpy python3-transforms3d libgazebo-dev libignition-math6-dev libbullet-dev

# Install Python packages
pip3 install transforms3d opencv-python
```

## Environment Configuration

### ROS 2 Workspace Setup
```bash
# Create workspace
mkdir -p ~/digital_twin_ws/src
cd ~/digital_twin_ws

# Build workspace
colcon build --packages-select digital_twin_simulation
source install/setup.bash
```

### Environment Variables
Add to `~/.bashrc`:
```bash
# ROS 2 Humble Setup
source /opt/ros/humble/setup.bash

# Gazebo Setup
source /usr/share/gazebo/setup.sh

# Digital Twin Workspace
source ~/digital_twin_ws/install/setup.bash

# Gazebo Models Path
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/digital_twin_ws/src/digital_twin_simulation/models
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:~/digital_twin_ws/src/digital_twin_simulation/worlds
```

## Verification Steps

### ROS 2 Verification
```bash
# Check ROS 2 installation
ros2 topic list
ros2 service list

# Test basic ROS 2 functionality
ros2 run demo_nodes_cpp talker &
ros2 run demo_nodes_py listener
```

### Gazebo Verification
```bash
# Launch Gazebo
gz sim

# Test Gazebo with ROS 2 bridge
ros2 run gazebo_ros spawn_entity.py -entity test_robot -file path/to/robot.urdf
```

### Unity Verification
1. Launch Unity project
2. Run ROS TCP Connector in Unity scene
3. Start ROS TCP Endpoint
4. Verify connection between Unity and ROS 2

## Troubleshooting Common Issues

### ROS 2 Connection Issues
- **Issue**: Cannot connect to ROS master
- **Solution**: Ensure ROS_DOMAIN_ID is consistent across all terminals

### Gazebo Physics Issues
- **Issue**: Robot falls through ground or exhibits unstable behavior
- **Solution**: Check URDF inertial properties and joint limits

### Unity Communication Issues
- **Issue**: Unity cannot connect to ROS TCP Endpoint
- **Solution**: Verify IP addresses and port numbers, check firewall settings

### Performance Issues
- **Issue**: Low frame rates in Unity or Gazebo
- **Solution**: Reduce model complexity, adjust physics parameters, ensure adequate hardware resources

## Containerized Setup (Optional)

### Docker Setup
```bash
# Install Docker
sudo apt install docker.io
sudo usermod -aG docker $USER

# Pull ROS 2 Humble image
docker pull osrf/ros:humble-desktop-full

# Run container with Gazebo support
docker run -it --rm \
  --env="DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --network=host \
  --name digital_twin_container \
  osrf/ros:humble-desktop-full
```

### Docker Compose Setup
Create a `docker-compose.yml` file for coordinated container management including ROS 2, Gazebo, and Unity components.