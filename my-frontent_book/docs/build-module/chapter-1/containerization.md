# Chapter 1: Containerization & Docker for Robotics

## Introduction

Containerization is a crucial technology for robotics development that enables consistent, reproducible environments across different platforms. In this chapter, we'll explore how to containerize robotics applications using Docker, focusing on ROS 2 and simulation environments.

## Why Containerization for Robotics?

Robotics applications face unique challenges that make containerization particularly valuable:

- **Complex Dependencies**: Multiple libraries, drivers, and middleware components
- **Environment Consistency**: Ensuring identical behavior across development, testing, and deployment
- **Multi-platform Support**: Running on various hardware configurations
- **Version Management**: Managing different versions of ROS, Gazebo, and other tools
- **Isolation**: Preventing conflicts between different projects and their dependencies

## Docker Fundamentals for Robotics

### Basic Docker Concepts

- **Images**: Templates that contain the application and all its dependencies
- **Containers**: Running instances of images
- **Dockerfile**: Instructions for building an image
- **Docker Compose**: Configuration for multi-container applications

### Dockerfile Best Practices for Robotics

```dockerfile
# Use official ROS 2 image as base
FROM ros:humble-ros-base-jammy

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV ROS_WS=/opt/ros_ws

# Install system dependencies
RUN apt-get update && apt-get install -y \
    git \
    curl \
    wget \
    build-essential \
    python3-pip \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
RUN mkdir -p $ROS_WS/src

# Copy package.xml and CMakeLists.txt first for better caching
COPY package.xml CMakeLists.txt $ROS_WS/

# Install ROS dependencies
RUN cd $ROS_WS && \
    apt-get update && \
    rosdep install --from-paths . --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

# Copy source code
COPY . $ROS_WS/src/

# Build the workspace
RUN cd $ROS_WS && \
    colcon build --packages-select digital_twin_simulation --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source ROS environment
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc
RUN echo "source $ROS_WS/install/setup.bash" >> /root/.bashrc

# Set working directory
WORKDIR $ROS_WS

# Default command
CMD ["bash"]
```

## ROS 2 Specific Docker Patterns

### Multi-stage Builds

```dockerfile
# Build stage
FROM ros:humble-ros-base-jammy AS builder

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_WS=/opt/ros_ws

RUN mkdir -p $ROS_WS/src
COPY . $ROS_WS/src/

RUN cd $ROS_WS && \
    apt-get update && \
    rosdep install --from-paths . --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/* && \
    colcon build --packages-select digital_twin_simulation

# Runtime stage
FROM ros:humble-ros-base-jammy

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_WS=/opt/ros_ws

# Install only runtime dependencies
RUN apt-get update && apt-get install -y \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-gazebo-ros-pkgs \
    && rm -rf /var/lib/apt/lists/*

# Copy built artifacts from build stage
COPY --from=builder $ROS_WS/install $ROS_WS/install

# Source ROS environment
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "source $ROS_WS/install/setup.bash" >> /root/.bashrc

WORKDIR $ROS_WS
CMD ["bash"]
```

### GPU Support for Simulation

```dockerfile
FROM nvidia/ros:humble-desktop-foxy:latest

# Install additional dependencies for Gazebo simulation
RUN apt-get update && apt-get install -y \
    nvidia-opengl-dev \
    mesa-utils \
    xvfb \
    && rm -rf /var/lib/apt/lists/*

# Set environment for GPU acceleration
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute
ENV QT_X11_NO_MITSHM=1

# Rest of Dockerfile...
```

## Docker Compose for Multi-Component Systems

For complex robotics systems with multiple services:

```yaml
# docker-compose.yml
version: '3.8'

services:
  ros-core:
    build: .
    container_name: ros_core
    command: ros2 daemon start && bash
    volumes:
      - ./src:/opt/ros_ws/src
    environment:
      - ROS_DOMAIN_ID=42
      - DISPLAY=${DISPLAY}
    network_mode: host
    privileged: true

  gazebo:
    build: .
    container_name: gazebo_sim
    command: bash -c "source /opt/ros_ws/install/setup.bash && gz sim -v 4"
    environment:
      - DISPLAY=${DISPLAY}
      - QT_X11_NO_MITSHM=1
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
      - ./models:/root/.gazebo/models
    devices:
      - /dev/dri:/dev/dri
    network_mode: host

  unity-bridge:
    build: .
    container_name: unity_bridge
    command: bash -c "source /opt/ros_ws/install/setup.bash && python3 -m ros_tcp_endpoint"
    ports:
      - "10000:10000"
    environment:
      - ROS_DOMAIN_ID=42
    network_mode: host
```

## Building and Running Containerized Robotics Applications

### Building the Image

```bash
# Build with specific target (if using multi-stage)
docker build -t digital-twin-sim:latest .

# Build with build arguments
docker build --build-arg ROS_DISTRO=humble -t digital-twin-sim:latest .

# Build with cache mount for faster builds
docker build --mount=type=cache,target=/var/cache/apt -t digital-twin-sim:latest .
```

### Running the Container

```bash
# Basic run
docker run -it digital-twin-sim:latest

# With GPU support
docker run -it --gpus all digital-twin-sim:latest

# With display support for GUI applications
docker run -it \
  --env="DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --privileged \
  digital-twin-sim:latest

# With network access
docker run -it --network host digital-twin-sim:latest
```

## Optimizing Docker Images for Robotics

### Layer Caching Strategy

```dockerfile
# Copy package files first to leverage layer caching
COPY package.xml CMakeLists.txt $ROS_WS/
RUN cd $ROS_WS && rosdep install --from-paths . --ignore-src -r -y

# Copy source code last (changes most frequently)
COPY . $ROS_WS/src/
```

### Multi-architecture Builds

```bash
# Build for multiple architectures
docker buildx build --platform linux/amd64,linux/arm64 -t digital-twin-sim:latest .

# Create manifest for multi-arch images
docker buildx build --platform linux/amd64,linux/arm64 --push -t digital-twin-sim:latest .
```

## Security Considerations

### User Management

```dockerfile
# Create non-root user
RUN groupadd -r rosuser && useradd -r -g rosuser rosuser
USER rosuser
```

### Image Scanning

```bash
# Scan for vulnerabilities
docker scan digital-twin-sim:latest

# Use security scanning in CI/CD
docker scan --severity critical --only-fixed digital-twin-sim:latest
```

## Performance Optimization

### Memory Management

```dockerfile
# Limit memory usage
docker run -m 4g digital-twin-sim:latest

# Optimize build cache
docker system prune -a --volumes
```

### Volume Management for Development

```bash
# Mount source code for development
docker run -v $(pwd)/src:/opt/ros_ws/src -it digital-twin-sim:latest

# Use named volumes for persistent data
docker volume create ros_workspace
docker run -v ros_workspace:/opt/ros_ws digital-twin-sim:latest
```

## Exercise: Containerize Your Digital Twin System

1. **Create a Dockerfile** for your digital twin simulation system
2. **Build the image** with appropriate caching strategies
3. **Run the container** with proper GUI support
4. **Test the simulation** inside the container
5. **Optimize the image size** using multi-stage builds

## Troubleshooting Common Issues

### Display Issues
- Ensure X11 forwarding is properly configured
- Check DISPLAY environment variable
- Verify GPU drivers are accessible

### Network Issues
- Use host networking for ROS communication
- Check ROS_DOMAIN_ID consistency
- Verify port bindings for Unity communication

### Performance Issues
- Limit container resources appropriately
- Use volume mounts instead of copying large datasets
- Optimize base image size

## Summary

In this chapter, we've covered the fundamentals of containerizing robotics applications:
- Basic Docker concepts and best practices for robotics
- Multi-stage builds for optimized images
- GPU support for simulation environments
- Docker Compose for multi-component systems
- Security and performance considerations

In the next chapter, we'll explore ROS 2 build systems and package management.