# Integration Project: Complete Build & Deployment System

## Overview

In this integration project, we'll combine all the concepts learned in previous chapters to create a complete build and deployment system for our digital twin simulation environment. This system will include containerization, CI/CD pipelines, cloud deployment, and comprehensive monitoring.

## System Architecture

Our complete build and deployment system will have the following architecture:

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        Build & Deployment System                        │
├─────────────────────────────────────────────────────────────────────────┤
│  ┌─────────────┐  ┌─────────────────┐  ┌─────────────────────────────┐  │
│  │ Source Code │  │  CI/CD Pipeline │  │    Cloud Infrastructure    │  │
│  │   Control   │  │                 │  │                             │  │
│  │  (GitHub)   │  │  • Build        │  │  • Kubernetes Cluster      │  │
│  │             │  │  • Test         │  │  • Container Registry      │  │
│  │ • Git       │  │  • Package      │  │  • Load Balancer           │  │
│  │ • Branches  │  │  • Deploy       │  │  • Monitoring Stack        │  │
│  └─────────────┘  └─────────────────┘  └─────────────────────────────┘  │
│           │                │                        │                   │
│           │                │                        │                   │
│           └────────────────┼────────────────────────┘                   │
│                            │                                            │
│                    ┌───────────────┐                                    │
│                    │  Container    │                                    │
│                    │  Orchestration│                                    │
│                    │  (Kubernetes) │                                    │
│                    │               │                                    │
│                    │ • ROS Core    │                                    │
│                    │ • Gazebo Sim  │                                    │
│                    │ • Unity Bridge│                                    │
│                    │ • Monitoring  │                                    │
│                    └───────────────┘                                    │
└─────────────────────────────────────────────────────────────────────────┘
```

## Implementation Steps

### Step 1: Complete Docker Configuration

First, let's create a comprehensive Docker configuration for our system:

```dockerfile
# Dockerfile
# Multi-stage build for digital twin simulation
FROM ros:humble-ros-base-jammy AS base

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
    python3-rosdep \
    # Gazebo dependencies
    gazebo \
    libgazebo-dev \
    ros-$ROS_DISTRO-gazebo-ros-pkgs \
    ros-$ROS_DISTRO-gazebo-ros2-control \
    # Graphics and display
    mesa-utils \
    xvfb \
    # Performance tools
    htop \
    iotop \
    sysstat \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
RUN mkdir -p $ROS_WS/src

# Install Python dependencies
RUN pip3 install \
    launch-ros \
    ros2launch \
    ros-tcp-endpoint \
    prometheus-client \
    requests \
    && rm -rf /root/.cache/pip

# Copy package files for dependency resolution
COPY package.xml CMakeLists.txt $ROS_WS/

# Install ROS dependencies
RUN cd $ROS_WS && \
    apt-get update && \
    rosdep install --from-paths . --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

# Build stage
FROM base AS builder

COPY . $ROS_WS/src/

RUN cd $ROS_WS && \
    colcon build \
    --packages-select digital_twin_simulation \
    --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    --cmake-args -DCMAKE_CXX_FLAGS_RELEASE="-O3 -DNDEBUG"

# Runtime stage
FROM base AS runtime

# Copy built artifacts
COPY --from=builder $ROS_WS/install $ROS_WS/install

# Create non-root user
RUN groupadd -r rosuser && useradd -r -g rosuser rosuser
USER rosuser

# Source ROS environment
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/rosuser/.bashrc
RUN echo "source $ROS_WS/install/setup.bash" >> /home/rosuser/.bashrc

# Set working directory
WORKDIR $ROS_WS

# Create directories for runtime
RUN mkdir -p /home/rosuser/.gazebo/models

# Default command
CMD ["bash"]
```

### Step 2: Create Complete ROS 2 Package

Create the package.xml file:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>digital_twin_simulation</name>
  <version>1.0.0</version>
  <description>Complete digital twin simulation system with monitoring</description>
  <maintainer email="maintainer@example.com">Maintainer Name</maintainer>
  <license>Apache License 2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>trajectory_msgs</depend>
  <depend>builtin_interfaces</depend>
  <depend>ros2_control</depend>
  <depend>gazebo_ros2_control</depend>
  <depend>gazebo_dev</depend>
  <depend>launch</depend>
  <depend>launch_ros</depend>

  <exec_depend>ros2launch</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>joint_state_publisher</exec_depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

Create the CMakeLists.txt file:

```cmake
cmake_minimum_required(VERSION 3.8)
project(digital_twin_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(ros2_control REQUIRED)
find_package(gazebo_ros2_control REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(launch REQUIRED)
find_package(launch_ros REQUIRED)

# Add synchronization node
add_executable(synchronization_node src/synchronization_node.cpp)
ament_target_dependencies(synchronization_node
  rclcpp
  std_msgs
  sensor_msgs
  builtin_interfaces
)

# Add monitoring node
add_executable(monitoring_node src/monitoring_node.cpp)
ament_target_dependencies(monitoring_node
  rclcpp
  std_msgs
  sensor_msgs
)

# Install targets
install(TARGETS
  synchronization_node
  monitoring_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  worlds
  models
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  scripts/start_simulation.sh
  scripts/health_check.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

### Step 3: Create Launch Files

Create a comprehensive launch file:

```python
# launch/digital_twin_complete.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    robot_description_path = LaunchConfiguration('robot_description_path',
        default=PathJoinSubstitution([
            FindPackageShare('digital_twin_simulation'),
            'models',
            'digital_twin_humanoid.urdf.xacro'
        ]))

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ),
        launch_arguments={
            'world': PathJoinSubstitution([
                FindPackageShare('digital_twin_simulation'),
                'worlds',
                'digital_twin_world.sdf'
            ])
        }.items()
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': PathJoinSubstitution([
                FindPackageShare('digital_twin_simulation'),
                'models',
                'digital_twin_humanoid.urdf.xacro'
            ])}
        ]
    )

    # Joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Controller manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('digital_twin_simulation'),
                'config',
                'digital_twin_controllers.yaml'
            ]),
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # Joint trajectory controller spawner
    joint_trajectory_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['humanoid_joint_trajectory_controller'],
        output='screen'
    )

    # Synchronization node
    sync_node = Node(
        package='digital_twin_simulation',
        executable='synchronization_node',
        name='synchronization_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Monitoring node
    monitoring_node = Node(
        package='digital_twin_simulation',
        executable='monitoring_node',
        name='monitoring_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Unity bridge (if needed)
    unity_bridge = Node(
        package='digital_twin_simulation',
        executable='unity_bridge_node',
        name='unity_bridge',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'robot_description_path',
            default_value=PathJoinSubstitution([
                FindPackageShare('digital_twin_simulation'),
                'models',
                'digital_twin_humanoid.urdf.xacro'
            ]),
            description='Path to robot description file'
        ),
        gazebo,
        robot_state_publisher,
        joint_state_publisher,
        controller_manager,
        joint_trajectory_spawner,
        sync_node,
        monitoring_node,
        unity_bridge
    ])
```

### Step 4: Create CI/CD Pipeline Configuration

Create a comprehensive GitHub Actions workflow:

```yaml
# .github/workflows/complete_pipeline.yml
name: Complete Build & Deployment Pipeline

on:
  push:
    branches: [ main, develop ]
  pull_request:
    branches: [ main ]
  release:
    types: [ published ]

env:
  REGISTRY: ghcr.io
  IMAGE_NAME: ${{ github.repository }}

jobs:
  build-and-test:
    runs-on: ubuntu-22.04
    container:
      image: ros:humble-ros-base-jammy

    strategy:
      matrix:
        include:
          - arch: amd64
            platform: linux/amd64

    steps:
    - uses: actions/checkout@v4

    - name: Setup Environment
      run: |
        echo "source /opt/ros/humble/setup.bash" >> $GITHUB_ENV
        echo "source /opt/ros_ws/install/setup.bash" >> $GITHUB_ENV

    - name: Install System Dependencies
      run: |
        apt-get update
        apt-get install -y \
          python3-colcon-common-extensions \
          python3-rosdep \
          ros-humble-gazebo-ros-pkgs \
          ros-humble-ros2-control \
          ros-humble-ros2-controllers \
          && rm -rf /var/lib/apt/lists/*

    - name: Create Workspace
      run: |
        mkdir -p /opt/ros_ws/src
        cp -r . /opt/ros_ws/src/digital_twin_simulation

    - name: Install ROS Dependencies
      run: |
        cd /opt/ros_ws
        rosdep install --from-paths src --ignore-src -r -y

    - name: Build Package
      run: |
        cd /opt/ros_ws
        colcon build \
          --packages-select digital_twin_simulation \
          --cmake-args -DCMAKE_BUILD_TYPE=Release

    - name: Run Unit Tests
      run: |
        cd /opt/ros_ws
        colcon test --packages-select digital_twin_simulation
        colcon test-result --all --verbose

    - name: Generate Code Coverage
      run: |
        cd /opt/ros_ws
        pip3 install gcovr
        gcovr --root=src/digital_twin_simulation --xml --output=coverage.xml

    - name: Upload Coverage to Codecov
      uses: codecov/codecov-action@v3
      with:
        file: /opt/ros_ws/coverage.xml
        fail_ci_if_error: true

    - name: Run Integration Tests
      run: |
        cd /opt/ros_ws
        source install/setup.bash
        # Run integration tests with simulation
        timeout 300 ros2 launch digital_twin_simulation integration_test.launch.py

    - name: Upload Artifacts
      uses: actions/upload-artifact@v3
      with:
        name: build-artifacts-${{ matrix.arch }}
        path: |
          /opt/ros_ws/install/
          /opt/ros_ws/test_results/
          /opt/ros_ws/coverage.xml

  security-scan:
    runs-on: ubuntu-22.04
    needs: build-and-test
    steps:
    - uses: actions/checkout@v4

    - name: Run Security Scanning
      uses: aquasecurity/trivy-action@master
      with:
        scan-type: 'fs'
        scan-ref: '.'
        format: 'sarif'
        output: 'trivy-results.sarif'

    - name: Upload Trivy scan results to GitHub Security tab
      uses: github/codeql-action/upload-sarif@v2
      if: always()
      with:
        sarif_file: 'trivy-results.sarif'

  docker-build:
    runs-on: ubuntu-22.04
    needs: security-scan
    strategy:
      matrix:
        include:
          - arch: amd64
            platform: linux/amd64

    steps:
    - uses: actions/checkout@v4

    - name: Set up Docker Buildx
      uses: docker/setup-buildx-action@v2

    - name: Log in to Container Registry
      uses: docker/login-action@v2
      with:
        registry: ${{ env.REGISTRY }}
        username: ${{ github.actor }}
        password: ${{ secrets.GITHUB_TOKEN }}

    - name: Extract metadata
      id: meta
      uses: docker/metadata-action@v4
      with:
        images: ${{ env.REGISTRY }}/${{ env.IMAGE_NAME }}
        tags: |
          type=ref,event=branch
          type=ref,event=pr
          type=sha,prefix={{branch}}-
          type=raw,value=latest,enable={{is_default_branch}}

    - name: Build and push Docker image
      uses: docker/build-push-action@v4
      with:
        context: .
        file: Dockerfile
        platforms: ${{ matrix.platform }}
        push: ${{ github.event_name != 'pull_request' }}
        tags: ${{ steps.meta.outputs.tags }}
        labels: ${{ steps.meta.outputs.labels }}
        cache-from: type=gha
        cache-to: type=gha,mode=max

  deploy-dev:
    runs-on: ubuntu-22.04
    needs: docker-build
    if: github.ref == 'refs/heads/develop'
    environment: development

    steps:
    - name: Deploy to Development
      run: |
        echo "Deploying to development environment..."
        # Add your deployment commands here
        # This could include kubectl apply, terraform apply, etc.

  deploy-prod:
    runs-on: ubuntu-22.04
    needs: docker-build
    if: github.event_name == 'release' && github.event.action == 'published'
    environment: production

    steps:
    - name: Deploy to Production
      run: |
        echo "Deploying to production environment..."
        # Add your production deployment commands here
        # This could include kubectl apply, terraform apply, etc.

  post-deployment-test:
    runs-on: ubuntu-22.04
    needs: [deploy-dev, deploy-prod]
    steps:
    - name: Run Post-Deployment Tests
      run: |
        echo "Running post-deployment tests..."
        # Add health checks and validation tests
```

### Step 5: Create Kubernetes Deployment Configuration

Create comprehensive Kubernetes manifests:

```yaml
# k8s/digital-twin-full.yaml
apiVersion: v1
kind: Namespace
metadata:
  name: digital-twin-system
---
apiVersion: v1
kind: ConfigMap
metadata:
  name: digital-twin-config
  namespace: digital-twin-system
data:
  ros-domain-id: "42"
  gazebo-update-rate: "1000"
  simulation-world: "digital_twin_world.sdf"
  use-sim-time: "true"
---
apiVersion: v1
kind: ServiceAccount
metadata:
  name: digital-twin-sa
  namespace: digital-twin-system
---
apiVersion: rbac.authorization.k8s.io/v1
kind: Role
metadata:
  namespace: digital-twin-system
  name: digital-twin-role
rules:
- apiGroups: [""]
  resources: ["pods", "services", "configmaps"]
  verbs: ["get", "list", "watch", "create", "update", "patch", "delete"]
---
apiVersion: rbac.authorization.k8s.io/v1
kind: RoleBinding
metadata:
  name: digital-twin-rolebinding
  namespace: digital-twin-system
subjects:
- kind: ServiceAccount
  name: digital-twin-sa
  namespace: digital-twin-system
roleRef:
  kind: Role
  name: digital-twin-role
  apiGroup: rbac.authorization.k8s.io
---
apiVersion: apps/v1
kind: Deployment
metadata:
  name: digital-twin-core
  namespace: digital-twin-system
  labels:
    app: digital-twin-core
spec:
  replicas: 1
  selector:
    matchLabels:
      app: digital-twin-core
  template:
    metadata:
      labels:
        app: digital-twin-core
    spec:
      serviceAccountName: digital-twin-sa
      containers:
      - name: ros-core
        image: ghcr.io/your-username/digital-twin-sim:latest
        command: ["bash", "-c"]
        args:
          - |
            source /opt/ros/humble/setup.bash
            source /opt/ros_ws/install/setup.bash
            export ROS_DOMAIN_ID=$(ROS_DOMAIN_ID)
            export GAZEBO_UPDATE_RATE=$(GAZEBO_UPDATE_RATE)
            ros2 daemon start
            ros2 run digital_twin_simulation synchronization_node
        ports:
        - containerPort: 11311
          name: ros-master
        env:
        - name: ROS_DOMAIN_ID
          valueFrom:
            configMapKeyRef:
              name: digital-twin-config
              key: ros-domain-id
        - name: GAZEBO_UPDATE_RATE
          valueFrom:
            configMapKeyRef:
              name: digital-twin-config
              key: gazebo-update-rate
        - name: USE_SIM_TIME
          valueFrom:
            configMapKeyRef:
              name: digital-twin-config
              key: use-sim-time
        resources:
          requests:
            memory: "2Gi"
            cpu: "1000m"
          limits:
            memory: "4Gi"
            cpu: "2000m"
        volumeMounts:
        - name: ros-workspace
          mountPath: /opt/ros_ws
        - name: shared-memory
          mountPath: /dev/shm
        - name: config-volume
          mountPath: /etc/digital-twin
        livenessProbe:
          exec:
            command: ["ros2", "node", "list"]
          initialDelaySeconds: 30
          periodSeconds: 10
        readinessProbe:
          exec:
            command: ["ros2", "node", "info", "synchronization_node"]
          initialDelaySeconds: 10
          periodSeconds: 5
      volumes:
      - name: ros-workspace
        emptyDir: {}
      - name: shared-memory
        emptyDir:
          medium: Memory
          sizeLimit: "1Gi"
      - name: config-volume
        configMap:
          name: digital-twin-config
---
apiVersion: apps/v1
kind: Deployment
metadata:
  name: digital-twin-simulation
  namespace: digital-twin-system
  labels:
    app: digital-twin-sim
spec:
  replicas: 1
  selector:
    matchLabels:
      app: digital-twin-sim
  template:
    metadata:
      labels:
        app: digital-twin-sim
    spec:
      serviceAccountName: digital-twin-sa
      containers:
      - name: gazebo-sim
        image: ghcr.io/your-username/digital-twin-sim:latest
        command: ["bash", "-c"]
        args:
          - |
            source /opt/ros/humble/setup.bash
            source /opt/ros_ws/install/setup.bash
            export GAZEBO_UPDATE_RATE=$(GAZEBO_UPDATE_RATE)
            export GAZEBO_WORLD_FILE=$(GAZEBO_WORLD_FILE)
            gz sim -v 4
        ports:
        - containerPort: 11345
          name: gz-server
        env:
        - name: GAZEBO_UPDATE_RATE
          valueFrom:
            configMapKeyRef:
              name: digital-twin-config
              key: gazebo-update-rate
        - name: GAZEBO_WORLD_FILE
          valueFrom:
            configMapKeyRef:
              name: digital-twin-config
              key: simulation-world
        - name: DISPLAY
          value: ":0"
        resources:
          requests:
            memory: "4Gi"
            cpu: "2000m"
            nvidia.com/gpu: 1
          limits:
            memory: "8Gi"
            cpu: "4000m"
            nvidia.com/gpu: 1
        volumeMounts:
        - name: shared-memory
          mountPath: /dev/shm
        securityContext:
          privileged: true
      volumes:
      - name: shared-memory
        emptyDir:
          medium: Memory
          sizeLimit: "2Gi"
---
apiVersion: v1
kind: Service
metadata:
  name: digital-twin-service
  namespace: digital-twin-system
spec:
  selector:
    app: digital-twin-core
  ports:
  - protocol: TCP
    port: 11311
    targetPort: 11311
    name: ros-master
  type: LoadBalancer
---
apiVersion: v1
kind: Service
metadata:
  name: gazebo-service
  namespace: digital-twin-system
spec:
  selector:
    app: digital-twin-sim
  ports:
  - protocol: TCP
    port: 11345
    targetPort: 11345
    name: gz-server
  type: ClusterIP
---
apiVersion: networking.k8s.io/v1
kind: NetworkPolicy
metadata:
  name: digital-twin-network-policy
  namespace: digital-twin-system
spec:
  podSelector:
    matchLabels:
      app: digital-twin-core
  policyTypes:
  - Ingress
  - Egress
  ingress:
  - from:
    - podSelector:
        matchLabels:
          app: digital-twin-sim
    ports:
    - protocol: TCP
      port: 11311
  egress:
  - to: []
    ports:
    - protocol: TCP
      port: 53
    - protocol: TCP
      port: 11311
    - protocol: TCP
      port: 11345
---
apiVersion: autoscaling/v2
kind: HorizontalPodAutoscaler
metadata:
  name: digital-twin-hpa
  namespace: digital-twin-system
spec:
  scaleTargetRef:
    apiVersion: apps/v1
    kind: Deployment
    name: digital-twin-core
  minReplicas: 1
  maxReplicas: 3
  metrics:
  - type: Resource
    resource:
      name: cpu
      target:
        type: Utilization
        averageUtilization: 70
  - type: Resource
    resource:
      name: memory
      target:
        type: Utilization
        averageUtilization: 80
```

### Step 6: Create Monitoring Configuration

Create comprehensive monitoring setup:

```yaml
# monitoring/monitoring-stack.yaml
apiVersion: v1
kind: Namespace
metadata:
  name: monitoring
---
apiVersion: v1
kind: ConfigMap
metadata:
  name: prometheus-config
  namespace: monitoring
data:
  prometheus.yml: |
    global:
      scrape_interval: 15s
      evaluation_interval: 15s
    rule_files:
      - "digital_twin_rules.yml"
    scrape_configs:
    - job_name: 'kubernetes-pods'
      kubernetes_sd_configs:
      - role: pod
        namespaces:
          names:
          - digital-twin-system
      relabel_configs:
      - source_labels: [__meta_kubernetes_pod_annotation_prometheus_io_scrape]
        action: keep
        regex: true
      - source_labels: [__meta_kubernetes_pod_annotation_prometheus_io_path]
        action: replace
        target_label: __metrics_path__
        regex: (.+)
      - source_labels: [__address__, __meta_kubernetes_pod_annotation_prometheus_io_port]
        action: replace
        regex: ([^:]+)(?::\d+)?;(\d+)
        replacement: $1:$2
        target_label: __address__
      - source_labels: [__meta_kubernetes_pod_name]
        target_label: pod
    - job_name: 'node-exporter'
      kubernetes_sd_configs:
      - role: endpoints
      relabel_configs:
      - source_labels: [__meta_kubernetes_endpoints_name]
        regex: 'node-exporter'
        action: keep
---
apiVersion: v1
kind: ConfigMap
metadata:
  name: prometheus-rules
  namespace: monitoring
data:
  digital_twin_rules.yml: |
    groups:
    - name: digital_twin.rules
      rules:
      - alert: DigitalTwinDown
        expr: up{job="digital-twin"} == 0
        for: 5m
        labels:
          severity: critical
        annotations:
          summary: "Digital Twin is down"
          description: "Digital Twin has been down for more than 5 minutes."
      - alert: HighCPUUsage
        expr: 100 - (avg by(instance) (rate(node_cpu_seconds_total{mode="idle"}[5m])) * 100) > 80
        for: 2m
        labels:
          severity: warning
        annotations:
          summary: "High CPU usage on node"
          description: "CPU usage is above 80%: {{ $value }}%"
---
apiVersion: apps/v1
kind: Deployment
metadata:
  name: prometheus
  namespace: monitoring
spec:
  replicas: 1
  selector:
    matchLabels:
      app: prometheus
  template:
    metadata:
      labels:
        app: prometheus
    spec:
      containers:
      - name: prometheus
        image: prom/prometheus:latest
        args:
          - '--config.file=/etc/prometheus/prometheus.yml'
          - '--storage.tsdb.path=/prometheus/'
          - '--web.console.libraries=/etc/prometheus/console_libraries'
          - '--web.console.templates=/etc/prometheus/consoles'
          - '--storage.tsdb.retention.time=200h'
          - '--web.enable-lifecycle'
        ports:
        - containerPort: 9090
        volumeMounts:
        - name: prometheus-config-volume
          mountPath: /etc/prometheus/
        - name: prometheus-storage-volume
          mountPath: /prometheus/
      volumes:
      - name: prometheus-config-volume
        configMap:
          defaultMode: 420
          name: prometheus-config
      - name: prometheus-storage-volume
        emptyDir: {}
---
apiVersion: v1
kind: Service
metadata:
  name: prometheus-service
  namespace: monitoring
spec:
  selector:
    app: prometheus
  ports:
  - protocol: TCP
    port: 9090
    targetPort: 9090
  type: LoadBalancer
---
apiVersion: apps/v1
kind: Deployment
metadata:
  name: grafana
  namespace: monitoring
spec:
  replicas: 1
  selector:
    matchLabels:
      app: grafana
  template:
    metadata:
      labels:
        app: grafana
    spec:
      containers:
      - name: grafana
        image: grafana/grafana:latest
        ports:
        - containerPort: 3000
        env:
        - name: GF_SECURITY_ADMIN_PASSWORD
          value: "admin123"
        - name: GF_USERS_ALLOW_SIGN_UP
          value: "false"
        volumeMounts:
        - name: grafana-storage
          mountPath: /var/lib/grafana
      volumes:
      - name: grafana-storage
        emptyDir: {}
---
apiVersion: v1
kind: Service
metadata:
  name: grafana-service
  namespace: monitoring
spec:
  selector:
    app: grafana
  ports:
  - protocol: TCP
    port: 3000
    targetPort: 3000
  type: LoadBalancer
```

### Step 7: Create Deployment Scripts

Create a deployment script:

```bash
#!/bin/bash
# scripts/deploy.sh

set -e

echo "Starting Digital Twin System Deployment..."

# Check if kubectl is available
if ! command -v kubectl &> /dev/null; then
    echo "kubectl is required but not installed. Aborting."
    exit 1
fi

# Check if Docker image exists
IMAGE_NAME="ghcr.io/your-username/digital-twin-sim:latest"
if ! docker manifest inspect $IMAGE_NAME &> /dev/null; then
    echo "Docker image $IMAGE_NAME does not exist. Please build and push first."
    exit 1
fi

echo "Creating namespace..."
kubectl create namespace digital-twin-system --dry-run=client -o yaml | kubectl apply -f -

echo "Applying configuration..."
kubectl apply -f k8s/digital-twin-full.yaml

echo "Waiting for deployments to be ready..."
kubectl wait --for=condition=ready pod -l app=digital-twin-core -n digital-twin-system --timeout=300s
kubectl wait --for=condition=ready pod -l app=digital-twin-sim -n digital-twin-system --timeout=300s

echo "Applying monitoring stack..."
kubectl create namespace monitoring --dry-run=client -o yaml | kubectl apply -f -
kubectl apply -f monitoring/monitoring-stack.yaml

echo "Deployment completed successfully!"
echo "Services are now available:"
echo "  - ROS Core: $(kubectl get service digital-twin-service -n digital-twin-system -o jsonpath='{.status.loadBalancer.ingress[0].ip}'):11311"
echo "  - Prometheus: $(kubectl get service prometheus-service -n monitoring -o jsonpath='{.status.loadBalancer.ingress[0].ip}'):9090"
echo "  - Grafana: $(kubectl get service grafana-service -n monitoring -o jsonpath='{.status.loadBalancer.ingress[0].ip}'):3000"

# Verify deployment
echo "Verifying deployment..."
kubectl get pods -n digital-twin-system
kubectl get services -n digital-twin-system
kubectl get pods -n monitoring
kubectl get services -n monitoring

echo "Digital Twin System deployment complete!"
```

### Step 8: Create Health Check Script

Create a health monitoring script:

```python
#!/usr/bin/env python3
# scripts/health_check.py

import subprocess
import json
import time
import requests
from datetime import datetime

def check_kubernetes_health():
    """Check Kubernetes cluster health"""
    try:
        # Check nodes status
        result = subprocess.run(['kubectl', 'get', 'nodes', '-o', 'json'],
                              capture_output=True, text=True)
        if result.returncode != 0:
            return False, f"Kubectl command failed: {result.stderr}"

        nodes = json.loads(result.stdout)
        for node in nodes['items']:
            for condition in node['status']['conditions']:
                if condition['type'] == 'Ready' and condition['status'] != 'True':
                    return False, f"Node {node['metadata']['name']} is not ready"

        return True, "All nodes are ready"
    except Exception as e:
        return False, f"Error checking Kubernetes health: {str(e)}"

def check_digital_twin_health():
    """Check digital twin system health"""
    try:
        # Check if pods are running
        result = subprocess.run(['kubectl', 'get', 'pods', '-n', 'digital-twin-system', '-o', 'json'],
                              capture_output=True, text=True)
        if result.returncode != 0:
            return False, f"Failed to get pods: {result.stderr}"

        pods = json.loads(result.stdout)
        for pod in pods['items']:
            status = pod['status']['phase']
            if status not in ['Running', 'Succeeded']:
                return False, f"Pod {pod['metadata']['name']} is in {status} state"

        return True, "All pods are running"
    except Exception as e:
        return False, f"Error checking digital twin health: {str(e)}"

def check_monitoring_health():
    """Check monitoring system health"""
    try:
        # Check if monitoring services are accessible
        prometheus_svc = subprocess.run(['kubectl', 'get', 'service', 'prometheus-service', '-n', 'monitoring', '-o', 'json'],
                                      capture_output=True, text=True)
        if prometheus_svc.returncode != 0:
            return False, "Prometheus service not found"

        grafana_svc = subprocess.run(['kubectl', 'get', 'service', 'grafana-service', '-n', 'monitoring', '-o', 'json'],
                                   capture_output=True, text=True)
        if grafana_svc.returncode != 0:
            return False, "Grafana service not found"

        return True, "Monitoring services are available"
    except Exception as e:
        return False, f"Error checking monitoring health: {str(e)}"

def run_health_checks():
    """Run comprehensive health checks"""
    print(f"Running health checks at {datetime.now()}")

    checks = [
        ("Kubernetes Cluster", check_kubernetes_health),
        ("Digital Twin System", check_digital_twin_health),
        ("Monitoring System", check_monitoring_health)
    ]

    all_healthy = True

    for check_name, check_function in checks:
        is_healthy, message = check_function()
        status = "✓" if is_healthy else "✗"
        print(f"{status} {check_name}: {message}")

        if not is_healthy:
            all_healthy = False

    if all_healthy:
        print(f"\n✓ All systems are healthy!")
        return 0
    else:
        print(f"\n✗ Some systems are not healthy!")
        return 1

if __name__ == "__main__":
    exit_code = run_health_checks()
    exit(exit_code)
```

## Testing the Complete System

### Step 1: Local Testing

```bash
# Build and test locally first
cd ~/digital_twin_ws
colcon build --packages-select digital_twin_simulation
source install/setup.bash
ros2 launch digital_twin_simulation digital_twin_complete.launch.py
```

### Step 2: Docker Testing

```bash
# Build and test Docker image
docker build -t digital-twin-test .
docker run -it --rm digital-twin-test bash
```

### Step 3: Kubernetes Testing

```bash
# Test deployment locally with kind or minikube
kind create cluster
kubectl apply -f k8s/digital-twin-full.yaml
kubectl get pods -n digital-twin-system
```

## Performance Benchmarks

### Expected Performance Metrics

- **Container Build Time**: < 10 minutes for optimized builds
- **Deployment Time**: < 5 minutes for complete system
- **System Startup**: < 2 minutes from deployment to ready
- **Resource Usage**:
  - CPU: < 70% under normal load
  - Memory: < 80% of allocated resources
  - GPU: < 85% utilization for simulation
- **Response Time**: < 50ms for ROS service calls
- **Throughput**: > 1000 messages/second for joint states

## Troubleshooting

### Common Deployment Issues

1. **Resource Constraints**
   - Ensure sufficient CPU, memory, and GPU resources
   - Check node resource availability
   - Adjust resource requests and limits

2. **Network Connectivity**
   - Verify service discovery between components
   - Check network policies
   - Ensure proper load balancer configuration

3. **Image Pull Issues**
   - Verify container registry access
   - Check image pull secrets
   - Ensure image exists and is accessible

4. **Permission Issues**
   - Verify RBAC configuration
   - Check service account permissions
   - Ensure proper security contexts

## Maintenance and Operations

### Daily Operations

1. **Health Monitoring**: Run health checks regularly
2. **Log Analysis**: Monitor system logs for issues
3. **Resource Monitoring**: Track resource usage trends
4. **Backup Verification**: Ensure backups are running

### Weekly Tasks

1. **Performance Review**: Analyze performance metrics
2. **Security Updates**: Apply security patches
3. **Configuration Review**: Update configurations as needed
4. **Capacity Planning**: Plan for resource scaling

### Monthly Tasks

1. **System Audit**: Review system configuration and access
2. **Backup Testing**: Test backup and recovery procedures
3. **Documentation Update**: Update operational documentation
4. **Performance Optimization**: Optimize system performance

## Summary

In this integration project, we've successfully created a complete build and deployment system that:

1. **Containerizes** the digital twin simulation with optimized Docker builds
2. **Implements CI/CD pipelines** with comprehensive testing and security scanning
3. **Deploys to Kubernetes** with proper resource management and networking
4. **Provides monitoring** with Prometheus and Grafana dashboards
5. **Includes health checks** and operational procedures
6. **Follows security best practices** with proper RBAC and network policies

This system provides a production-ready platform for deploying and operating digital twin simulation environments, with the ability to scale, monitor, and maintain the system effectively. The modular architecture allows for easy extension with additional components and services as needed for specific robotics applications.