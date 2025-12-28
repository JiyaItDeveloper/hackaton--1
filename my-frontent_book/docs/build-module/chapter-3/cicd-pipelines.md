# Chapter 3: CI/CD Pipelines for Robotics

## Introduction

Continuous Integration and Continuous Deployment (CI/CD) are essential practices for robotics development. In this chapter, we'll explore how to implement robust CI/CD pipelines for robotics projects, focusing on automated testing, building, and deployment of our digital twin simulation system.

## CI/CD Fundamentals for Robotics

### Why CI/CD is Critical for Robotics

Robotics projects have unique requirements that make CI/CD especially important:

- **Complex Dependencies**: Multiple software components, hardware drivers, and middleware
- **Safety Requirements**: Need for rigorous testing and validation
- **Hardware Integration**: Ensuring software works with physical systems
- **Multi-platform Support**: Different robots, operating systems, and architectures
- **Rapid Iteration**: Fast development cycles for algorithm improvement

### CI/CD Pipeline Components

```
Source Code Repository
         ↓
Trigger (Push/Pull Request)
         ↓
Build & Compile
         ↓
Unit Tests
         ↓
Integration Tests
         ↓
System Tests
         ↓
Package & Deploy
         ↓
Monitoring & Feedback
```

## GitHub Actions for Robotics CI/CD

### Basic GitHub Actions Workflow

```yaml
# .github/workflows/robotics_ci.yml
name: Robotics CI/CD Pipeline

on:
  push:
    branches: [ main, develop ]
  pull_request:
    branches: [ main ]

jobs:
  build-and-test:
    runs-on: ubuntu-22.04

    strategy:
      matrix:
        ros-distro: [humble]

    steps:
    - uses: actions/checkout@v3

    - name: Setup ROS 2
      uses: ros-tooling/setup-ros@0.7.3
      with:
        required-ros-distributions: ${{ matrix.ros-distro }}

    - name: Install Dependencies
      run: |
        sudo apt-get update
        rosdep update
        rosdep install --from-paths . --ignore-src -r -y

    - name: Build Workspace
      uses: ros-tooling/action-ros-ci@v0.3
      with:
        package-name: digital_twin_simulation
        target-ros2-distro: ${{ matrix.ros-distro }}
        colcon-defaults: |
          {
            "build": {
              "cmake-args": ["-DCMAKE_BUILD_TYPE=Release"]
            }
          }

    - name: Run Tests
      run: |
        source install/setup.bash
        colcon test --packages-select digital_twin_simulation
        colcon test-result --all

    - name: Upload Test Results
      uses: actions/upload-artifact@v3
      if: always()
      with:
        name: test-results
        path: |
          test_results/
```

### Advanced GitHub Actions Configuration

```yaml
# .github/workflows/advanced_robotics_ci.yml
name: Advanced Robotics CI/CD

on:
  push:
    branches: [ main ]
  pull_request:
    branches: [ main ]
  schedule:
    # Run daily at midnight UTC
    - cron: '0 0 * * *'

env:
  ROS_DISTRO: humble
  ROS_WS: /home/runner/work/ros_ws

jobs:
  build-and-test:
    runs-on: ubuntu-22.04
    container:
      image: ros:humble-ros-base-jammy

    strategy:
      matrix:
        include:
          - arch: amd64
          - arch: arm64
        exclude:
          - arch: arm64  # Exclude until ARM64 support is ready

    steps:
    - uses: actions/checkout@v3

    - name: Setup ROS Environment
      run: |
        echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> $GITHUB_ENV
        echo "source $ROS_WS/install/setup.bash" >> $GITHUB_ENV

    - name: Install System Dependencies
      run: |
        apt-get update
        apt-get install -y \
          python3-colcon-common-extensions \
          python3-rosdep \
          ros-$ROS_DISTRO-gazebo-ros-pkgs \
          ros-$ROS_DISTRO-ros2-control \
          ros-$ROS_DISTRO-ros2-controllers

    - name: Install Python Dependencies
      run: |
        pip3 install \
          launch-ros \
          ros2launch \
          pytest

    - name: Create Workspace
      run: |
        mkdir -p $ROS_WS/src
        cp -r . $ROS_WS/src/digital_twin_simulation

    - name: Install Package Dependencies
      run: |
        cd $ROS_WS
        rosdep install --from-paths src --ignore-src -r -y

    - name: Build Packages
      run: |
        cd $ROS_WS
        colcon build \
          --packages-select digital_twin_simulation \
          --cmake-args -DCMAKE_BUILD_TYPE=Release

    - name: Run Unit Tests
      run: |
        cd $ROS_WS
        colcon test --packages-select digital_twin_simulation
        colcon test-result --all --verbose

    - name: Run Integration Tests
      run: |
        cd $ROS_WS
        source install/setup.bash
        # Run integration tests with simulation
        ros2 launch digital_twin_simulation integration_test.launch.py

    - name: Generate Code Coverage
      run: |
        cd $ROS_WS
        pip3 install gcovr
        # Generate coverage report
        gcovr --root=. --xml --output=coverage.xml

    - name: Upload Artifacts
      uses: actions/upload-artifact@v3
      with:
        name: build-artifacts-${{ matrix.arch }}
        path: |
          ${{ env.ROS_WS }}/install/
          ${{ env.ROS_WS }}/test_results/
          ${{ env.ROS_WS }}/coverage.xml

    - name: Run Static Analysis
      run: |
        cd $ROS_WS
        pip3 install cppcheck
        cppcheck --enable=all --xml --xml-version=2 src/ 2> cppcheck.xml

    - name: Security Scanning
      run: |
        # Run security scanning on the codebase
        pip3 install bandit
        bandit -r src/ -f json -o bandit.json
```

## Docker-based CI/CD

### Multi-stage Docker Build with CI

```dockerfile
# Dockerfile.build
FROM ros:humble-ros-base-jammy AS builder

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_WS=/opt/ros_ws

# Install build dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

WORKDIR $ROS_WS

# Copy package files for dependency resolution
COPY package.xml CMakeLists.txt ./

# Install ROS dependencies
RUN rosdep install --from-paths . --ignore-src -r -y

# Copy source code
COPY . ./src/

# Build the workspace
RUN colcon build --packages-select digital_twin_simulation \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

# Runtime stage
FROM ros:humble-ros-base-jammy AS runtime

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_WS=/opt/ros_ws

# Install runtime dependencies
RUN apt-get update && apt-get install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-ros2-control \
    && rm -rf /var/lib/apt/lists/*

# Copy built artifacts
COPY --from=builder $ROS_WS/install $ROS_WS/install

# Source ROS environment
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "source $ROS_WS/install/setup.bash" >> /root/.bashrc

WORKDIR $ROS_WS
```

### CI/CD Pipeline with Docker

```yaml
# .github/workflows/docker_ci.yml
name: Docker CI/CD Pipeline

on:
  push:
    branches: [ main ]
  pull_request:
    branches: [ main ]

jobs:
  docker-build:
    runs-on: ubuntu-22.04

    steps:
    - uses: actions/checkout@v3

    - name: Set up Docker Buildx
      uses: docker/setup-buildx-action@v2

    - name: Login to DockerHub
      if: github.event_name != 'pull_request'
      uses: docker/login-action@v2
      with:
        username: ${{ secrets.DOCKERHUB_USERNAME }}
        password: ${{ secrets.DOCKERHUB_TOKEN }}

    - name: Extract metadata
      id: meta
      uses: docker/metadata-action@v4
      with:
        images: yourusername/digital-twin-sim
        tags: |
          type=ref,event=branch
          type=ref,event=pr
          type=sha,prefix={{branch}}-

    - name: Build and push
      uses: docker/build-push-action@v4
      with:
        context: .
        file: Dockerfile.build
        push: ${{ github.event_name != 'pull_request' }}
        tags: ${{ steps.meta.outputs.tags }}
        labels: ${{ steps.meta.outputs.labels }}
        cache-from: type=gha
        cache-to: type=gha,mode=max

    - name: Run container tests
      run: |
        docker build -t digital-twin-test -f Dockerfile.test .
        docker run --rm digital-twin-test pytest /tests/

    - name: Security scan
      run: |
        docker scan digital-twin-test
```

## Testing Strategies in CI/CD

### Unit Testing for Robotics

```python
# test/test_synchronization_node.py
import unittest
import rclpy
from digital_twin_simulation.synchronization_node import SynchronizationNode

class TestSynchronizationNode(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = SynchronizationNode()

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def test_joint_state_sync(self):
        """Test that joint states are properly synchronized"""
        # Mock joint state message
        joint_state_msg = JointState()
        joint_state_msg.name = ['joint1', 'joint2']
        joint_state_msg.position = [1.0, 2.0]

        # Call synchronization method
        result = self.node.synchronize_joint_states(joint_state_msg)

        # Assert expected behavior
        self.assertEqual(len(result), 2)
        self.assertAlmostEqual(result[0], 1.0, places=3)

    def test_latency_measurement(self):
        """Test latency measurement functionality"""
        start_time = self.node.get_clock().now()
        # Simulate some processing
        result = self.node.process_data()
        end_time = self.node.get_clock().now()

        latency = (end_time - start_time).nanoseconds / 1e9
        self.assertLess(latency, 0.1)  # Less than 100ms

if __name__ == '__main__':
    unittest.main()
```

### Integration Testing

```python
# test/test_integration.py
import unittest
import rclpy
from rclpy.qos import QoSProfile
import time

class TestDigitalTwinIntegration(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node('integration_tester')

        # Create publishers and subscribers for testing
        self.joint_pub = self.node.create_publisher(
            JointState, '/joint_states',
            QoSProfile(depth=10)
        )
        self.sync_sub = self.node.create_subscription(
            JointState, '/sync_joint_states',
            self.sync_callback,
            QoSProfile(depth=10)
        )

        self.received_sync = False
        self.sync_msg = None

    def sync_callback(self, msg):
        self.received_sync = True
        self.sync_msg = msg

    def test_end_to_end_sync(self):
        """Test complete synchronization pipeline"""
        # Publish joint states
        joint_msg = JointState()
        joint_msg.name = ['test_joint']
        joint_msg.position = [1.5]

        # Publish and wait for synchronization
        self.joint_pub.publish(joint_msg)

        # Wait for sync message
        timeout = time.time() + 10.0  # 10 second timeout
        while not self.received_sync and time.time() < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.assertTrue(self.received_sync)
        self.assertIsNotNone(self.sync_msg)
        self.assertEqual(self.sync_msg.name[0], 'test_joint')
        self.assertAlmostEqual(self.sync_msg.position[0], 1.5, places=2)

if __name__ == '__main__':
    unittest.main()
```

## Deployment Pipelines

### Automated Deployment Configuration

```yaml
# .github/workflows/deployment.yml
name: Deployment Pipeline

on:
  push:
    tags:
      - 'v*'  # Trigger on version tags

jobs:
  deploy:
    runs-on: ubuntu-22.04
    environment: production

    steps:
    - uses: actions/checkout@v3

    - name: Setup ROS
      uses: ros-tooling/setup-ros@0.7.3
      with:
        required-ros-distributions: humble

    - name: Build Release
      run: |
        mkdir -p ~/release_ws/src
        cp -r . ~/release_ws/src/digital_twin_simulation
        cd ~/release_ws
        rosdep install --from-paths src --ignore-src -r -y
        colcon build --packages-select digital_twin_simulation \
          --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

    - name: Create Release Package
      run: |
        cd ~/release_ws
        tar -czf digital_twin_simulation-$(date +%Y%m%d-%H%M%S).tar.gz install/

    - name: Create GitHub Release
      uses: softprops/action-gh-release@v1
      with:
        files: |
          ~/release_ws/*.tar.gz
        draft: false
        prerelease: false

    - name: Deploy to Target Systems
      run: |
        # Deploy to production systems
        # This would include your specific deployment steps
        echo "Deploying to production systems..."

    - name: Run Post-deployment Tests
      run: |
        # Verify deployment was successful
        # Run health checks, etc.
        echo "Running post-deployment tests..."
```

## Performance and Quality Gates

### Performance Testing in CI/CD

```python
# performance_test.py
import subprocess
import time
import json

def run_performance_test():
    """Run performance tests and generate metrics"""
    start_time = time.time()

    # Start the system under test
    process = subprocess.Popen([
        'ros2', 'launch', 'digital_twin_simulation', 'performance_test.launch.py'
    ])

    # Collect metrics for 60 seconds
    metrics = []
    for i in range(60):
        # Collect system metrics
        cpu_usage = get_cpu_usage()
        memory_usage = get_memory_usage()
        latency = measure_latency()

        metrics.append({
            'timestamp': time.time(),
            'cpu': cpu_usage,
            'memory': memory_usage,
            'latency': latency
        })

        time.sleep(1)

    # Stop the process
    process.terminate()
    process.wait()

    # Calculate performance metrics
    avg_cpu = sum(m['cpu'] for m in metrics) / len(metrics)
    avg_latency = sum(m['latency'] for m in metrics) / len(metrics)

    return {
        'average_cpu': avg_cpu,
        'average_latency': avg_latency,
        'max_memory': max(m['memory'] for m in metrics),
        'duration': time.time() - start_time
    }

def check_performance_gates(metrics):
    """Check if performance metrics meet requirements"""
    gates = {
        'cpu_threshold': 80.0,  # percent
        'latency_threshold': 0.05,  # seconds
        'memory_threshold': 2.0  # GB
    }

    if metrics['average_cpu'] > gates['cpu_threshold']:
        raise Exception(f"CPU usage too high: {metrics['average_cpu']}% > {gates['cpu_threshold']}%")

    if metrics['average_latency'] > gates['latency_threshold']:
        raise Exception(f"Latency too high: {metrics['average_latency']}s > {gates['latency_threshold']}s")

    print("Performance gates passed!")
```

### Quality Gates Configuration

```yaml
# .github/workflows/quality_gates.yml
name: Quality Gates

on:
  pull_request:
    branches: [ main ]

jobs:
  quality-checks:
    runs-on: ubuntu-22.04

    steps:
    - uses: actions/checkout@v3

    - name: Code Quality Analysis
      run: |
        # Run static analysis
        pip3 install pylint mypy
        pylint src/ --output-format=text --reports=yes
        mypy src/

    - name: Security Scanning
      run: |
        # Run security tools
        pip3 install bandit safety
        bandit -r src/
        safety check

    - name: License Compliance
      run: |
        # Check for license compliance
        python3 scripts/check_licenses.py

    - name: Code Coverage Check
      run: |
        # Check code coverage meets minimum requirements
        COVERAGE=$(python3 scripts/check_coverage.py)
        if [ $COVERAGE -lt 80 ]; then
          echo "Coverage too low: $COVERAGE% < 80%"
          exit 1
        fi
```

## Monitoring and Observability

### Pipeline Monitoring

```python
# monitoring/pipeline_monitor.py
import requests
import json
from datetime import datetime

class PipelineMonitor:
    def __init__(self, webhook_url):
        self.webhook_url = webhook_url

    def report_build_status(self, status, duration, artifacts):
        """Report build status to monitoring system"""
        payload = {
            'timestamp': datetime.utcnow().isoformat(),
            'status': status,
            'duration': duration,
            'artifacts': artifacts,
            'pipeline': 'robotics_ci_cd',
            'environment': 'production'
        }

        requests.post(self.webhook_url, json=payload)

    def check_pipeline_health(self):
        """Check overall pipeline health"""
        # Implementation to check pipeline metrics
        pass

# Example usage in pipeline
monitor = PipelineMonitor('https://hooks.example.com/webhook')
monitor.report_build_status('success', 300, ['build_artifact.tar.gz'])
```

## Exercise: Implement CI/CD Pipeline

1. **Create a GitHub Actions workflow** for your robotics project
2. **Implement unit and integration tests** that run in the pipeline
3. **Add quality gates** for code coverage and security scanning
4. **Set up deployment automation** for tagged releases
5. **Configure monitoring** for pipeline performance

## Troubleshooting CI/CD Issues

### Common Pipeline Issues
- **Dependency resolution failures**: Ensure rosdep keys are properly defined
- **Build timeouts**: Optimize build processes and increase timeout limits
- **Test flakiness**: Implement proper test isolation and cleanup
- **Resource constraints**: Use appropriate runner specifications

### Debugging Strategies
- Use detailed logging in pipeline steps
- Implement health checks for complex systems
- Use caching to speed up builds
- Set up proper notifications for failures

## Summary

In this chapter, we've covered CI/CD pipelines for robotics:
- GitHub Actions configuration for robotics projects
- Docker-based build and testing strategies
- Comprehensive testing approaches (unit, integration, performance)
- Deployment automation and quality gates
- Monitoring and observability practices

In the next chapter, we'll explore cloud deployment and monitoring for robotics applications.