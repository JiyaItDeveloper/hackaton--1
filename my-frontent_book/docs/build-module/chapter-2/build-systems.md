# Chapter 2: ROS 2 Build Systems

## Introduction

ROS 2 uses the colcon build system to manage packages and dependencies. In this chapter, we'll explore how to create, build, and manage ROS 2 packages, focusing on best practices for complex robotics applications like our digital twin simulation system.

## Understanding colcon

colcon is the command-line build tool for ROS 2 workspaces. It replaces the older catkin build system and provides better support for:
- Mixed build systems (CMake, Python, etc.)
- Parallel builds
- Flexible workspace layouts
- Improved dependency management

### colcon Architecture

```
Workspace Root
├── src/ (source packages)
├── build/ (intermediate build files)
├── install/ (installed packages)
└── log/ (build logs)
```

## Creating ROS 2 Packages

### Package Structure

A typical ROS 2 package follows this structure:

```
digital_twin_simulation/
├── CMakeLists.txt
├── package.xml
├── src/
│   ├── synchronization_node.cpp
│   └── other_cpp_files.cpp
├── include/
│   └── digital_twin_simulation/
│       └── header_files.hpp
├── scripts/
├── launch/
├── config/
├── test/
└── README.md
```

### package.xml Template

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>digital_twin_simulation</name>
  <version>0.1.0</version>
  <description>Digital twin simulation system for humanoid robots</description>
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

  <exec_depend>ros2launch</exec_depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### CMakeLists.txt Template

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

# Add executable
add_executable(synchronization_node src/synchronization_node.cpp)
ament_target_dependencies(synchronization_node
  rclcpp
  std_msgs
  sensor_msgs
  builtin_interfaces
)

# Install targets
install(TARGETS
  synchronization_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

## Building with colcon

### Basic Build Commands

```bash
# Create workspace
mkdir -p ~/digital_twin_ws/src
cd ~/digital_twin_ws

# Build all packages
colcon build

# Build specific package
colcon build --packages-select digital_twin_simulation

# Build with specific build type
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Build with parallel jobs
colcon build --parallel-workers 4

# Build and install in one step
colcon build --install-base /opt/ros/humble
```

### Advanced Build Options

```bash
# Clean build
colcon build --clean-build

# Clean all
colcon build --clean-all

# Build with specific compiler
colcon build --cmake-args -DCMAKE_C_COMPILER=gcc -DCMAKE_CXX_COMPILER=g++

# Build with verbose output
colcon build --event-handlers console_direct+

# Build with symlinks (faster)
colcon build --symlink-install
```

## Package Management Strategies

### Workspace Overlaying

```bash
# Source the base ROS installation
source /opt/ros/humble/setup.bash

# Build and source your workspace
cd ~/digital_twin_ws
colcon build
source install/setup.bash

# Your packages will now be available alongside base ROS packages
```

### Package Selection and Filtering

```bash
# Build packages by regex
colcon build --packages-select "digital_twin_*"

# Build packages except specific ones
colcon build --packages-skip gazebo_ros_pkgs

# Build packages in topological order
colcon build --packages-up-to digital_twin_simulation

# Build packages that depend on a specific package
colcon build --packages-above-and-dependencies digital_twin_simulation
```

## CMake Best Practices for ROS 2

### Conditional Compilation

```cmake
# Enable specific features based on build type
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  add_compile_definitions(DEBUG_MODE)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0")
elseif(CMAKE_BUILD_TYPE STREQUAL "Release")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -DNDEBUG")
endif()

# Platform-specific compilation
if(WIN32)
  # Windows-specific settings
elseif(APPLE)
  # macOS-specific settings
elseif(UNIX)
  # Linux-specific settings
endif()
```

### Dependency Management

```cmake
# Find ROS 2 packages with version requirements
find_package(rclcpp REQUIRED VERSION 1.0.0)

# Find system libraries
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

# Add system library
target_link_libraries(your_node ${YAML_CPP_LIBRARIES})
target_include_directories(your_node PRIVATE ${YAML_CPP_INCLUDE_DIRS})
```

## Python Package Integration

### Python Package Structure

```
digital_twin_simulation_py/
├── package.xml
├── setup.py
├── setup.cfg
├── digital_twin_simulation_py/
│   ├── __init__.py
│   ├── synchronization_node.py
│   └── utils/
└── test/
```

### setup.py Template

```python
from setuptools import setup
import os
from glob import glob

package_name = 'digital_twin_simulation_py'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Include config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Python nodes for digital twin simulation',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'synchronization_node = digital_twin_simulation_py.synchronization_node:main',
        ],
    },
)
```

## Build System Optimization

### Fast Incremental Builds

```bash
# Use ccache for faster rebuilds
colcon build --cmake-args -DCMAKE_CXX_COMPILER_LAUNCHER=ccache

# Build only changed packages
colcon build --packages-select $(colcon list --packages-up-to --paths src/)

# Use build cache
colcon build --build-base build_cached
```

### Memory and Performance Optimization

```bash
# Limit memory usage per job
colcon build --parallel-workers 2 --executor sequential

# Use Ninja generator for faster builds
colcon build --cmake-args -GNinja

# Build with minimal output
colcon build --event-handlers console_cohesion+
```

## Cross-Platform Builds

### Architecture-Specific Builds

```bash
# Build for ARM64
colcon build --cmake-args -DCMAKE_SYSTEM_PROCESSOR=aarch64

# Cross-compile for different architectures
colcon build --cmake-args \
  -DCMAKE_TOOLCHAIN_FILE=toolchain.cmake \
  -DCMAKE_SYSTEM_NAME=Linux
```

## Testing and Quality Assurance

### Integration with Build System

```cmake
# CMakeLists.txt additions for testing
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  # Add C++ tests
  ament_add_gtest(test_synchronization test/test_synchronization.cpp)
  ament_target_dependencies(test_synchronization rclcpp std_msgs)

  # Add Python tests
  add_subdirectory(test)
endif()
```

### Code Quality Tools

```bash
# Lint packages
colcon build --packages-select digital_twin_simulation --cmake-target ament_clang_format
colcon build --packages-select digital_twin_simulation --cmake-target ament_clang_tidy

# Run tests during build
colcon build --packages-select digital_twin_simulation --test-result-base test_results
```

## Performance Profiling

### Build Performance Analysis

```bash
# Profile build times
colcon build --event-handlers console_direct+ --executor sequential

# Generate build time reports
colcon build --event-handlers console_cohesion+ --build-base build_profiled
```

### Runtime Performance

```bash
# Build with profiling enabled
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

# Build with specific performance flags
colcon build --cmake-args \
  -DCMAKE_CXX_FLAGS_RELEASE="-O3 -DNDEBUG -march=native" \
  -DCMAKE_C_FLAGS_RELEASE="-O3 -DNDEBUG -march=native"
```

## Exercise: Create a Complete ROS 2 Package

1. **Create a new package** for digital twin simulation
2. **Define proper dependencies** in package.xml
3. **Write CMakeLists.txt** with proper build configuration
4. **Build the package** using colcon
5. **Test the build** and verify functionality

## Troubleshooting Build Issues

### Common Build Errors
- **Missing dependencies**: Use `rosdep install` to install missing packages
- **CMake errors**: Check CMakeLists.txt for syntax and dependency issues
- **Linker errors**: Verify target dependencies are properly specified
- **Include path issues**: Ensure headers are properly installed

### Debugging Strategies
- Use `colcon build --symlink-install` for faster rebuilds during development
- Check build logs in the log directory
- Use `--cmake-args -DCMAKE_VERBOSE_MAKEFILE=ON` for detailed output
- Build individual packages to isolate issues

## Summary

In this chapter, we've covered ROS 2 build systems:
- Understanding colcon and workspace structure
- Creating proper package.xml and CMakeLists.txt files
- Advanced build techniques and optimization
- Python package integration
- Testing and quality assurance practices

In the next chapter, we'll explore continuous integration and deployment pipelines.