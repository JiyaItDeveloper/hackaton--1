---
sidebar_position: 2
---

# Chapter 1: ROS 2 Fundamentals

## Learning Goals
- Understand the purpose of ROS 2 in Physical AI
- Learn ROS 2 architecture and design goals
- Compare ROS 2 vs ROS 1 at a high level

## Introduction to ROS 2 as a Middleware

ROS 2 (Robot Operating System 2) is not an operating system but rather a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms.

### Key Design Principles
- **Distributed Systems**: ROS 2 is designed to work in distributed environments where multiple processes and potentially multiple machines need to communicate seamlessly.
- **Message Passing**: The core communication paradigm is based on passing messages between different components (nodes) of the robot system.
- **Real-time Capabilities**: Enhanced support for real-time systems compared to ROS 1.
- **Security**: Built-in security features to protect robot systems from unauthorized access.
- **Multi-robot Systems**: Better support for coordinating multiple robots.

## ROS 2 vs ROS 1: Key Advantages

### 1. Improved Architecture
- **DDS-based**: ROS 2 is built on Data Distribution Service (DDS), providing better support for distributed systems
- **Middleware Agnostic**: Can work with different DDS implementations (Fast DDS, Cyclone DDS, RTI Connext DDS)

### 2. Real-time Support
- Better real-time capabilities for time-critical applications
- Improved determinism in message delivery

### 3. Security Features
- Built-in security framework with authentication, access control, and encryption
- Secure communication between nodes

### 4. Cross-platform Support
- Better support for different operating systems (Linux, Windows, macOS)
- Improved support for embedded systems

## Setting up Your ROS 2 Environment

### Installation Requirements
- **ROS 2 Distribution**: Humble Hawksbill (or latest LTS version)
- **Python**: 3.8+
- **Operating System**: Ubuntu 22.04 LTS (recommended) or equivalent ROS 2 supported platform

### Basic Workspace Structure
A typical ROS 2 workspace follows this structure:
```
workspace_folder/      # e.g., ~/ros2_ws
  src/                 # Source code
    package1/
      CMakeLists.txt
      package.xml
      src/
      include/
      test/
    package2/
      ...
```

## Simple Publisher/Subscriber Example

Let's start with a basic example that demonstrates the core communication pattern in ROS 2:

```python
# publisher_member_function.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

This example demonstrates the basic publisher pattern in ROS 2, which will be fundamental to our humanoid control systems.

## Summary

In this chapter, you've learned about the core concepts of ROS 2, its advantages over ROS 1, and the basic structure of a ROS 2 system. In the next chapter, we'll dive deeper into the communication model and learn about nodes, topics, services, and actions.