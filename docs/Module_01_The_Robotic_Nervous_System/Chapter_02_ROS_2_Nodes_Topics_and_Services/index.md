---
sidebar_position: 2
---

# Chapter 2: Understanding ROS 2 Nodes, Topics, and Services

## Overview

In this chapter, we'll dive deep into the fundamental communication mechanisms of ROS 2: nodes, topics, and services. These elements form the backbone of any ROS 2 application and are essential for creating distributed robotic systems that can effectively coordinate their activities.

## Learning Objectives

By the end of this chapter, you will:
- Understand the concept and implementation of ROS 2 nodes
- Master the publisher-subscriber communication pattern using topics
- Learn about service-based synchronous communication
- Know when to use topics versus services for different use cases
- Be able to implement basic ROS 2 communication patterns in Python

## ROS 2 Nodes: The Foundation

### What is a Node?

A node is a process that performs computation in ROS. Nodes are the fundamental building blocks of a ROS 2 system. Each node typically performs a specific function within the larger robotic application. For example, one node might handle camera data processing, another might control wheel motors, and yet another might plan navigation routes.

### Node Characteristics

- **Single Responsibility**: Each node should perform a specific task
- **Communication Hub**: Nodes communicate with other nodes through topics, services, and actions
- **Lifecycle Management**: Nodes can be started, stopped, and managed independently
- **Parameter Configuration**: Nodes can accept parameters to modify their behavior

### Node Implementation

In ROS 2, nodes are implemented as classes that inherit from `rclpy.Node` (in Python) or `rclcpp::Node` (in C++). A node typically includes:

1. **Initialization**: Setting up the node with a unique name
2. **Communication Interfaces**: Creating publishers, subscribers, services, and clients
3. **Execution Loop**: Processing data and performing computations
4. **Cleanup**: Properly shutting down resources

## Topics and Publishers/Subscribers

### Publisher-Subscriber Pattern

The publisher-subscriber pattern is the primary communication mechanism in ROS 2. It enables asynchronous communication between nodes:

- **Publishers** send messages to a topic
- **Subscribers** receive messages from a topic
- **Topics** serve as named buses over which messages are sent

This pattern creates loose coupling between nodes, meaning publishers don't need to know which nodes are subscribing to their messages, and subscribers don't need to know which nodes are publishing messages to their subscribed topics.

### Message Types

Messages in ROS 2 are strongly typed and defined using Interface Definition Language (IDL) files. Common message types include:
- `std_msgs`: Basic data types (integers, floats, strings, etc.)
- `sensor_msgs`: Sensor data (images, laser scans, IMU data, etc.)
- `geometry_msgs`: Geometric primitives (points, poses, transforms, etc.)
- Custom message types defined for specific applications

### Quality of Service (QoS)

ROS 2 introduces QoS settings that allow fine-tuning of communication characteristics:

- **Reliability**: Best effort vs. reliable delivery
- **Durability**: Volatile vs. transient local (for late-joining subscribers)
- **History**: Keep last N messages vs. keep all messages
- **Rate**: Publishing frequency constraints

## Services and Client-Server Pattern

### Synchronous Communication

While topics provide asynchronous communication, services enable synchronous request-response communication. This is useful for operations that require a definitive response or when the caller needs to wait for completion.

### Service Characteristics

- **Request-Response**: A client sends a request and waits for a response
- **Blocking Calls**: The client thread is blocked until a response is received
- **Defined Interface**: Services have strictly defined request and response message types
- **Suitable for Actions**: Good for operations like calibration, configuration, or triggering specific behaviors

### When to Use Services vs Topics

**Use Topics when:**
- Continuous data streaming is required
- Loose coupling between sender and receiver is desired
- Multiple subscribers need the same data
- Real-time performance is critical

**Use Services when:**
- A specific action needs to be triggered
- Request-response semantics are required
- The operation has a clear beginning and end
- Error handling and return codes are important

## Practical Implementation Example

Let's look at a practical example of how nodes, topics, and services work together in a humanoid robot context:

### Node Structure Example
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from my_robot_interfaces.srv import CalibrateJoint

class JointController(Node):
    def __init__(self):
        super().__init__('joint_controller')

        # Create publisher for joint state
        self.joint_state_publisher = self.create_publisher(
            JointState, 'joint_states', 10)

        # Create subscriber for commands
        self.command_subscriber = self.create_subscription(
            String, 'joint_commands', self.command_callback, 10)

        # Create service server for calibration
        self.calibration_service = self.create_service(
            CalibrateJoint, 'calibrate_joint', self.calibrate_callback)

    def command_callback(self, msg):
        # Process incoming commands
        self.get_logger().info(f'Received command: {msg.data}')

    def calibrate_callback(self, request, response):
        # Handle calibration request
        self.get_logger().info(f'Calibrating joint: {request.joint_name}')
        response.success = True
        response.message = f'Calibrated {request.joint_name}'
        return response
```

## Advanced Communication Patterns

### Actions

For long-running operations with feedback, ROS 2 provides Actions, which combine the benefits of both topics and services:
- Goal: Similar to a service request
- Feedback: Continuous updates during execution (like topics)
- Result: Final outcome (like a service response)

### Parameter Server

ROS 2 includes a distributed parameter system that allows nodes to share configuration values dynamically without requiring recompilation.

## Best Practices

1. **Modular Design**: Create nodes with single responsibilities
2. **Appropriate QoS**: Choose QoS settings based on application requirements
3. **Message Efficiency**: Use appropriate message types and sizes
4. **Error Handling**: Implement proper error handling for network communications
5. **Naming Conventions**: Use consistent and descriptive names for topics and services

## Summary

In this chapter, we've explored the fundamental communication mechanisms of ROS 2. We've learned how nodes form the basic computational units, how topics enable asynchronous communication through the publisher-subscriber pattern, and how services provide synchronous request-response interactions. These concepts are crucial for building distributed robotic systems that can effectively coordinate their activities.

## Exercises

1. Create a simple publisher node that publishes sensor data
2. Implement a subscriber that processes the published data
3. Design a service that performs a specific robot action
4. Experiment with different QoS settings and observe their effects