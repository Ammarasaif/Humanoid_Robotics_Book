---
sidebar_position: 3
---

# Chapter 3: Building Python Agents with rclpy

## Overview

This chapter focuses on the Python client library for ROS 2, known as `rclpy`. We'll explore how to create Python-based agents that can interact with the ROS 2 ecosystem, implement nodes, and communicate with other robotic components. Python's simplicity and extensive ecosystem make it an ideal choice for rapid prototyping and developing sophisticated robotic applications.

## Learning Objectives

By the end of this chapter, you will:
- Understand the structure and usage of the `rclpy` library
- Be able to create ROS 2 nodes using Python
- Implement publishers, subscribers, services, and clients in Python
- Learn best practices for Python-based robotic agents
- Understand how to integrate Python agents with other ROS 2 components

## Introduction to rclpy

### What is rclpy?

`rclpy` is the Python client library for ROS 2. It provides Python bindings to the ROS 2 client library (rcl), which in turn interfaces with the underlying middleware (DDS). `rclpy` allows Python developers to create ROS 2 nodes that can communicate with nodes written in other languages like C++.

### Why Python for Robotics?

Python has become increasingly popular in robotics for several reasons:
- **Rapid Prototyping**: Python's simplicity allows for quick development and testing
- **Rich Ecosystem**: Extensive libraries for machine learning, computer vision, and mathematics
- **Scientific Computing**: Strong support for NumPy, SciPy, and other scientific libraries
- **Community**: Large community of researchers and developers
- **Integration**: Easy integration with ROS 2 and other robotic frameworks

## Setting Up rclpy

### Installation

To use `rclpy`, you need to have ROS 2 properly installed on your system. The library is typically included with the ROS 2 installation. You can import it in your Python scripts with:

```python
import rclpy
from rclpy.node import Node
```

### Basic Node Structure

Every ROS 2 Python node follows a similar structure:

```python
import rclpy
from rclpy.node import Node

def main(args=None):
    rclpy.init(args=args)  # Initialize ROS communications
    node = MyNode()        # Create your custom node
    rclpy.spin(node)       # Keep the node running
    node.destroy_node()    # Clean up
    rclpy.shutdown()       # Shutdown ROS communications

if __name__ == '__main__':
    main()
```

## Creating Custom Nodes

### Basic Node Class

A custom node inherits from `rclpy.node.Node`:

```python
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_publisher')  # Node name
        # Initialize publishers, subscribers, etc. here
```

### Node Initialization Best Practices

- Call `super().__init__('node_name')` to properly initialize the node
- Use descriptive names that follow ROS naming conventions
- Initialize all communication interfaces in the constructor
- Use the node's logger for debugging and information messages

## Implementing Publishers

### Creating a Publisher

Publishers send messages to topics:

```python
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.get_clock().now().nanoseconds
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
```

### Publisher Best Practices

- Use appropriate queue sizes based on your application needs
- Implement proper error handling for message publishing
- Consider QoS settings for different communication requirements
- Use descriptive topic names that follow ROS conventions

## Implementing Subscribers

### Creating a Subscriber

Subscribers receive messages from topics:

```python
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
```

### Subscriber Best Practices

- Handle messages efficiently to avoid blocking the callback
- Use appropriate queue sizes to prevent message loss
- Consider threading models for CPU-intensive message processing
- Implement proper error handling for message reception

## Implementing Services

### Creating a Service Server

Service servers respond to requests:

```python
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response
```

### Creating a Service Client

Service clients make requests to service servers:

```python
from example_interfaces.srv import AddTwoInts

class MinimalClient(Node):
    def __init__(self):
        super().__init__('minimal_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
```

## Working with Parameters

### Declaring and Using Parameters

Parameters allow runtime configuration of nodes:

```python
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

class ParamNode(Node):
    def __init__(self):
        super().__init__('param_node')

        # Declare parameters with default values
        self.declare_parameter('my_param', 'default_value')
        self.declare_parameter('threshold', 0.5)

        # Get parameter values
        my_param = self.get_parameter('my_param').value
        threshold = self.get_parameter('threshold').value

        self.get_logger().info(f'My param: {my_param}, Threshold: {threshold}')
```

### Parameter Callbacks

Handle parameter changes dynamically:

```python
def parameter_callback(self, parameters):
    for param in parameters:
        if param.name == 'threshold' and param.type_ == Parameter.Type.DOUBLE:
            self.threshold = param.value
            self.get_logger().info(f'Updated threshold to: {self.threshold}')
    return SetParametersResult(successful=True)

# Register the callback
self.add_on_set_parameters_callback(self.parameter_callback)
```

## Advanced rclpy Features

### Timers

Timers allow periodic execution of functions:

```python
def __init__(self):
    super().__init__('timer_node')
    self.timer = self.create_timer(0.1, self.timer_callback)  # 0.1 seconds

def timer_callback(self):
    self.get_logger().info('Timer callback executed')
```

### Multi-threading

For CPU-intensive operations, consider using multi-threading:

```python
import threading
from rclpy.qos import QoSProfile

def __init__(self):
    super().__init__('multithreaded_node')
    self.subscription = self.create_subscription(
        String, 'topic', self.process_message, QoSProfile(depth=10))

def process_message(self, msg):
    # Start processing in a separate thread
    thread = threading.Thread(target=self.computationally_intensive_task, args=(msg,))
    thread.start()

def computationally_intensive_task(self, msg):
    # Perform heavy computation without blocking the main thread
    pass
```

### Actions

For long-running operations with feedback:

```python
from rclpy.action import ActionServer
from example_interfaces.action import Fibonacci

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()

            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1])

            goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        return result
```

## Best Practices for Python Agents

### Error Handling

Always implement proper error handling:

```python
try:
    # ROS operations
    rclpy.spin(node)
except KeyboardInterrupt:
    node.get_logger().info('Interrupted by user')
except Exception as e:
    node.get_logger().error(f'Error occurred: {e}')
finally:
    node.destroy_node()
    rclpy.shutdown()
```

### Resource Management

Ensure proper cleanup of resources:

```python
def destroy_node(self):
    # Clean up any resources before destroying the node
    if hasattr(self, 'publisher_'):
        self.publisher_.destroy()
    if hasattr(self, 'subscription'):
        self.subscription.destroy()
    if hasattr(self, 'timer'):
        self.timer.destroy()
    super().destroy_node()
```

### Performance Considerations

- Minimize data copying in callbacks
- Use efficient data structures
- Consider memory usage for long-running nodes
- Profile CPU-intensive operations

## Integration with Humanoid Robotics

### Sensor Data Processing

Python is excellent for processing sensor data from humanoid robots:

```python
from sensor_msgs.msg import JointState
import numpy as np

class JointStateProcessor(Node):
    def __init__(self):
        super().__init__('joint_state_processor')
        self.subscription = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10)

    def joint_state_callback(self, msg):
        # Process joint positions, velocities, efforts
        positions = np.array(msg.position)
        velocities = np.array(msg.velocity)
        efforts = np.array(msg.effort)

        # Apply filtering, control algorithms, etc.
        processed_data = self.apply_control_algorithm(positions, velocities)

        # Publish results
        self.publish_control_commands(processed_data)
```

### AI and Machine Learning Integration

Leverage Python's ML ecosystem with ROS 2:

```python
import tensorflow as tf
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class MLProcessor(Node):
    def __init__(self):
        super().__init__('ml_processor')
        self.bridge = CvBridge()
        self.model = tf.keras.models.load_model('path/to/model')
        self.subscription = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10)

    def image_callback(self, msg):
        # Convert ROS image to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Preprocess and run inference
        input_tensor = self.preprocess_image(cv_image)
        prediction = self.model(input_tensor)

        # Process results and publish
        self.publish_prediction_results(prediction)
```

## Summary

In this chapter, we've explored how to build Python agents using the `rclpy` library. We've covered the fundamental components of ROS 2 nodes, including publishers, subscribers, services, and parameters. We've also looked at advanced features like timers, multi-threading, and actions, and discussed best practices for developing robust Python-based robotic agents. Python's integration with scientific computing and machine learning libraries makes it an invaluable tool for humanoid robotics applications.

## Exercises

1. Create a Python node that publishes sensor data with configurable parameters
2. Implement a subscriber that processes data using NumPy operations
3. Build a service that performs a specific calculation or transformation
4. Design a multi-threaded node that handles both real-time and non-real-time tasks