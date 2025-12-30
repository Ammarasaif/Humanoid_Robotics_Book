---
sidebar_position: 4
---

# Chapter 4: ROS 2 Services and Communication Patterns

## Overview

This chapter delves into the various communication patterns available in ROS 2, with a focus on services and how they complement other communication mechanisms. We'll explore when and how to use different patterns for optimal robotic system design, particularly in the context of humanoid robotics where coordination between multiple subsystems is crucial.

## Learning Objectives

By the end of this chapter, you will:
- Master the implementation and usage of ROS 2 services
- Understand advanced communication patterns and their appropriate use cases
- Learn how to design effective communication architectures for humanoid robots
- Implement complex communication scenarios using multiple patterns
- Understand Quality of Service (QoS) settings and their impact on communication

## Deep Dive into Services

### Service Architecture

Services in ROS 2 follow a client-server model where:
- **Service Server**: Implements the service and responds to requests
- **Service Client**: Sends requests and receives responses
- **Service Interface**: Defines the request and response message types

### Service Definition

Services are defined using `.srv` files with the following structure:
```
# Request message
int64 a
int64 b
---
# Response message
int64 sum
```

The part before `---` defines the request message, and the part after defines the response message.

### Implementing Service Servers

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class ServiceServer(Node):
    def __init__(self):
        super().__init__('service_server')
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_callback
        )

    def add_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Request: {request.a} + {request.b}')
        self.get_logger().info(f'Response: {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    service_server = ServiceServer()
    rclpy.spin(service_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Implementing Service Clients

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class ServiceClient(Node):
    def __init__(self):
        super().__init__('service_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.request = AddTwoInts.Request()

    def send_request(self, a, b):
        self.request.a = a
        self.request.b = b
        self.future = self.cli.call_async(self.request)
        return self.future

def main(args=None):
    rclpy.init(args=args)
    service_client = ServiceClient()

    # Send request
    future = service_client.send_request(1, 2)

    # Wait for response
    rclpy.spin_until_future_complete(service_client, future)

    if future.result() is not None:
        response = future.result()
        service_client.get_logger().info(f'Result: {response.sum}')
    else:
        service_client.get_logger().error('Exception while calling service: %r' % future.exception())

    service_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Communication Pattern Comparison

### Topics vs Services vs Actions

| Pattern | Type | Use Case | Characteristics |
|---------|------|----------|-----------------|
| Topics | Asynchronous | Continuous data streams | Publisher-subscriber, loose coupling |
| Services | Synchronous | Request-response operations | Blocking calls, immediate response |
| Actions | Asynchronous with feedback | Long-running tasks | Goal-feedback-result pattern |

### When to Use Each Pattern

**Use Topics when:**
- Broadcasting sensor data continuously
- Publishing control commands at high frequency
- Multiple nodes need to receive the same information
- Real-time performance is critical
- Loose coupling between nodes is desired

**Use Services when:**
- Performing a specific action with a clear result
- Need guaranteed delivery and response
- Operations have a defined start and end
- Configuration or calibration tasks
- Querying current state or parameters

**Use Actions when:**
- Executing long-running operations (navigation, manipulation)
- Need feedback during execution
- Operations can be preempted or canceled
- Complex workflows with intermediate results

## Advanced Communication Patterns

### Publisher-Subscriber with Services

Combining patterns for complex scenarios:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from example_interfaces.srv import Trigger

class HybridNode(Node):
    def __init__(self):
        super().__init__('hybrid_node')

        # Publisher for status updates
        self.status_publisher = self.create_publisher(String, 'status', 10)

        # Service for triggering operations
        self.trigger_service = self.create_service(
            Trigger, 'trigger_operation', self.trigger_callback
        )

        # Timer for periodic status updates
        self.timer = self.create_timer(1.0, self.status_timer_callback)
        self.operation_active = False

    def trigger_callback(self, request, response):
        self.operation_active = True
        response.success = True
        response.message = "Operation started"

        # Publish status update
        status_msg = String()
        status_msg.data = "Operation started by service call"
        self.status_publisher.publish(status_msg)

        return response

    def status_timer_callback(self):
        if self.operation_active:
            status_msg = String()
            status_msg.data = "Operation running"
            self.status_publisher.publish(status_msg)
```

### Quality of Service (QoS) Patterns

QoS settings allow fine-tuning communication behavior:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# For critical control commands
critical_qos = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE
)

# For sensor data where some loss is acceptable
sensor_qos = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE
)

# For configuration parameters that should persist
config_qos = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL
)

# Using QoS in publishers and subscribers
publisher = self.create_publisher(String, 'critical_cmd', critical_qos)
subscriber = self.create_subscription(String, 'sensor_data', callback, sensor_qos)
```

## Communication Design Patterns for Humanoid Robotics

### Sensor Fusion Pattern

Multiple sensors publishing to a central fusion node:

```
IMU Publisher ──┐
Camera Publisher ──┼──→ Sensor Fusion Node ──→ Processed Data
Lidar Publisher ──┘
```

Implementation example:

```python
from sensor_msgs.msg import Imu, Image, LaserScan
from custom_msgs.msg import FusedSensorData

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion')

        # Multiple subscribers for different sensors
        self.imu_sub = self.create_subscription(Imu, 'imu/data', self.imu_callback, 10)
        self.cam_sub = self.create_subscription(Image, 'camera/image_raw', self.camera_callback, 10)
        self.lidar_sub = self.create_subscription(LaserScan, 'scan', self.lidar_callback, 10)

        # Publisher for fused data
        self.fused_pub = self.create_publisher(FusedSensorData, 'fused_sensor_data', 10)

        # Store latest sensor readings
        self.latest_imu = None
        self.latest_camera = None
        self.latest_lidar = None

    def imu_callback(self, msg):
        self.latest_imu = msg
        self.fuse_if_ready()

    def camera_callback(self, msg):
        self.latest_camera = msg
        self.fuse_if_ready()

    def lidar_callback(self, msg):
        self.latest_lidar = msg
        self.fuse_if_ready()

    def fuse_if_ready(self):
        if all([self.latest_imu, self.latest_camera, self.latest_lidar]):
            fused_data = self.perform_fusion()
            self.fused_pub.publish(fused_data)
```

### Control Hierarchy Pattern

Hierarchical control system with services for coordination:

```
High-Level Planner ──→ Service Request ──→ Low-Level Controller
        ↑                    │
        └──── Status Updates ←┘
```

### Distributed Coordination Pattern

Multiple nodes coordinating through a central coordinator:

```python
from example_interfaces.srv import Trigger
from std_msgs.msg import String

class CoordinatorNode(Node):
    def __init__(self):
        super().__init__('coordinator')

        # Service to coordinate actions
        self.coordination_service = self.create_service(
            Trigger, 'coordinate_action', self.coordinate_callback
        )

        # Publishers for individual subsystems
        self.arm_pub = self.create_publisher(String, 'arm_commands', 10)
        self.base_pub = self.create_publisher(String, 'base_commands', 10)
        self.head_pub = self.create_publisher(String, 'head_commands', 10)

    def coordinate_callback(self, request, response):
        # Send coordinated commands to different subsystems
        arm_cmd = String()
        arm_cmd.data = "move_to_position"
        self.arm_pub.publish(arm_cmd)

        base_cmd = String()
        base_cmd.data = "adjust_posture"
        self.base_pub.publish(base_cmd)

        head_cmd = String()
        head_cmd.data = "look_at_target"
        self.head_pub.publish(head_cmd)

        response.success = True
        response.message = "Coordinated action initiated"
        return response
```

## Error Handling and Fault Tolerance

### Service Client Error Handling

```python
async def call_service_with_retry(self, client, request, max_retries=3):
    for attempt in range(max_retries):
        try:
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

            if future.result() is not None:
                return future.result()
            else:
                self.get_logger().warning(f'Service call failed, attempt {attempt + 1}')
        except Exception as e:
            self.get_logger().error(f'Service call exception: {e}')

        if attempt < max_retries - 1:
            await asyncio.sleep(1.0)  # Wait before retry

    return None  # Return None if all retries failed
```

### Communication Timeout Handling

```python
def call_service_with_timeout(self, request, timeout_sec=5.0):
    future = self.client.call_async(request)

    # Use a timer to implement timeout
    timer = self.create_timer(timeout_sec, lambda: self.cancel_future(future))

    rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)

    if future.done():
        timer.cancel()
        return future.result()
    else:
        self.get_logger().error('Service call timed out')
        return None
```

## Performance Considerations

### Message Size Optimization

For humanoid robots with many joints and sensors:

```python
# Instead of publishing all joint states at once
# Consider publishing only changed values or using compression

from sensor_msgs.msg import JointState
import numpy as np

def optimize_joint_state(self, current_state, previous_state, threshold=0.01):
    """Only publish joints that have changed significantly"""
    optimized_state = JointState()

    for i, (curr_pos, prev_pos) in enumerate(zip(current_state.position, previous_state.position)):
        if abs(curr_pos - prev_pos) > threshold:
            optimized_state.name.append(current_state.name[i])
            optimized_state.position.append(curr_pos)
            if i < len(current_state.velocity):
                optimized_state.velocity.append(current_state.velocity[i])
            if i < len(current_state.effort):
                optimized_state.effort.append(current_state.effort[i])

    return optimized_state
```

### Communication Frequency Management

```python
class FrequencyManager:
    def __init__(self, node, frequency):
        self.node = node
        self.period = 1.0 / frequency
        self.last_time = self.node.get_clock().now()

    def should_publish(self):
        current_time = self.node.get_clock().now()
        time_diff = (current_time - self.last_time).nanoseconds / 1e9

        if time_diff >= self.period:
            self.last_time = current_time
            return True
        return False

# Usage in a publisher
freq_manager = FrequencyManager(self, 30.0)  # 30 Hz

def timer_callback(self):
    if freq_manager.should_publish():
        # Publish message
        self.publisher.publish(msg)
```

## Summary

In this chapter, we've explored the various communication patterns available in ROS 2, with a focus on services and how they integrate with other communication mechanisms. We've learned when to use different patterns, how to implement complex communication architectures, and how to apply these patterns specifically to humanoid robotics applications. Understanding these communication patterns is crucial for building robust, efficient, and maintainable robotic systems.

## Exercises

1. Design a service-based interface for a humanoid robot's walking controller
2. Implement a hybrid communication system that uses both topics and services
3. Create a fault-tolerant service client with retry logic
4. Design a coordination system for multiple robotic subsystems using appropriate communication patterns