---
sidebar_position: 5
---

# Chapter 5: URDF Fundamentals for Humanoid Robots

## Overview

This chapter introduces the Unified Robot Description Format (URDF), the standard XML-based format for representing robot models in ROS. We'll explore how URDF enables the description of humanoid robots, including their kinematic structure, visual appearance, and physical properties. Understanding URDF is essential for simulating, controlling, and visualizing humanoid robots in ROS-based systems.

## Learning Objectives

By the end of this chapter, you will:
- Understand the structure and components of URDF files
- Learn to create URDF descriptions for humanoid robots
- Master the definition of joints, links, and their properties
- Know how to include visual and collision properties in URDF
- Understand how URDF integrates with ROS tools and simulation environments
- Learn best practices for organizing complex humanoid robot models

## Introduction to URDF

### What is URDF?

The Unified Robot Description Format (URDF) is an XML-based format used to describe robots in ROS. It defines the physical and visual properties of a robot, including:
- Kinematic structure (links and joints)
- Visual appearance (meshes, colors, materials)
- Collision properties (shapes, bounds)
- Inertial properties (mass, center of mass, moments of inertia)

### Why URDF for Humanoid Robots?

Humanoid robots have complex kinematic structures with many degrees of freedom. URDF provides:
- **Standard representation**: Common format understood by ROS tools
- **Kinematic modeling**: Clear definition of joint relationships
- **Simulation compatibility**: Works with Gazebo and other simulators
- **Visualization**: Proper display in RViz and other tools
- **Motion planning**: Input for kinematic solvers and planners

## URDF Structure

### Basic URDF File

A minimal URDF file has the following structure:

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <!-- Links define rigid bodies -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="1 1 1"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="1 1 1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
  </link>

  <!-- Joints connect links -->
  <joint name="joint_name" type="revolute">
    <parent link="base_link"/>
    <child link="child_link"/>
    <origin xyz="0 0 1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="1"/>
  </joint>

  <link name="child_link">
    <!-- Child link definition -->
  </link>
</robot>
```

## Links: Defining Rigid Bodies

### Link Components

Each link in a URDF file can contain:

1. **Visual**: How the link appears in visualization tools
2. **Collision**: How the link interacts in physics simulations
3. **Inertial**: Physical properties for dynamics calculations

### Visual Properties

```xml
<link name="link_name">
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <!-- Can be box, cylinder, sphere, or mesh -->
      <box size="0.1 0.1 0.1"/>
      <!-- OR -->
      <cylinder radius="0.05" length="0.1"/>
      <!-- OR -->
      <sphere radius="0.05"/>
      <!-- OR -->
      <mesh filename="package://my_robot/meshes/link.stl"/>
    </geometry>
    <material name="red">
      <color rgba="1 0 0 1"/>
    </material>
  </visual>
</link>
```

### Collision Properties

```xml
<link name="link_name">
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <!-- Similar to visual but often simplified -->
      <box size="0.1 0.1 0.1"/>
    </geometry>
  </collision>
</link>
```

### Inertial Properties

```xml
<link name="link_name">
  <inertial>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <mass value="0.1"/>
    <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
  </inertial>
</link>
```

## Joints: Defining Connections

### Joint Types

URDF supports several joint types:

- **revolute**: Rotational joint with limited range
- **continuous**: Rotational joint without limits
- **prismatic**: Linear sliding joint with limits
- **fixed**: No movement (rigid connection)
- **floating**: 6-DOF movement (rarely used)
- **planar**: Movement on a plane (rarely used)

### Joint Definition

```xml
<joint name="joint_name" type="revolute">
  <parent link="parent_link_name"/>
  <child link="child_link_name"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  <dynamics damping="0.1" friction="0.0"/>
</joint>
```

### Common Joint Configurations for Humanoid Robots

**Revolute Joints for Limbs:**
```xml
<joint name="hip_joint" type="revolute">
  <parent link="torso"/>
  <child link="thigh"/>
  <origin xyz="0 0.1 -0.1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="2"/>
</joint>
```

**Spherical Joints (using multiple revolute joints):**
```xml
<!-- Shoulder complex with 3 DOF -->
<joint name="shoulder_yaw" type="revolute">
  <parent link="torso"/>
  <child link="upper_arm"/>
  <origin xyz="0.2 0 0.1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-2.0" upper="2.0" effort="50" velocity="2"/>
</joint>
```

## Humanoid Robot Structure

### Typical Humanoid Kinematic Chain

A humanoid robot typically has the following structure:

```
base_footprint
    └── base_link (pelvis)
        ├── torso
        │   ├── head
        │   ├── left_shoulder
        │   │   ├── left_upper_arm
        │   │   ├── left_lower_arm
        │   │   └── left_hand
        │   ├── right_shoulder
        │   │   ├── right_upper_arm
        │   │   ├── right_lower_arm
        │   │   └── right_hand
        │   ├── left_hip
        │   │   ├── left_upper_leg
        │   │   ├── left_lower_leg
        │   │   └── left_foot
        │   └── right_hip
        │       ├── right_upper_leg
        │       ├── right_lower_leg
        │       └── right_foot
```

### Complete Humanoid URDF Example

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Base/Fixed link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.1 0.3"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.1 0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.2" ixy="0" ixz="0" iyy="0.2" iyz="0" izz="0.2"/>
    </inertial>
  </link>

  <!-- Torso Joint -->
  <joint name="torso_joint" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.25"/>
  </joint>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.3"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10" velocity="1"/>
  </joint>

  <!-- Left Arm -->
  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.005"/>
    </inertial>
  </link>

  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.2 0.1 0.1" rpy="0 0 0.785"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="2"/>
  </joint>
</robot>
```

## Advanced URDF Features

### Using Xacro for Complex Models

Xacro (XML Macros) allows parameterization and reuse in URDF:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid_with_xacro">

  <!-- Define properties -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="arm_length" value="0.3" />
  <xacro:property name="arm_radius" value="0.05" />

  <!-- Macro for creating arm links -->
  <xacro:macro name="arm_link" params="name parent xyz rpy length radius">
    <link name="${name}">
      <visual>
        <geometry>
          <cylinder length="${length}" radius="${radius}"/>
        </geometry>
        <origin xyz="0 0 ${length/2}" rpy="0 0 0"/>
      </visual>
      <collision>
        <geometry>
          <cylinder length="${length}" radius="${radius}"/>
        </geometry>
        <origin xyz="0 0 ${length/2}" rpy="0 0 0"/>
      </collision>
      <inertial>
        <mass value="1.0"/>
        <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.005"/>
      </inertial>
    </link>

    <joint name="${name}_joint" type="revolute">
      <parent link="${parent}"/>
      <child link="${name}"/>
      <origin xyz="${xyz}" rpy="${rpy}"/>
      <axis xyz="0 1 0"/>
      <limit lower="-1.57" upper="1.57" effort="50" velocity="2"/>
    </joint>
  </xacro:macro>

  <!-- Use the macro to create arms -->
  <xacro:arm_link name="left_arm" parent="torso" xyz="0.2 0.1 0.1" rpy="0 0 0.785" length="${arm_length}" radius="${arm_radius}"/>

</robot>
```

### Including External Files

```xml
<!-- Include other URDF files -->
<xacro:include filename="$(find my_robot_description)/urdf/arms.urdf.xacro" />
<xacro:include filename="$(find my_robot_description)/urdf/legs.urdf.xacro" />

<!-- Use macros from included files -->
<xacro:left_arm prefix="l"/>
<xacro:right_arm prefix="r"/>
```

## URDF Tools and Visualization

### URDF to DH Parameters

Use ROS tools to extract kinematic information:

```bash
# Check URDF validity
check_urdf /path/to/robot.urdf

# Generate a tree structure of the robot
urdf_to_graphiz /path/to/robot.urdf
```

### Visualization in RViz

```xml
<!-- URDF is automatically visualized in RViz when published -->
<!-- Make sure to set up robot_state_publisher -->
```

### Robot State Publisher

```xml
<!-- In launch file -->
<node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher">
  <param name="robot_description" command="$(find xacro)/xacro $(find my_robot_description)/urdf/robot.urdf.xacro" />
</node>
```

## Best Practices for Humanoid URDF

### 1. Proper Mass Distribution

Ensure realistic inertial properties for stable simulation:

```xml
<!-- Good: Realistic inertial values -->
<inertial>
  <mass value="2.5"/>
  <origin xyz="0 0 0.05" rpy="0 0 0"/>
  <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.002"/>
</inertial>
```

### 2. Collision Optimization

Use simplified geometries for collision detection:

```xml
<!-- Complex visual shape -->
<visual>
  <mesh filename="package://my_robot/meshes/detailed_hand.stl"/>
</visual>

<!-- Simplified collision shape -->
<collision>
  <geometry>
    <box size="0.1 0.05 0.08"/>
  </geometry>
</collision>
```

### 3. Joint Limitations

Set realistic joint limits based on mechanical constraints:

```xml
<!-- Human-like joint limits -->
<joint name="elbow_joint" type="revolute">
  <limit lower="0" upper="2.356" effort="50" velocity="2"/>
  <!-- Elbow can only bend in one direction (0 to 135 degrees) -->
</joint>
```

### 4. Coordinate Frame Conventions

Follow ROS coordinate frame conventions:
- X: Forward
- Y: Left
- Z: Up

### 5. Organizing Complex Models

Split large URDF files into manageable components:

```
urdf/
├── robot.urdf.xacro          # Main file
├── materials.urdf.xacro      # Material definitions
├── common_properties.urdf.xacro # Shared properties
├── arms.urdf.xacro           # Arm definitions
├── legs.urdf.xacro           # Leg definitions
└── torso.urdf.xacro          # Torso definitions
```

## Integration with ROS Ecosystem

### Robot State Publisher

The robot_state_publisher node uses URDF to publish transforms between robot links:

```python
# In your launch file
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_path = get_package_share_directory('my_robot_description')
    urdf_file = os.path.join(pkg_path, 'urdf', 'robot.urdf.xacro')

    robot_description = {'robot_description': Command(['xacro ', urdf_file])}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    return LaunchDescription([
        node_robot_state_publisher
    ])
```

### TF Frames

URDF automatically creates TF frames for each link, enabling spatial relationships:

```python
import rclpy
from tf2_ros import TransformListener
from rclpy.node import Node

class TFExample(Node):
    def __init__(self):
        super().__init__('tf_example')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def get_transform(self, target_frame, source_frame):
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame, source_frame, rclpy.time.Time())
            return transform
        except TransformException as ex:
            self.get_logger().info(f'Could not transform {target_frame} to {source_frame}: {ex}')
            return None
```

## Common Pitfalls and Troubleshooting

### 1. Invalid URDF

Common errors include:
- Non-physical inertial values (negative masses, non-positive definite inertia matrices)
- Incorrect joint connections (disconnected links)
- Missing parent-child relationships

### 2. Simulation Instability

Caused by:
- Unrealistic inertial values
- Joint limits that are too permissive
- Insufficient damping in joint dynamics

### 3. Visualization Issues

- Check that mesh files exist and are accessible
- Verify that all links have proper visual/collision properties
- Ensure transforms are being published by robot_state_publisher

## Summary

In this chapter, we've explored the Unified Robot Description Format (URDF) and its critical role in describing humanoid robots in ROS. We've learned how to structure URDF files, define links and joints, and create complex kinematic chains typical of humanoid robots. We've also covered advanced features like Xacro for parameterization and best practices for creating realistic, stable robot models. URDF forms the foundation for robot simulation, visualization, and control in the ROS ecosystem.

## Exercises

1. Create a simple humanoid URDF with at least 10 links and appropriate joints
2. Use Xacro to parameterize your URDF and create multiple variants
3. Add realistic inertial properties to your robot model
4. Validate your URDF using ROS tools and visualize it in RViz