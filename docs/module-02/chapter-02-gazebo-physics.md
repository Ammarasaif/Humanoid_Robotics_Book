---
title: "Chapter 02: Physics Simulation with Gazebo"
description: "Rigid body dynamics, gravity, friction, collisions, world setup, and debugging in Gazebo"
sidebar_label: "Chapter 02: Physics Simulation with Gazebo"
---

# Chapter 02: Physics Simulation with Gazebo

Gazebo is a powerful physics-based simulation environment that provides realistic simulation of robots in complex environments. It's particularly well-suited for testing robot dynamics, sensors, and control algorithms before deployment on physical robots.

## Understanding Gazebo's Physics Engine

Gazebo uses the Open Dynamics Engine (ODE), Bullet Physics, or Simbody as its underlying physics engines. These engines handle:

- **Rigid Body Dynamics**: How objects move and interact based on forces and torques
- **Collision Detection**: Identifying when objects make contact
- **Contact Processing**: Calculating the resulting forces when objects collide
- **Joint Simulation**: Modeling different types of mechanical joints

## Setting Up Rigid Body Dynamics

### Mass and Inertia Properties

For accurate simulation, each link in your robot model needs proper mass and inertia properties:

```xml
<link name="link_name">
  <inertial>
    <mass value="1.0"/>
    <inertia ixx="0.01" ixy="0.0" ixz="0.0"
             iyy="0.01" iyz="0.0" izz="0.01"/>
  </inertial>
  <visual>
    <!-- Visual properties -->
  </visual>
  <collision>
    <!-- Collision properties -->
  </collision>
</link>
```

### Inertial Considerations

The inertia tensor describes how mass is distributed in a rigid body. For common shapes:
- **Box**: `ixx = m*(h² + d²)/12`, `iyy = m*(w² + d²)/12`, `izz = m*(w² + h²)/12`
- **Cylinder**: `ixx = m*(3*r² + h²)/12`, `izz = m*r²/2`
- **Sphere**: `ixx = iyy = izz = 2*m*r²/5`

Where m=mass, w=width, h=height, d=depth, r=radius, h=height.

## Configuring Gravity and Friction

### Gravity Settings

Gravity is typically set in the world file:

```xml
<sdf version="1.6">
  <world name="default">
    <gravity>0 0 -9.8</gravity>
    <!-- Other world properties -->
  </world>
</sdf>
```

For humanoid robots, ensure gravity matches Earth's standard (9.8 m/s²) unless simulating different environments.

### Friction Models

Gazebo supports several friction models:

```xml
<collision name="collision">
  <surface>
    <friction>
      <ode>
        <mu>1.0</mu>        <!-- Static friction coefficient -->
        <mu2>1.0</mu2>      <!-- Secondary friction coefficient -->
        <slip1>0.0</slip1>  <!-- Primary slip coefficient -->
        <slip2>0.0</slip2>  <!-- Secondary slip coefficient -->
      </ode>
    </friction>
  </surface>
</collision>
```

## Collision Detection and Configuration

### Collision Shapes

Choose appropriate collision shapes based on your needs:

- **Box**: Fastest computation, good for rectangular objects
- **Cylinder**: Good for wheels, limbs
- **Sphere**: Fastest for round objects
- **Mesh**: Most accurate but computationally expensive

### Contact Parameters

Fine-tune contact behavior:

```xml
<collision name="collision">
  <surface>
    <contact>
      <ode>
        <soft_cfm>0</soft_cfm>     <!-- Constraint Force Mixing -->
        <soft_erp>0.2</soft_erp>   <!-- Error Reduction Parameter -->
        <kp>1e+13</kp>            <!-- Spring stiffness -->
        <kd>1.0</kd>              <!-- Damping coefficient -->
        <max_vel>100.0</max_vel>   <!-- Maximum contact correction velocity -->
        <min_depth>0.0</min_depth> <!-- Minimum contact depth -->
      </ode>
    </contact>
  </surface>
</collision>
```

## World Setup and Configuration

### Creating Custom Worlds

World files define the simulation environment:

```xml
<sdf version="1.6">
  <world name="my_world">
    <!-- Include models -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Define static objects -->
    <model name="table">
      <pose>2 0 0 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 0.8</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 0.8</size>
            </box>
          </geometry>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

### Physics Parameters

Configure global physics settings:

```xml
<world name="default">
  <physics type="ode">
    <max_step_size>0.001</max_step_size>  <!-- Simulation time step -->
    <real_time_factor>1.0</real_time_factor>  <!-- Real-time simulation speed -->
    <real_time_update_rate>1000.0</real_time_update_rate>  <!-- Updates per second -->
  </physics>
</world>
```

## Debugging Physics Issues

### Common Physics Problems

1. **Robot falls through the ground**: Check collision geometry and ground plane configuration
2. **Unstable joints**: Verify joint limits, damping, and stiffness parameters
3. **Excessive jittering**: Adjust physics time step and contact parameters
4. **Inconsistent behavior**: Check for missing mass properties or incorrect inertia

### Debugging Tools

Use Gazebo's built-in tools:

```bash
# Launch Gazebo with verbose output
gazebo -v 4 world_file.world

# Enable contact visualization
# In Gazebo GUI: View → Contacts
```

### Physics Parameter Tuning

Start with conservative parameters and gradually adjust:

```xml
<physics type="ode">
  <max_step_size>0.001</max_step_size>  <!-- Start with small steps -->
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000.0</real_time_update_rate>
  <max_contacts>20</max_contacts>  <!-- Limit contacts per collision -->
</physics>
```

## Performance Optimization

### Simulation Speed

Balance accuracy with performance:

- **Time Step**: Smaller steps = more accurate but slower
- **Real-time Factor**: 1.0 = real-time, `<1.0` = slower, `>1.0` = faster
- **Update Rate**: Higher rates = more accurate but more CPU usage

### Model Simplification

For performance-critical simulations:

- Use simpler collision geometries (boxes instead of meshes)
- Reduce the number of complex joints
- Optimize URDF/SDF models for simulation

Gazebo provides a robust foundation for physics-based simulation of humanoid robots, enabling thorough testing of control algorithms and robot behaviors before real-world deployment.

## Navigation

- [← Chapter 01: Digital Twins in Physical AI](./chapter-01-digital-twins)
- [Chapter 03: Robot Modeling and Sensor Simulation](./chapter-03-sensors) →