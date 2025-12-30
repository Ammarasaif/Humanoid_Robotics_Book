---
title: "Chapter 03: Robot Modeling and Sensor Simulation"
description: "URDF/SDF humanoids, LiDAR, depth cameras, IMUs, noise and latency simulation"
sidebar_label: "Chapter 03: Robot Modeling and Sensor Simulation"
---

# Chapter 03: Robot Modeling and Sensor Simulation

Accurate robot modeling and sensor simulation are crucial for creating effective digital twins. This chapter covers how to create realistic humanoid robot models with properly simulated sensors that behave similarly to their real-world counterparts.

## URDF and SDF for Humanoid Robots

### URDF (Unified Robot Description Format)

URDF is the standard format for describing robot models in ROS. For humanoid robots, it includes:

```xml
<?xml version="1.0"?>
<robot name="humanoid_robot">
  <!-- Base link -->
  <link name="base_link">
    <inertial>
      <mass value="5.0"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0"
               iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
    <visual>
      <geometry>
        <cylinder length="0.2" radius="0.1"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.2" radius="0.1"/>
      </geometry>
    </collision>
  </link>

  <!-- Joint connecting torso -->
  <joint name="base_torso_joint" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
  </joint>

  <link name="torso">
    <inertial>
      <mass value="10.0"/>
      <origin xyz="0 0 0.2"/>
      <inertia ixx="0.5" ixy="0.0" ixz="0.0"
               iyy="0.5" iyz="0.0" izz="0.5"/>
    </inertial>
    <visual>
      <geometry>
        <box size="0.3 0.2 0.4"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.4"/>
      </geometry>
    </collision>
  </link>
</robot>
```

### SDF (Simulation Description Format)

SDF is Gazebo's native format, which can include more simulation-specific parameters:

```xml
<sdf version="1.6">
  <model name="humanoid_robot">
    <link name="base_link">
      <inertial>
        <mass>5.0</mass>
        <inertia>
          <ixx>0.1</ixx> <ixy>0.0</ixy> <ixz>0.0</ixz>
          <iyy>0.1</iyy> <iyz>0.0</iyz> <izz>0.1</izz>
        </inertia>
      </inertial>
      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>0.1</radius>
            <length>0.2</length>
          </cylinder>
        </geometry>
      </visual>
      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>0.1</radius>
            <length>0.2</length>
          </cylinder>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
```

## LiDAR Simulation

### Creating LiDAR Sensors

LiDAR sensors are crucial for navigation and mapping:

```xml
<sensor name="lidar_sensor" type="ray">
  <pose>0 0 0.3 0 0 0</pose>
  <visualize>true</visualize>
  <update_rate>10</update_rate>
  <ray>
    <scan>
      <horizontal>
        <samples>360</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.1</min>
      <max>30.0</max>
      <resolution>0.01</resolution>
    </range>
  </ray>
  <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
    <ros>
      <namespace>/humanoid_robot</namespace>
      <remapping>~/out:=scan</remapping>
    </ros>
    <output_type>sensor_msgs/LaserScan</output_type>
  </plugin>
</sensor>
```

### LiDAR Noise Modeling

Real LiDAR sensors have noise characteristics:

```xml
<sensor name="lidar_sensor" type="ray">
  <!-- ... previous configuration ... -->
  <noise>
    <type>gaussian</type>
    <mean>0.0</mean>
    <stddev>0.01</stddev>  <!-- 1cm standard deviation -->
  </noise>
</sensor>
```

## Depth Camera Simulation

### Creating Depth Cameras

Depth cameras provide 3D perception capabilities:

```xml
<sensor name="depth_camera" type="depth">
  <pose>0.1 0 0.1 0 0 0</pose>
  <visualize>true</visualize>
  <update_rate>30</update_rate>
  <camera name="depth_cam">
    <horizontal_fov>1.047</horizontal_fov>  <!-- 60 degrees -->
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>10.0</far>
    </clip>
  </camera>
  <plugin name="camera_controller" filename="libgazebo_ros_openni_kinect.so">
    <ros>
      <namespace>/humanoid_robot</namespace>
      <remapping>~/rgb/image_raw:=rgb/image_raw</remapping>
      <remapping>~/depth/image_raw:=depth/image_raw</remapping>
      <remapping>~/points:=depth/points</remapping>
    </ros>
    <camera_name>depth_camera</camera_name>
    <frame_name>depth_camera_frame</frame_name>
    <baseline>0.2</baseline>
    <distortion_k1>0.0</distortion_k1>
    <distortion_k2>0.0</distortion_k2>
    <distortion_k3>0.0</distortion_k3>
    <distortion_t1>0.0</distortion_t1>
    <distortion_t2>0.0</distortion_t2>
  </plugin>
</sensor>
```

### Camera Noise and Distortion

Model real-world camera imperfections:

```xml
<sensor name="depth_camera" type="depth">
  <!-- ... previous configuration ... -->
  <camera>
    <!-- ... previous camera configuration ... -->
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.007</stddev>  <!-- Noise in pixels -->
    </noise>
  </camera>
</sensor>
```

## IMU Simulation

### Creating IMU Sensors

IMUs are essential for balance and orientation:

```xml
<sensor name="imu_sensor" type="imu">
  <pose>0 0 0.2 0 0 0</pose>
  <visualize>false</visualize>
  <update_rate>100</update_rate>
  <imu>
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.0017</stddev>  <!-- ~0.1 deg/s (1-sigma) -->
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.0017</stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.0017</stddev>
        </noise>
      </z>
    </angular_velocity>
    <linear_acceleration>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.017</stddev>  <!-- 1.7% of gravity -->
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.017</stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.017</stddev>
        </noise>
      </z>
    </linear_acceleration>
  </imu>
  <plugin name="imu_plugin" filename="libgazebo_ros_imu.so">
    <ros>
      <namespace>/humanoid_robot</namespace>
      <remapping>~/out:=imu/data</remapping>
    </ros>
    <frame_name>imu_link</frame_name>
    <body_name>torso</body_name>
    <update_rate>100</update_rate>
  </plugin>
</sensor>
```

## Sensor Noise and Latency Modeling

### Noise Models

Real sensors have various types of noise:

1. **Gaussian Noise**: Random variations following a normal distribution
2. **Bias**: Systematic offset from true values
3. **Drift**: Slow changes in sensor characteristics over time

### Latency Simulation

Model communication delays:

```xml
<sensor name="sensor_with_latency" type="camera">
  <!-- ... sensor configuration ... -->
  <plugin name="delayed_camera" filename="libgazebo_ros_camera.so">
    <ros>
      <namespace>/humanoid_robot</namespace>
    </ros>
    <update_rate>30</update_rate>
    <image_topic_name>image_raw</image_topic_name>
    <camera_info_topic_name>camera_info</camera_info_topic_name>
    <!-- Simulate network delay -->
    <sensor_latency_mean>0.02</sensor_latency_mean>  <!-- 20ms mean delay -->
    <sensor_latency_stddev>0.005</sensor_latency_stddev>  <!-- 5ms std dev -->
  </plugin>
</sensor>
```

## Sensor Fusion in Simulation

### Combining Multiple Sensors

Real robots use multiple sensors for robust perception:

```xml
<!-- Example: Combining IMU and camera data -->
<sensor name="sensor_fusion_node">
  <!-- This would be implemented in a ROS node that subscribes to
       multiple sensor topics and publishes fused estimates -->
</sensor>
```

### Creating a Complete Humanoid Model

Here's an example of a simplified humanoid model with multiple sensors:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Body links -->
  <link name="base_link"/>

  <link name="torso">
    <inertial>
      <mass value="10.0"/>
      <origin xyz="0 0 0.2"/>
      <inertia ixx="0.5" ixy="0.0" ixz="0.0"
               iyy="0.5" iyz="0.0" izz="0.5"/>
    </inertial>
    <visual>
      <geometry>
        <box size="0.3 0.2 0.4"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.4"/>
      </geometry>
    </collision>
  </link>

  <!-- Sensors -->
  <joint name="torso_imu_joint" type="fixed">
    <parent link="torso"/>
    <child link="imu_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
  </joint>

  <link name="imu_link">
    <inertial>
      <mass value="0.01"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0"
               iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>

  <!-- IMU sensor -->
  <gazebo reference="imu_link">
    <sensor name="imu_sensor" type="imu">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <visualize>false</visualize>
    </sensor>
  </gazebo>
</robot>
```

## Validation and Calibration

### Sensor Validation

Validate sensor models against real hardware:

1. **Static Testing**: Compare sensor readings in static conditions
2. **Dynamic Testing**: Compare responses to known motions
3. **Environmental Testing**: Test under various lighting/condition scenarios

### Calibration Procedures

Calibrate simulation parameters:

1. **Collect real data** from physical sensors
2. **Analyze noise characteristics** (mean, std dev, bias)
3. **Adjust simulation parameters** to match real behavior
4. **Validate** with different test scenarios

Accurate sensor simulation is essential for effective sim-to-real transfer, allowing control algorithms to be developed and tested in simulation before deployment on physical robots.

## Navigation

- [← Chapter 02: Physics Simulation with Gazebo](./chapter-02-gazebo-physics)
- [Chapter 04: High-Fidelity Simulation with Unity](./chapter-04-unity) →