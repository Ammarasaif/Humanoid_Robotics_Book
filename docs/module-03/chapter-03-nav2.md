---
title: "Chapter 03: Path Planning with Nav2"
description: Nav2 configuration for humanoid robots, bipedal movement, and navigation strategies
sidebar_label: "Chapter 03: Path Planning with Nav2"
---

# Chapter 03: Path Planning with Nav2

Navigation2 (Nav2) is the next-generation navigation framework for ROS 2, specifically designed to handle complex navigation tasks for mobile robots. This chapter focuses on configuring Nav2 for humanoid robots, with particular attention to bipedal movement strategies and specialized navigation approaches.

## Nav2 Architecture Overview

Nav2 follows a behavior tree-based architecture that provides flexibility and modularity:

- **Planners**: Global and local path planning algorithms
- **Controllers**: Robot motion control and trajectory following
- **Recovery Behaviors**: Actions to take when navigation fails
- **Sensors**: Integration with various sensor types for localization
- **Lifecycle Management**: Proper state management for navigation system

### Core Components

1. **Global Planner**: Computes the overall path from start to goal
2. **Local Planner**: Handles immediate obstacle avoidance and trajectory execution
3. **Controller**: Low-level robot motion control
4. **Behavior Tree**: Orchestrates the navigation system

## Nav2 Configuration for Humanoid Robots

Humanoid robots present unique challenges for navigation that require specialized configuration:

### Bipedal Movement Considerations

Unlike wheeled robots, humanoid robots have complex kinematics and dynamics:

```yaml
# Nav2 configuration for bipedal humanoid robot
bt_navigator:
  ros__parameters:
    use_sim_time: false
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    # Behavior tree for bipedal navigation
    default_nav_through_poses_bt_xml: /opt/ros/humble/share/nav2_bt_navigator/behavior_trees/nav_through_poses_bipedal.xml
    default_nav_to_pose_bt_xml: /opt/ros/humble/share/nav2_bt_navigator/behavior_trees/nav_to_pose_bipedal.xml

planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: false
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true
      # Bipedal-specific costmap considerations
      costmap_resolution: 0.05  # Finer resolution for precise foot placement
      costmap_inflation_radius: 0.8  # Account for robot's full body

controller_server:
  ros__parameters:
    use_sim_time: false
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.001
    min_theta_velocity_threshold: 0.001
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "nav2_mppi_controller/MppiController"
      # Bipedal-specific control parameters
      time_steps: 20
      control_horizon: 1.0
      # Adjust for bipedal dynamics
      max_linear_speed: 0.5  # Slower for stability
      max_angular_speed: 0.5
      # Foot placement constraints
      min_linear_speed: 0.05
      min_angular_speed: 0.05
```

### Costmap Configuration for Humanoids

The costmap needs to account for the humanoid's unique shape and movement:

```yaml
# Costmap configuration for humanoid robot
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 10.0
      publish_frequency: 5.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: false
      rolling_window: true
      width: 6
      height: 6
      resolution: 0.05  # High resolution for foot placement
      # Inflation for humanoid body
      plugins: ["obstacle_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        inflation_radius: 0.8  # Account for full body
        cost_scaling_factor: 3.0  # Higher cost scaling for safety
        # Bipedal-specific considerations
        footprint_padding: 0.1
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0  # Humanoid height consideration
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 10.0
          raytrace_min_range: 0.0
          obstacle_max_range: 8.0
          obstacle_min_range: 0.1

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: false
      rolling_window: false
      width: 200
      height: 200
      resolution: 0.05
      # Plugins for humanoid navigation
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        inflation_radius: 1.0  # Larger for humanoid safety
        cost_scaling_factor: 2.0
```

## Bipedal Movement Strategies

Humanoid robots require specialized movement strategies due to their unique locomotion:

### Walking Patterns

```cpp
// Example C++ implementation of bipedal walking pattern
#include <nav2_core/controller.hpp>
#include <geometry_msgs/msg/twist.hpp>

class BipedalController : public nav2_core::Controller
{
public:
    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
        std::string name, const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros,
        const std::shared_ptr<nav2_core::LifecycleNode> &) override
    {
        auto node = parent.lock();
        costmap_ = costmap_ros->getCostmap();
        tf_ = costmap_ros->getTfBuffer();

        // Configure bipedal-specific parameters
        node->get_parameter_or("max_step_length", max_step_length_, 0.3);
        node->get_parameter_or("step_height", step_height_, 0.05);
        node->get_parameter_or("walking_frequency", walking_frequency_, 2.0);
    }

    geometry_msgs::msg::Twist computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped & pose,
        const geometry_msgs::msg::Twist & velocity,
        nav2_core::GoalChecker * goal_checker) override
    {
        geometry_msgs::msg::Twist cmd_vel;

        // Implement bipedal-specific velocity computation
        // considering step constraints and balance
        cmd_vel.linear.x = calculateBipedalVelocity(pose, velocity);
        cmd_vel.angular.z = calculateAngularVelocity(pose, velocity);

        return cmd_vel;
    }

private:
    double calculateBipedalVelocity(
        const geometry_msgs::msg::PoseStamped & pose,
        const geometry_msgs::msg::Twist & velocity)
    {
        // Calculate velocity based on:
        // - Step length constraints
        // - Balance requirements
        // - Terrain analysis
        // - Gait stability
        return 0.0; // Placeholder implementation
    }

    double max_step_length_;
    double step_height_;
    double walking_frequency_;
    nav2_costmap_2d::Costmap2D * costmap_;
    tf2_ros::Buffer * tf_;
};
```

### Balance and Stability Considerations

Bipedal navigation must account for balance and stability:

```python
# Python example for balance-aware navigation
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu

class BalanceAwareController:
    def __init__(self):
        self.com_height = 0.8  # Center of mass height
        self.step_length = 0.3
        self.balance_threshold = 0.1  # Balance margin
        self.imu_data = None

    def compute_stable_velocity(self, target_pose, current_pose, imu_data):
        # Calculate target velocity with balance constraints
        target_vel = self.calculate_base_velocity(target_pose, current_pose)

        # Apply balance constraints based on IMU data
        if self.imu_data:
            adjusted_vel = self.apply_balance_constraints(
                target_vel,
                self.imu_data
            )
        else:
            adjusted_vel = target_vel

        return adjusted_vel

    def apply_balance_constraints(self, target_vel, imu_data):
        # Calculate current balance state
        roll, pitch = self.get_orientation(imu_data)

        # Adjust velocity based on balance state
        if abs(pitch) > self.balance_threshold:
            # Reduce velocity to maintain balance
            target_vel.linear.x *= 0.5
            target_vel.angular.z *= 0.3

        return target_vel

    def get_orientation(self, imu_data):
        # Extract orientation from IMU quaternion
        quaternion = [
            imu_data.orientation.x,
            imu_data.orientation.y,
            imu_data.orientation.z,
            imu_data.orientation.w
        ]

        # Convert to roll/pitch/yaw
        # Implementation would use quaternion to Euler conversion
        roll = 0.0  # Placeholder
        pitch = 0.0  # Placeholder

        return roll, pitch
```

## Advanced Navigation Strategies

### Terrain-Aware Navigation

Humanoid robots must adapt to various terrains:

```yaml
# Terrain-aware navigation configuration
terrain_analysis:
  ros__parameters:
    # Parameters for terrain classification
    step_height_threshold: 0.1
    slope_threshold: 0.3  # radians
    ground_clearance: 0.1
    # Footstep planning parameters
    footstep_planner:
      max_step_width: 0.3
      max_step_height: 0.15
      min_step_overlap: 0.05
      step_spacing: 0.3
```

### Multi-Modal Navigation

Humanoids may need to switch between different movement modes:

```cpp
// Example of multi-modal navigation controller
enum MovementMode {
    WALKING,
    STANDING,
    CLIMBING,
    SITTING
};

class MultiModalController {
public:
    MovementMode determineMovementMode(
        const geometry_msgs::msg::PoseStamped & goal,
        const nav2_costmap_2d::Costmap2D & costmap)
    {
        // Analyze terrain and goal to determine best movement mode
        if (isSteepSlope(goal, costmap)) {
            return CLIMBING;
        } else if (isObstacleTooHigh(goal, costmap)) {
            return SITTING;  // Go around
        } else {
            return WALKING;
        }
    }

private:
    bool isSteepSlope(const geometry_msgs::msg::PoseStamped & goal,
                     const nav2_costmap_2d::Costmap2D & costmap) {
        // Analyze elevation changes in path
        return false;  // Placeholder implementation
    }

    bool isObstacleTooHigh(const geometry_msgs::msg::PoseStamped & goal,
                          const nav2_costmap_2d::Costmap2D & costmap) {
        // Check for obstacles that require special handling
        return false;  // Placeholder implementation
    }
};
```

## Nav2 Integration with Isaac

Isaac provides enhanced perception capabilities that can improve Nav2 performance:

### Isaac-Enhanced Localization

```yaml
# Isaac-enhanced localization configuration
localization:
  ros__parameters:
    use_sim_time: false
    # Use Isaac's visual-inertial odometry
    odometry_source: "isaac_vio"
    # Isaac's 3D localization
    localization_method: "isaac_3d_localization"

isaac_vio:
  ros__parameters:
    # Visual-inertial odometry parameters
    camera_topic: "/camera/rgb/image_rect_color"
    imu_topic: "/imu/data"
    # Bipedal-specific parameters
    max_velocity: 1.0
    imu_weight: 0.7
    visual_weight: 0.3
```

### Isaac Perception for Navigation

```python
# Python example of Isaac perception integration with Nav2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class IsaacPerceptionIntegrator(Node):
    def __init__(self):
        super().__init__('isaac_perception_integrator')

        # Isaac perception subscribers
        self.image_sub = self.create_subscription(
            Image, '/camera/rgb/image_rect_color',
            self.image_callback, 10
        )
        self.pointcloud_sub = self.create_subscription(
            PointCloud2, '/camera/depth/points',
            self.pointcloud_callback, 10
        )

        # Navigation publishers
        self.path_pub = self.create_publisher(
            Path, '/global_plan', 10
        )

    def image_callback(self, msg):
        # Use Isaac's perception capabilities to enhance navigation
        # Detect obstacles, stairs, slopes, etc.
        pass

    def pointcloud_callback(self, msg):
        # Process 3D point cloud data for terrain analysis
        # Identify walkable surfaces, obstacles, steps
        terrain_analysis = self.analyze_terrain(msg)
        self.update_navigation_plan(terrain_analysis)
```

## Performance Optimization

### Computing Resource Management

Humanoid navigation requires careful resource management:

```yaml
# Resource-optimized Nav2 configuration
compute_budget:
  ros__parameters:
    # Reduce frequency for complex computations
    global_planner_frequency: 1.0  # Lower for complex humanoid planning
    local_planner_frequency: 10.0  # Higher for obstacle avoidance
    controller_frequency: 20.0     # Critical for balance

    # Memory management
    max_costmap_memory: 512  # MB
    trajectory_horizon: 2.0  # seconds

    # Bipedal-specific optimizations
    step_planning_enabled: true
    balance_check_frequency: 50.0  # High frequency for stability
```

## Troubleshooting and Debugging

### Common Issues

1. **Footstep Planning Failures**: Ensure proper costmap resolution and inflation
2. **Balance Loss During Navigation**: Adjust velocity limits and step constraints
3. **Localization Drift**: Improve sensor fusion and landmark detection
4. **Path Execution Failures**: Verify controller parameters and robot dynamics

### Debugging Tools

```bash
# Monitor navigation performance
ros2 run nav2_util lifecycle_bringup navigation

# Visualize navigation in RViz
ros2 run rviz2 rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view_bipedal.rviz

# Monitor navigation logs
ros2 launch nav2_bringup nav2_logging_launch.py
```

Nav2 provides a robust framework for humanoid robot navigation when properly configured for bipedal movement patterns and stability requirements. The combination of traditional navigation approaches with Isaac's perception capabilities enables sophisticated humanoid mobility.

## Navigation

- [← Chapter 02: Isaac ROS Fundamentals](./chapter-02-isaac-ros)
- [Chapter 04: Advanced Perception & Training](./chapter-04-advanced-perception) →