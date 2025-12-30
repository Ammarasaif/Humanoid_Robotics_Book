---
title: "Chapter 04: Simulated Robot Execution"
description: "Executing robot commands in simulation with path planning, obstacle navigation, and object recognition"
sidebar_label: "Chapter 04: Simulated Robot Execution"
---

# Chapter 04: Simulated Robot Execution

This chapter covers executing robot commands in simulation with path planning, obstacle navigation, and object recognition.

## Path Planning Algorithms

Path planning algorithms are essential for autonomous robot navigation in simulated environments.

### Overview of Path Planning

Path planning is the process of finding a valid and optimal path from a start position to a goal position while avoiding obstacles. In humanoid robotics, path planning must account for the robot's physical constraints and bipedal nature.

### Types of Path Planning Algorithms

1. **Global Path Planning**: Computes a path based on a complete map of the environment
2. **Local Path Planning**: Adjusts the path in real-time based on immediate surroundings
3. **Any-angle Path Planning**: Finds paths that are not constrained to a grid
4. **Multi-constraint Path Planning**: Considers multiple factors like energy, safety, and time

### Common Path Planning Algorithms

#### A* Algorithm

A* is a popular graph traversal algorithm that finds the shortest path using a heuristic function:

```python
import heapq
from typing import List, Tuple, Dict, Set
import numpy as np

class AStarPlanner:
    def __init__(self, grid: np.ndarray):
        """
        Initialize A* planner with occupancy grid
        0 = free space, 1 = obstacle
        """
        self.grid = grid
        self.height, self.width = grid.shape

    def heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """
        Calculate heuristic distance (Manhattan distance)
        """
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def get_neighbors(self, pos: Tuple[int, int]) -> List[Tuple[int, int]]:
        """
        Get valid neighbors for a position
        """
        x, y = pos
        neighbors = []

        # 8-connected neighborhood (including diagonals)
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue  # Skip current position

                nx, ny = x + dx, y + dy

                # Check bounds
                if 0 <= nx < self.width and 0 <= ny < self.height:
                    # Check if not an obstacle
                    if self.grid[ny, nx] == 0:
                        neighbors.append((nx, ny))

        return neighbors

    def plan_path(self, start: Tuple[int, int], goal: Tuple[int, int]) -> List[Tuple[int, int]]:
        """
        Plan path using A* algorithm
        """
        # Priority queue: (f_score, position)
        open_set = [(0, start)]
        heapq.heapify(open_set)

        # Costs
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        # Parents for path reconstruction
        came_from = {}

        while open_set:
            current_f, current = heapq.heappop(open_set)

            # Goal reached
            if current == goal:
                return self._reconstruct_path(came_from, current)

            for neighbor in self.get_neighbors(current):
                # Calculate tentative g_score
                tentative_g = g_score[current] + 1  # Assuming uniform cost

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    # This path to neighbor is better than any previous one
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic(neighbor, goal)

                    # Add to open set if not already there
                    if neighbor not in [item[1] for item in open_set]:
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))

        # No path found
        return []

    def _reconstruct_path(self, came_from: Dict, current: Tuple[int, int]) -> List[Tuple[int, int]]:
        """
        Reconstruct path from came_from dictionary
        """
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)

        path.reverse()
        return path
```

#### Dijkstra's Algorithm

Dijkstra's algorithm finds the shortest path without using a heuristic function:

```python
import heapq
from typing import List, Tuple, Dict

class DijkstraPlanner:
    def __init__(self, grid: np.ndarray):
        self.grid = grid
        self.height, self.width = grid.shape

    def get_neighbors(self, pos: Tuple[int, int]) -> List[Tuple[int, int]]:
        """
        Get valid neighbors for a position
        """
        x, y = pos
        neighbors = []

        # 4-connected neighborhood (up, down, left, right)
        for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
            nx, ny = x + dx, y + dy

            if 0 <= nx < self.width and 0 <= ny < self.height and self.grid[ny, nx] == 0:
                neighbors.append((nx, ny))

        return neighbors

    def plan_path(self, start: Tuple[int, int], goal: Tuple[int, int]) -> List[Tuple[int, int]]:
        """
        Plan path using Dijkstra's algorithm
        """
        # Priority queue: (distance, position)
        pq = [(0, start)]
        heapq.heapify(pq)

        # Distances
        distances = {start: 0}
        # Parents for path reconstruction
        came_from = {}

        while pq:
            current_dist, current = heapq.heappop(pq)

            # Goal reached
            if current == goal:
                return self._reconstruct_path(came_from, current)

            # If we've already found a shorter path to this node, skip
            if current_dist > distances.get(current, float('inf')):
                continue

            for neighbor in self.get_neighbors(current):
                new_dist = current_dist + 1  # Uniform cost

                if new_dist < distances.get(neighbor, float('inf')):
                    distances[neighbor] = new_dist
                    came_from[neighbor] = current
                    heapq.heappush(pq, (new_dist, neighbor))

        # No path found
        return []

    def _reconstruct_path(self, came_from: Dict, current: Tuple[int, int]) -> List[Tuple[int, int]]:
        """
        Reconstruct path from came_from dictionary
        """
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)

        path.reverse()
        return path
```

#### RRT (Rapidly-exploring Random Trees)

RRT is useful for high-dimensional spaces and complex environments:

```python
import numpy as np
from typing import List, Tuple
import random

class RRTPlanner:
    def __init__(self, grid: np.ndarray, max_iterations: int = 1000, step_size: float = 1.0):
        self.grid = grid
        self.height, self.width = grid.shape
        self.max_iterations = max_iterations
        self.step_size = step_size

    def is_valid_position(self, pos: Tuple[float, float]) -> bool:
        """
        Check if position is valid (not in obstacle)
        """
        x, y = int(pos[0]), int(pos[1])
        if 0 <= x < self.width and 0 <= y < self.height:
            return self.grid[y, x] == 0
        return False

    def distance(self, pos1: Tuple[float, float], pos2: Tuple[float, float]) -> float:
        """
        Calculate Euclidean distance between two positions
        """
        return np.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)

    def plan_path(self, start: Tuple[float, float], goal: Tuple[float, float]) -> List[Tuple[float, float]]:
        """
        Plan path using RRT algorithm
        """
        # Tree structure: node -> parent
        tree = {start: None}
        nodes = [start]

        for _ in range(self.max_iterations):
            # Sample random point
            rand_point = (
                random.uniform(0, self.width),
                random.uniform(0, self.height)
            )

            # If random point is in obstacle, try again
            if not self.is_valid_position(rand_point):
                continue

            # Find nearest node in tree
            nearest_node = min(nodes, key=lambda n: self.distance(n, rand_point))

            # Create new node in direction of random point
            direction = (rand_point[0] - nearest_node[0], rand_point[1] - nearest_node[1])
            length = np.sqrt(direction[0]**2 + direction[1]**2)

            if length == 0:
                continue

            # Normalize direction and scale by step size
            direction = (direction[0] / length, direction[1] / length)
            new_node = (
                nearest_node[0] + direction[0] * self.step_size,
                nearest_node[1] + direction[1] * self.step_size
            )

            # Check if new node is valid
            if self.is_valid_position(new_node):
                tree[new_node] = nearest_node
                nodes.append(new_node)

                # Check if goal is reached
                if self.distance(new_node, goal) < self.step_size:
                    # Connect to goal if possible
                    if self.is_line_free(new_node, goal):
                        tree[goal] = new_node
                        return self._reconstruct_path(tree, start, goal)

        # If max iterations reached without finding path to goal
        # Try to find path to closest point to goal
        closest_node = min(nodes, key=lambda n: self.distance(n, goal))
        return self._reconstruct_path(tree, start, closest_node)

    def is_line_free(self, start: Tuple[float, float], end: Tuple[float, float]) -> bool:
        """
        Check if line between start and end is free of obstacles
        """
        # Sample points along the line
        steps = int(self.distance(start, end) / 0.5)  # Sample every 0.5 units
        for i in range(steps + 1):
            t = i / steps
            x = start[0] + t * (end[0] - start[0])
            y = start[1] + t * (end[1] - start[1])

            if not self.is_valid_position((x, y)):
                return False

        return True

    def _reconstruct_path(self, tree: Dict, start: Tuple[float, float],
                         goal: Tuple[float, float]) -> List[Tuple[float, float]]:
        """
        Reconstruct path from tree
        """
        path = []
        current = goal

        while current is not None:
            path.append(current)
            current = tree.get(current)

        path.reverse()
        return path
```

### Path Planning for Humanoid Robots

Humanoid robots have specific constraints that affect path planning:

1. **Bipedal Locomotion**: Paths must account for walking patterns and balance
2. **Foot Placement**: Need to consider where feet can be placed safely
3. **Center of Mass**: Paths should maintain stable center of mass movement
4. **Joint Limits**: Movement must respect joint angle constraints
5. **Dynamic Balance**: Consider the robot's ability to maintain balance during movement

### Integration with Simulation

Path planning algorithms can be integrated with simulation environments:

```python
class SimulationPathPlanner:
    def __init__(self, sim_env):
        self.sim_env = sim_env
        self.astar_planner = AStarPlanner(sim_env.get_occupancy_grid())

    def plan_and_execute(self, start_pose, goal_pose):
        """
        Plan path and execute in simulation
        """
        # Convert poses to grid coordinates
        start_grid = self.sim_env.pose_to_grid(start_pose)
        goal_grid = self.sim_env.pose_to_grid(goal_pose)

        # Plan path
        path = self.astar_planner.plan_path(start_grid, goal_grid)

        if not path:
            print("No path found!")
            return False

        # Execute path in simulation
        for i, (x, y) in enumerate(path):
            # Convert grid coordinates back to world coordinates
            world_pose = self.sim_env.grid_to_pose((x, y))

            # Move robot to next position
            self.sim_env.move_robot_to(world_pose)

            # Check for dynamic obstacles
            if self.sim_env.check_dynamic_obstacles():
                print("Dynamic obstacle detected, replanning...")
                return self.replan_path(start_pose, goal_pose, path[i:])

        return True

    def replan_path(self, start_pose, goal_pose, remaining_path):
        """
        Replan path when encountering dynamic obstacles
        """
        # Check if remaining path is still valid
        for i, pos in enumerate(remaining_path):
            if self.sim_env.is_occupied(pos):
                # Replan from current position
                current_pose = self.sim_env.get_robot_pose()
                return self.plan_and_execute(current_pose, goal_pose)

        # If remaining path is clear, continue with it
        return True
```

These algorithms form the foundation of autonomous navigation in simulated environments, allowing humanoid robots to navigate complex spaces safely and efficiently.

## Obstacle Navigation Techniques

Robots must be able to navigate around obstacles in their environment to reach their destinations safely.

### Types of Obstacle Navigation

1. **Global Navigation**: Plan a complete path avoiding known static obstacles
2. **Local Navigation**: React to unknown or dynamic obstacles in real-time
3. **Reactive Navigation**: Immediate response to sensor input
4. **Predictive Navigation**: Anticipate and plan for moving obstacles

### Common Obstacle Navigation Algorithms

#### Vector Field Histogram (VFH)

VFH is a local navigation technique that uses a histogram of obstacles to determine the best direction:

```python
import numpy as np
from typing import List, Tuple
import math

class VectorFieldHistogram:
    def __init__(self, robot_radius: float = 0.5, sensor_range: float = 3.0):
        self.robot_radius = robot_radius
        self.sensor_range = sensor_range
        self.sector_count = 72  # 5-degree sectors
        self.sector_width = 2 * math.pi / self.sector_count

    def create_histogram(self, laser_scan: List[float], goal_direction: float) -> np.ndarray:
        """
        Create a histogram from laser scan data
        """
        histogram = np.zeros(self.sector_count)

        # Process laser scan data
        for i, distance in enumerate(laser_scan):
            if distance < self.sensor_range:
                # Calculate which sector this measurement belongs to
                angle = i * (2 * math.pi / len(laser_scan)) - math.pi
                sector = int((angle + math.pi) / self.sector_width)

                if 0 <= sector < self.sector_count:
                    # Higher values indicate more obstacles in that direction
                    histogram[sector] += (self.sensor_range - distance) / self.sensor_range

        return histogram

    def find_safe_directions(self, histogram: np.ndarray, threshold: float = 0.3) -> List[int]:
        """
        Find sectors with fewer obstacles than the threshold
        """
        safe_sectors = []
        for i, value in enumerate(histogram):
            if value < threshold:
                safe_sectors.append(i)

        return safe_sectors

    def select_direction(self, histogram: np.ndarray, goal_direction: float) -> float:
        """
        Select the best direction based on goal direction and obstacle histogram
        """
        safe_sectors = self.find_safe_directions(histogram)

        if not safe_sectors:
            # No safe direction found, return current heading
            return goal_direction

        # Calculate which sector the goal direction corresponds to
        goal_sector = int((goal_direction + math.pi) / self.sector_width)

        # Find the safe sector closest to the goal direction
        best_sector = min(safe_sectors, key=lambda s: min(abs(s - goal_sector),
                                                          self.sector_count - abs(s - goal_sector)))

        # Convert sector back to angle
        best_angle = best_sector * self.sector_width - math.pi
        return best_angle

    def navigate(self, laser_scan: List[float], goal_direction: float) -> float:
        """
        Main navigation function
        """
        histogram = self.create_histogram(laser_scan, goal_direction)
        selected_direction = self.select_direction(histogram, goal_direction)
        return selected_direction
```

#### Dynamic Window Approach (DWA)

DWA is a local path planning algorithm that considers robot dynamics:

```python
import numpy as np
from typing import Tuple

class DynamicWindowApproach:
    def __init__(self, max_vel: float = 1.0, max_ang_vel: float = 1.0,
                 max_acc: float = 0.5, max_ang_acc: float = 0.5):
        self.max_vel = max_vel
        self.max_ang_vel = max_ang_acc
        self.max_acc = max_acc
        self.max_ang_acc = max_ang_acc
        self.dt = 0.1  # time step
        self.predict_time = 1.5  # prediction horizon
        self.to_goal_cost_gain = 0.15
        self.speed_cost_gain = 1.0
        self.obstacle_cost_gain = 1.0

    def dynamic_window(self, state: Tuple[float, float, float, float, float]) -> Tuple[float, float, float, float]:
        """
        Calculate dynamic window based on current state and constraints
        state: (x, y, theta, velocity, angular_velocity)
        """
        x, y, theta, vel, ang_vel = state

        # Dynamic window based on robot's capability
        vs = [-self.max_vel, self.max_vel, -self.max_ang_vel, self.max_ang_vel]

        # Dynamic window based on acceleration constraints
        vd = [
            vel - self.max_acc * self.dt,
            vel + self.max_acc * self.dt,
            ang_vel - self.max_ang_acc * self.dt,
            ang_vel + self.max_ang_acc * self.dt
        ]

        # Actual dynamic window
        dw = [
            max(vs[0], vd[0]),
            min(vs[1], vd[1]),
            max(vs[2], vd[2]),
            min(vs[3], vd[3])
        ]

        return dw

    def simulate_trajectory(self, state: Tuple[float, float, float, float, float],
                           vel: float, ang_vel: float) -> np.ndarray:
        """
        Simulate trajectory for given velocity and angular velocity
        """
        x, y, theta, _, _ = state
        trajectory = []
        time = 0

        while time < self.predict_time:
            x += vel * math.cos(theta) * self.dt
            y += vel * math.sin(theta) * self.dt
            theta += ang_vel * self.dt
            trajectory.append([x, y, theta, vel, ang_vel])
            time += self.dt

        return np.array(trajectory)

    def to_goal_cost(self, trajectory: np.ndarray, goal: Tuple[float, float]) -> float:
        """
        Calculate cost based on distance to goal
        """
        goal_x, goal_y = goal
        end_x, end_y = trajectory[-1][0], trajectory[-1][1]
        distance = math.sqrt((end_x - goal_x)**2 + (end_y - goal_y)**2)
        return distance

    def speed_cost(self, trajectory: np.ndarray) -> float:
        """
        Calculate cost based on speed (prefer higher speeds)
        """
        vel = trajectory[-1][3]
        return self.max_vel - vel

    def obstacle_cost(self, trajectory: np.ndarray, obstacles: List[Tuple[float, float]],
                     robot_radius: float = 0.5) -> float:
        """
        Calculate cost based on proximity to obstacles
        """
        min_dist = float('inf')

        for point in trajectory:
            x, y = point[0], point[1]
            for obs_x, obs_y in obstacles:
                dist = math.sqrt((x - obs_x)**2 + (y - obs_y)**2)
                min_dist = min(min_dist, dist)

        if min_dist == float('inf'):
            return float('inf')

        # Return high cost if trajectory gets too close to obstacles
        return 1.0 / min_dist if min_dist != 0 else float('inf')

    def navigate(self, state: Tuple[float, float, float, float, float],
                 goal: Tuple[float, float], obstacles: List[Tuple[float, float]]) -> Tuple[float, float]:
        """
        Calculate best velocity and angular velocity
        """
        dw = self.dynamic_window(state)

        best_vel = 0.0
        best_ang_vel = 0.0
        min_cost = float('inf')

        # Sample velocities in the dynamic window
        v_samples = np.linspace(dw[0], dw[1], 10)
        w_samples = np.linspace(dw[2], dw[3], 10)

        for vel in v_samples:
            for ang_vel in w_samples:
                # Simulate trajectory
                trajectory = self.simulate_trajectory(state, vel, ang_vel)

                # Calculate costs
                goal_cost = self.to_goal_cost(trajectory, goal)
                speed_cost = self.speed_cost(trajectory)
                obs_cost = self.obstacle_cost(trajectory, obstacles)

                # Total cost
                total_cost = (self.to_goal_cost_gain * goal_cost +
                             self.speed_cost_gain * speed_cost +
                             self.obstacle_cost_gain * obs_cost)

                if total_cost < min_cost:
                    min_cost = total_cost
                    best_vel = vel
                    best_ang_vel = ang_vel

        return best_vel, best_ang_vel
```

#### Potential Field Method

The potential field method treats the goal as an attractive force and obstacles as repulsive forces:

```python
import numpy as np
from typing import List, Tuple

class PotentialFieldNavigator:
    def __init__(self, attractive_gain: float = 1.0, repulsive_gain: float = 2.0,
                 obstacle_influence_radius: float = 2.0):
        self.attractive_gain = attractive_gain
        self.repulsive_gain = repulsive_gain
        self.obstacle_influence_radius = obstacle_influence_radius

    def attractive_force(self, robot_pos: Tuple[float, float],
                        goal_pos: Tuple[float, float]) -> Tuple[float, float]:
        """
        Calculate attractive force toward the goal
        """
        dx = goal_pos[0] - robot_pos[0]
        dy = goal_pos[1] - robot_pos[1]

        magnitude = self.attractive_gain * math.sqrt(dx**2 + dy**2)

        # Normalize and scale
        if magnitude > 0:
            fx = self.attractive_gain * dx
            fy = self.attractive_gain * dy
        else:
            fx, fy = 0, 0

        return fx, fy

    def repulsive_force(self, robot_pos: Tuple[float, float],
                       obstacles: List[Tuple[float, float]]) -> Tuple[float, float]:
        """
        Calculate repulsive forces from obstacles
        """
        total_fx, total_fy = 0, 0

        for obs_x, obs_y in obstacles:
            dx = robot_pos[0] - obs_x
            dy = robot_pos[1] - obs_y
            distance = math.sqrt(dx**2 + dy**2)

            if distance < self.obstacle_influence_radius and distance > 0:
                # Calculate repulsive force magnitude
                magnitude = (self.repulsive_gain *
                           (1/self.obstacle_influence_radius - 1/distance) /
                           (distance**2))

                # Calculate force components
                fx = magnitude * dx
                fy = magnitude * dy

                total_fx += fx
                total_fy += fy

        return total_fx, total_fy

    def navigate(self, robot_pos: Tuple[float, float], goal_pos: Tuple[float, float],
                obstacles: List[Tuple[float, float]]) -> Tuple[float, float]:
        """
        Calculate navigation force by combining attractive and repulsive forces
        """
        att_fx, att_fy = self.attractive_force(robot_pos, goal_pos)
        rep_fx, rep_fy = self.repulsive_force(robot_pos, obstacles)

        # Combine forces
        total_fx = att_fx + rep_fx
        total_fy = att_fy + rep_fy

        # Normalize to get direction
        magnitude = math.sqrt(total_fx**2 + total_fy**2)
        if magnitude > 0:
            direction_x = total_fx / magnitude
            direction_y = total_fy / magnitude
        else:
            direction_x, direction_y = 0, 0

        return direction_x, direction_y
```

### Integration with Simulation

These navigation techniques can be integrated into simulation environments:

```python
class SimulationNavigator:
    def __init__(self):
        self.vfh = VectorFieldHistogram()
        self.dwa = DynamicWindowApproach()
        self.potential_field = PotentialFieldNavigator()

        # Current navigation method (can be switched dynamically)
        self.current_method = "dwa"  # Options: "vfh", "dwa", "potential_field"

    def navigate_to_goal(self, robot_state: Tuple[float, float, float, float, float],
                        goal: Tuple[float, float], laser_scan: List[float],
                        obstacles: List[Tuple[float, float]]) -> Tuple[float, float]:
        """
        Navigate to goal using selected method
        """
        if self.current_method == "vfh":
            # Calculate goal direction
            goal_direction = math.atan2(goal[1] - robot_state[1], goal[0] - robot_state[0])
            selected_direction = self.vfh.navigate(laser_scan, goal_direction)

            # Convert direction to velocity commands
            vel = 0.5  # Fixed forward velocity
            ang_vel = selected_direction - robot_state[2]  # Difference from current heading
            return vel, ang_vel

        elif self.current_method == "dwa":
            vel, ang_vel = self.dwa.navigate(robot_state, goal, obstacles)
            return vel, ang_vel

        elif self.current_method == "potential_field":
            direction_x, direction_y = self.potential_field.navigate(
                (robot_state[0], robot_state[1]), goal, obstacles)

            # Convert to velocity commands
            vel = math.sqrt(direction_x**2 + direction_y**2) * 0.5  # Scale to reasonable speed
            ang_vel = math.atan2(direction_y, direction_x) - robot_state[2]
            return vel, ang_vel

    def switch_navigation_method(self, method: str):
        """
        Switch to different navigation method
        """
        if method in ["vfh", "dwa", "potential_field"]:
            self.current_method = method
            print(f"Switched to {method} navigation method")
        else:
            print(f"Invalid navigation method: {method}")

    def adaptive_navigation(self, robot_state: Tuple[float, float, float, float, float],
                           goal: Tuple[float, float], laser_scan: List[float],
                           obstacles: List[Tuple[float, float]]) -> Tuple[float, float]:
        """
        Adaptive navigation that switches methods based on situation
        """
        # Check if environment is cluttered (many obstacles nearby)
        nearby_obstacles = sum(1 for dist in laser_scan if dist < 1.0)
        total_readings = len(laser_scan)

        obstacle_density = nearby_obstacles / total_readings if total_readings > 0 else 0

        # Switch method based on obstacle density
        if obstacle_density > 0.3:  # High obstacle density
            self.current_method = "dwa"  # Better for cluttered environments
        else:
            self.current_method = "potential_field"  # Better for sparse obstacles

        return self.navigate_to_goal(robot_state, goal, laser_scan, obstacles)
```

These obstacle navigation techniques provide different approaches for handling static and dynamic obstacles in simulation environments, each with their own strengths and weaknesses depending on the specific scenario.

## Object Recognition in Simulation

Object recognition capabilities enable robots to identify and interact with objects in their environment.

### Overview of Object Recognition

Object recognition in robotics involves identifying, locating, and classifying objects within the robot's environment. In simulation, this process mimics real-world computer vision tasks but with perfect or near-perfect data.

### Types of Object Recognition

1. **2D Object Detection**: Identifying objects in camera images
2. **3D Object Detection**: Identifying objects in 3D space using depth sensors
3. **Semantic Segmentation**: Classifying each pixel in an image
4. **Instance Segmentation**: Identifying individual object instances
5. **Object Tracking**: Following objects across multiple frames

### Object Recognition Techniques

#### YOLO (You Only Look Once) Implementation

YOLO is a popular real-time object detection algorithm that can be adapted for simulation:

```python
import torch
import torch.nn as nn
import torchvision.transforms as transforms
import cv2
import numpy as np
from typing import List, Tuple, Dict

class YOLOSimulator:
    def __init__(self, model_path: str = None):
        """
        Initialize YOLO object detector
        In simulation, we can use pre-trained models or simplified versions
        """
        self.model = self._load_model(model_path)
        self.confidence_threshold = 0.5
        self.nms_threshold = 0.4

        # COCO dataset class names (80 classes)
        self.classes = [
            'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train',
            'truck', 'boat', 'traffic light', 'fire hydrant', 'stop sign',
            'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep',
            'cow', 'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella',
            'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard',
            'sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard',
            'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup', 'fork',
            'knife', 'spoon', 'bowl', 'banana', 'apple', 'sandwich', 'orange',
            'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair',
            'couch', 'potted plant', 'bed', 'dining table', 'toilet', 'tv',
            'laptop', 'mouse', 'remote', 'keyboard', 'cell phone', 'microwave',
            'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase',
            'scissors', 'teddy bear', 'hair drier', 'toothbrush'
        ]

    def _load_model(self, model_path: str):
        """
        Load YOLO model (in simulation, this could be a simplified version)
        """
        # In a real implementation, this would load a pre-trained model
        # For simulation purposes, we'll create a placeholder
        if model_path:
            # Load from file
            model = torch.load(model_path)
        else:
            # Use a simple placeholder model for simulation
            model = None

        return model

    def detect_objects(self, image: np.ndarray) -> List[Dict]:
        """
        Detect objects in the given image
        Returns list of detected objects with bounding boxes and confidence scores
        """
        # Convert image to tensor
        transform = transforms.Compose([
            transforms.ToTensor(),
        ])

        # In simulation, we could use perfect detection or add controlled noise
        # For this example, we'll simulate detection results
        results = self._simulate_detection(image)

        # Apply non-maximum suppression to remove duplicate detections
        filtered_results = self._non_max_suppression(results)

        return filtered_results

    def _simulate_detection(self, image: np.ndarray) -> List[Dict]:
        """
        Simulate object detection results
        In a real simulation environment, this would interface with the simulator's
        object information
        """
        height, width = image.shape[:2]
        detections = []

        # Simulate detection of some common objects
        # In a real simulator like Gazebo or Isaac Sim, we would get this from the
        # simulation environment directly
        simulated_objects = [
            {"class": "bottle", "bbox": [width*0.3, height*0.4, width*0.4, height*0.6], "confidence": 0.85},
            {"class": "cup", "bbox": [width*0.6, height*0.5, width*0.7, height*0.7], "confidence": 0.78},
            {"class": "person", "bbox": [width*0.1, height*0.2, width*0.25, height*0.8], "confidence": 0.92},
        ]

        for obj in simulated_objects:
            # Only include detections above confidence threshold
            if obj["confidence"] >= self.confidence_threshold:
                detections.append(obj)

        return detections

    def _non_max_suppression(self, detections: List[Dict]) -> List[Dict]:
        """
        Apply non-maximum suppression to remove duplicate detections
        """
        if len(detections) == 0:
            return []

        # Sort by confidence score
        sorted_detections = sorted(detections, key=lambda x: x["confidence"], reverse=True)

        keep = []
        while len(sorted_detections) > 0:
            # Take the detection with highest confidence
            current = sorted_detections.pop(0)
            keep.append(current)

            # Remove overlapping detections
            remaining = []
            for detection in sorted_detections:
                if self._calculate_iou(current["bbox"], detection["bbox"]) < self.nms_threshold:
                    remaining.append(detection)

            sorted_detections = remaining

        return keep

    def _calculate_iou(self, bbox1: List[float], bbox2: List[float]) -> float:
        """
        Calculate Intersection over Union between two bounding boxes
        """
        x1, y1, x2, y2 = bbox1
        x3, y3, x4, y4 = bbox2

        # Calculate intersection area
        inter_x1 = max(x1, x3)
        inter_y1 = max(y1, y3)
        inter_x2 = min(x2, x4)
        inter_y2 = min(y2, y4)

        if inter_x2 < inter_x1 or inter_y2 < inter_y1:
            return 0.0

        inter_area = (inter_x2 - inter_x1) * (inter_y2 - inter_y1)

        # Calculate union area
        area1 = (x2 - x1) * (y2 - y1)
        area2 = (x4 - x3) * (y4 - y3)
        union_area = area1 + area2 - inter_area

        return inter_area / union_area if union_area > 0 else 0.0
```

#### Point Cloud-Based Object Recognition

For 3D object recognition using depth sensors or LiDAR:

```python
import numpy as np
from sklearn.cluster import DBSCAN
from typing import List, Tuple
import open3d as o3d  # Optional: for 3D processing

class PointCloudObjectRecognizer:
    def __init__(self):
        self.voxel_size = 0.05  # 5cm voxels
        self.cluster_eps = 0.1  # 10cm clustering distance
        self.min_cluster_points = 10

    def process_depth_image(self, depth_image: np.ndarray,
                           camera_intrinsics: Dict) -> List[Dict]:
        """
        Process depth image to extract 3D point cloud and recognize objects
        """
        # Convert depth image to point cloud
        point_cloud = self._depth_to_pointcloud(depth_image, camera_intrinsics)

        # Segment the point cloud into objects
        object_clusters = self._segment_pointcloud(point_cloud)

        # Recognize objects in each cluster
        recognized_objects = []
        for i, cluster in enumerate(object_clusters):
            obj_info = self._recognize_object_cluster(cluster, i)
            if obj_info:
                recognized_objects.append(obj_info)

        return recognized_objects

    def _depth_to_pointcloud(self, depth_image: np.ndarray,
                           camera_intrinsics: Dict) -> np.ndarray:
        """
        Convert depth image to 3D point cloud
        """
        height, width = depth_image.shape
        cx, cy = camera_intrinsics['cx'], camera_intrinsics['cy']
        fx, fy = camera_intrinsics['fx'], camera_intrinsics['fy']

        # Create coordinate grids
        x_coords, y_coords = np.meshgrid(np.arange(width), np.arange(height))

        # Convert to 3D coordinates
        z_coords = depth_image
        x_coords = (x_coords - cx) * z_coords / fx
        y_coords = (y_coords - cy) * z_coords / fy

        # Stack to create point cloud
        points = np.stack([x_coords, y_coords, z_coords], axis=-1)

        # Reshape to (N, 3) format and remove invalid points
        points = points.reshape(-1, 3)
        valid_mask = ~np.isnan(points).any(axis=1) & ~np.isinf(points).any(axis=1)
        valid_points = points[valid_mask]

        return valid_points

    def _segment_pointcloud(self, point_cloud: np.ndarray) -> List[np.ndarray]:
        """
        Segment point cloud into individual objects using clustering
        """
        # Apply DBSCAN clustering to group points into objects
        clustering = DBSCAN(eps=self.cluster_eps, min_samples=self.min_cluster_points)
        labels = clustering.fit_predict(point_cloud)

        # Group points by cluster label
        clusters = []
        for label in set(labels):
            if label == -1:  # Noise points
                continue

            cluster_points = point_cloud[labels == label]
            if len(cluster_points) >= self.min_cluster_points:
                clusters.append(cluster_points)

        return clusters

    def _recognize_object_cluster(self, cluster_points: np.ndarray,
                                cluster_id: int) -> Dict:
        """
        Recognize an object from a point cluster
        """
        if len(cluster_points) < self.min_cluster_points:
            return None

        # Calculate object properties
        centroid = np.mean(cluster_points, axis=0)
        bbox_3d = self._calculate_3d_bbox(cluster_points)

        # Simple shape classification based on dimensions
        dimensions = bbox_3d[1] - bbox_3d[0]  # width, height, depth
        shape_class = self._classify_shape(dimensions)

        # Calculate volume as a simple recognition feature
        volume = np.prod(dimensions)

        return {
            "id": cluster_id,
            "centroid": centroid,
            "bbox_3d": bbox_3d,
            "dimensions": dimensions,
            "volume": volume,
            "shape_class": shape_class,
            "point_count": len(cluster_points)
        }

    def _calculate_3d_bbox(self, points: np.ndarray) -> np.ndarray:
        """
        Calculate 3D bounding box for a set of points
        Returns [[min_x, min_y, min_z], [max_x, max_y, max_z]]
        """
        min_vals = np.min(points, axis=0)
        max_vals = np.max(points, axis=0)
        return np.array([min_vals, max_vals])

    def _classify_shape(self, dimensions: np.ndarray) -> str:
        """
        Simple shape classification based on dimensions
        """
        sorted_dims = np.sort(dimensions)
        ratios = sorted_dims / sorted_dims[0]  # Ratios relative to smallest dimension

        if ratios[1] < 1.5 and ratios[2] < 1.5:
            return "cube"
        elif ratios[2] > 3 and ratios[1] < 2:
            return "cylinder"
        elif ratios[2] > 5:
            return "elongated"
        else:
            return "irregular"
```

### Integration with Simulation Environments

Object recognition can be integrated with simulation environments like Isaac Sim or Gazebo:

```python
class SimulationObjectRecognition:
    def __init__(self, sim_env):
        self.sim_env = sim_env
        self.yolo_detector = YOLOSimulator()
        self.pointcloud_recognizer = PointCloudObjectRecognizer()

        # In simulation, we can also access ground truth object information
        self.use_ground_truth = True

    def recognize_objects(self, camera_image: np.ndarray = None,
                        depth_image: np.ndarray = None,
                        camera_intrinsics: Dict = None) -> List[Dict]:
        """
        Recognize objects using both 2D and 3D information
        """
        recognized_objects = []

        # Use 2D object detection
        if camera_image is not None:
            detected_2d = self.yolo_detector.detect_objects(camera_image)
            recognized_objects.extend(self._process_2d_detections(detected_2d))

        # Use 3D point cloud recognition
        if depth_image is not None and camera_intrinsics is not None:
            detected_3d = self.pointcloud_recognizer.process_depth_image(
                depth_image, camera_intrinsics)
            recognized_objects.extend(detected_3d)

        # In simulation, we can also use ground truth information
        if self.use_ground_truth:
            ground_truth_objects = self._get_ground_truth_objects()
            recognized_objects.extend(ground_truth_objects)

        # Merge and deduplicate detections
        merged_objects = self._merge_detections(recognized_objects)

        return merged_objects

    def _process_2d_detections(self, detections: List[Dict]) -> List[Dict]:
        """
        Process 2D detections to add 3D information where possible
        """
        processed_detections = []

        for detection in detections:
            # Add 3D position if depth information is available
            # This would be done by projecting the 2D bounding box to 3D space
            detection_3d = detection.copy()
            detection_3d["type"] = "2d_detection"
            processed_detections.append(detection_3d)

        return processed_detections

    def _get_ground_truth_objects(self) -> List[Dict]:
        """
        Get ground truth object information from simulation
        This is available in simulation but not in real-world scenarios
        """
        # In a real simulation environment, this would interface with
        # the simulator to get exact object poses and types
        gt_objects = []

        # Example: Get all objects in the simulation
        sim_objects = self.sim_env.get_all_objects()

        for obj in sim_objects:
            gt_objects.append({
                "id": obj.id,
                "class": obj.class_name,
                "position": obj.position,
                "orientation": obj.orientation,
                "dimensions": obj.dimensions,
                "type": "ground_truth"
            })

        return gt_objects

    def _merge_detections(self, all_detections: List[Dict]) -> List[Dict]:
        """
        Merge multiple detection sources and remove duplicates
        """
        # This would implement logic to merge detections from different sources
        # based on spatial proximity and class consistency
        return all_detections

    def get_object_interaction_info(self, target_object_class: str) -> Dict:
        """
        Get information needed for object interaction
        """
        all_objects = self.recognize_objects()

        # Find objects of the target class
        target_objects = [obj for obj in all_objects
                         if obj.get("class") == target_object_class]

        if not target_objects:
            return None

        # Return information for the closest object
        closest_obj = min(target_objects,
                         key=lambda x: np.linalg.norm(x["position"]))

        return {
            "object": closest_obj,
            "position": closest_obj["position"],
            "approach_point": self._calculate_approach_point(closest_obj),
            "grasp_points": self._calculate_grasp_points(closest_obj)
        }

    def _calculate_approach_point(self, obj: Dict) -> np.ndarray:
        """
        Calculate a safe approach point for the robot to reach the object
        """
        # Simple approach: position in front of the object
        approach_offset = np.array([0.5, 0, 0])  # 50cm in front
        return np.array(obj["position"]) + approach_offset

    def _calculate_grasp_points(self, obj: Dict) -> List[np.ndarray]:
        """
        Calculate potential grasp points for the object
        """
        # For simulation, we can use object-specific grasp points
        # In reality, this would require more complex grasp planning
        position = np.array(obj["position"])
        grasp_points = [
            position + np.array([0, 0, obj["dimensions"][2]/2 + 0.1]),  # Top center
            position + np.array([obj["dimensions"][0]/2 + 0.1, 0, 0]),  # Side
        ]

        return grasp_points
```

These object recognition techniques provide humanoid robots with the ability to perceive and understand their environment in simulation, enabling more sophisticated interaction and manipulation tasks.

## Navigation

- [Previous: Chapter 03 - Cognitive Planning with LLMs](./chapter-03-cognitive-planning.md)
- [Next: Chapter 05 - Capstone: Autonomous Humanoid](./chapter-05-capstone.md)