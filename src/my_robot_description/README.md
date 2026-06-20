# Autonomous Differential-Drive Robot — ROS2 + Gazebo Harmonic

A complete autonomous navigation stack for a custom-built differential-drive robot, simulated end-to-end in Gazebo Harmonic with ROS2 Jazzy. The robot maps an unknown environment using SLAM, localizes itself within that map, and autonomously plans and executes paths to goal positions while avoiding obstacles in real time.

## What this project demonstrates

This is not a tutorial clone. The robot, world, and full navigation pipeline were built from scratch:

- Custom URDF robot model (no pre-built robot description used)
- Custom Gazebo world with obstacles
- Differential-drive physics plugin wired to a hand-built URDF
- Reactive obstacle avoidance written from raw lidar data
- Full SLAM mapping pipeline (slam_toolbox)
- Full Nav2 stack: AMCL localization, global/local planning, recovery behaviors, multi-waypoint navigation

## System architecture

```
                         ┌─────────────────┐
                         │   Gazebo Sim     │
                         │  (physics, lidar,│
                         │   diff-drive)    │
                         └────────┬─────────┘
                                  │ /scan, /odom, /tf
                ┌─────────────────┼─────────────────┐
                ▼                                   ▼
      ┌──────────────────┐                ┌──────────────────┐
      │   SLAM Toolbox    │                │  Obstacle Avoid   │
      │ (mapping mode)     │                │  (reactive node)  │
      └────────┬───────────┘                └──────────────────┘
               │ /map
               ▼
      ┌──────────────────┐
      │   Nav2 Stack       │
      │ AMCL → Planner →   │
      │ Controller → BT    │
      └────────┬───────────┘
               │ /cmd_vel
               ▼
        Robot moves autonomously
```

## Robot specification

Custom URDF with:

- Rectangular base link
- Two continuous-joint driven wheels (differential drive)
- One fixed passive caster wheel
- Lidar sensor (360°, 0.12m–3.5m range, mounted on top)

Differential drive plugin parameters:

- Wheel separation: 0.24m
- Wheel radius: 0.033m

## Repository structure

```
src/
├── my_robot_description/
│   ├── urdf/
│   │   └── my_robot.urdf          # robot model + gazebo plugins
│   ├── launch/
│   │   └── spawn_robot.launch.py  # spawns robot into Gazebo
│   ├── worlds/
│   │   └── my_world.sdf           # custom world with walls
│   └── maps/
│       ├── my_world_map.pgm       # saved occupancy grid
│       └── my_world_map.yaml      # map metadata
└── my_robot/
    ├── my_robot/
    │   ├── square_driver.py       # open-loop control node
    │   └── obstacle_avoidance.py  # reactive lidar-based avoidance
    └── package.xml
```

## Prerequisites

- Ubuntu 24.04
- ROS2 Jazzy
- Gazebo Harmonic (`gz sim`)
- `ros-jazzy-nav2-bringup`
- `ros-jazzy-slam-toolbox`
- `ros-jazzy-ros-gz`

```bash
sudo apt install ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox ros-jazzy-ros-gz \
                 ros-jazzy-joint-state-publisher -y
```

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_description my_robot
source install/setup.bash
```

## Usage

### 1. Launch the simulation

```bash
ros2 launch my_robot_description spawn_robot.launch.py
```

Spawns the robot into the custom world. Verify topics:

```bash
ros2 topic list
# expect: /cmd_vel, /scan, /odom, /joint_states, /tf
```

### 2. Mapping mode (SLAM)

```bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true
```

Drive the robot manually or run `obstacle_avoidance` to explore the world, then save the map:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/my_robot_description/maps/my_world_map
```

### 3. Navigation mode (using the saved map)

```bash
# Terminal 1
ros2 launch my_robot_description spawn_robot.launch.py

# Terminal 2 — localization against the saved map
ros2 launch nav2_bringup localization_launch.py \
  use_sim_time:=true \
  map:=~/ros2_ws/src/my_robot_description/maps/my_world_map.yaml

# Terminal 3 — navigation stack
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=true \
  params_file:=/opt/ros/jazzy/share/nav2_bringup/params/nav2_params.yaml

# Terminal 4 — RViz with Nav2 panel
ros2 launch nav2_bringup rviz_launch.py
```

In RViz: set **2D Pose Estimate** first to initialize AMCL, then use **Nav2 Goal** or the waypoint panel to send goals.

### 4. Reactive obstacle avoidance (standalone)

```bash
ros2 run my_robot obstacle_avoidance
```

Subscribes to `/scan`, filters the front 60° sector, and steers away from obstacles closer than 0.5m with hysteresis to prevent oscillation.

## Engineering notes — problems solved during development

**Frame ID mismatch (Gazebo ↔ ROS2 bridge):** Gazebo's auto-generated frame naming (`my_robot/base_footprint/lidar`) didn't match the URDF link name (`base_scan`) that SLAM expected, silently breaking mapping. Fixed by explicitly setting `<gz_frame_id>` in the lidar sensor's Gazebo plugin block.

**SLAM node running but not mapping:** `slam_toolbox`'s `async_slam_toolbox_node` is a ROS2 lifecycle node — it does not auto-activate. It was alive but had zero subscriptions until explicitly transitioned through `configure` → `activate`. Resolved by switching to the official `online_async_launch.py`, which manages this lifecycle automatically.

**Recovery behavior triggering on first navigation attempt:** Early goal-planning attempts failed with `Failed to create plan with tolerance of: 0.5` because AMCL's pose estimate hadn't converged yet. Nav2's behavior tree correctly triggered `spin` → `wait` → `backup` recovery in sequence, which let AMCL gather more lidar readings to refine localization before the planner succeeded.

## Tech stack

ROS2 Jazzy · Gazebo Harmonic · Nav2 · slam_toolbox · Python (rclpy) · URDF/SDF · RViz2

## Author

Pavan M — [github.com/Hangman-dot](https://github.com/Hangman-dot)
