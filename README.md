# ROS2 Autonomous Robotics Portfolio

Three connected ROS2 projects built end-to-end: a fully autonomous differential-drive robot (SLAM + Nav2 in Gazebo), a real-time multi-threaded vision pipeline, and a set of Python modules that reimplement ROS2's internal pub/sub mechanism from first principles. Built incrementally, debugging real failures along the way rather than following a single tutorial path.

## Repository structure

```
src/
├── my_robot_description/        # Project 1 — robot model, world, maps
│   ├── urdf/my_robot.urdf
│   ├── launch/spawn_robot.launch.py
│   ├── worlds/my_world.sdf
│   └── maps/my_world_map.{pgm,yaml}
├── my_robot/                    # Project 1 — control + avoidance nodes
│   └── my_robot/
│       ├── square_driver.py
│       └── obstacle_avoidance.py
├── vision_node/                 # Project 2 — perception pipeline
    └── vision_node/
        ├── face_detector_node.py
        └── vision_monitor_node.py
```

---

## Project 1 — Autonomous Differential-Drive Robot (ROS2 + Gazebo Harmonic)

A complete autonomous navigation stack for a custom-built differential-drive robot, simulated end-to-end in Gazebo Harmonic with ROS2 Jazzy. The robot maps an unknown environment using SLAM, localizes itself within that map, and autonomously plans and executes paths to goal positions while avoiding obstacles in real time.

**What this demonstrates:** custom URDF robot model built from scratch (no pre-built robot description used), a custom Gazebo world, a differential-drive physics plugin wired to that URDF, reactive obstacle avoidance written directly against raw lidar data, a full SLAM mapping pipeline (slam_toolbox), and a full Nav2 stack — AMCL localization, global/local planning, recovery behaviors, multi-waypoint navigation.

### Architecture

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
      │   SLAM Toolbox     │                │  Obstacle Avoid   │
      │ (mapping mode)      │                │  (reactive node)  │
      └────────┬────────────┘                └──────────────────┘
               │ /map
               ▼
      ┌──────────────────┐
      │   Nav2 Stack        │
      │ AMCL → Planner →     │
      │ Controller → BT      │
      └────────┬────────────┘
               │ /cmd_vel
               ▼
        Robot moves autonomously
```

### Robot spec

Custom URDF with a rectangular base link, two continuous-joint driven wheels (differential drive), one fixed passive caster wheel, and a 360° lidar (0.12m–3.5m range) mounted on top. Wheel separation 0.24m, wheel radius 0.033m.

### Build

```bash
sudo apt install ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox ros-jazzy-ros-gz \
                 ros-jazzy-joint-state-publisher -y

cd ~/ros2_ws
colcon build --packages-select my_robot_description my_robot
source install/setup.bash
```

### Usage

**1. Launch simulation**

```bash
ros2 launch my_robot_description spawn_robot.launch.py
```

**2. Mapping mode (SLAM)**

```bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true
# drive the robot or run obstacle_avoidance to explore, then save:
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/my_robot_description/maps/my_world_map
```

**3. Navigation mode (using the saved map)**

```bash
# Terminal 1
ros2 launch my_robot_description spawn_robot.launch.py

# Terminal 2 — localization
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

In RViz: set **2D Pose Estimate** first to initialize AMCL, then send goals via **Nav2 Goal** or the waypoint panel.

**4. Reactive obstacle avoidance (standalone)**

```bash
ros2 run my_robot obstacle_avoidance
```

Subscribes to `/scan`, filters the front 60° sector, steers away from obstacles closer than 0.5m with hysteresis to prevent oscillation.

### Engineering problems solved

**Frame ID mismatch (Gazebo ↔ ROS2 bridge):** Gazebo's auto-generated frame naming (`my_robot/base_footprint/lidar`) didn't match the URDF link name (`base_scan`) SLAM expected, silently breaking mapping. Fixed by explicitly setting `<gz_frame_id>` in the lidar sensor's Gazebo plugin block.

**SLAM node alive but not mapping:** `async_slam_toolbox_node` is a ROS2 lifecycle node that does not auto-activate — it was running with zero subscriptions until explicitly transitioned through `configure` → `activate`. Resolved by switching to the official `online_async_launch.py`, which manages this automatically.

**Recovery behaviors firing on first navigation attempt:** Early planning attempts failed with `Failed to create plan with tolerance of: 0.5` because AMCL's pose estimate hadn't converged yet. Nav2's behavior tree correctly triggered `spin` → `wait` → `backup` recovery, letting AMCL refine localization before the planner succeeded.

**Tech stack:** ROS2 Jazzy · Gazebo Harmonic · Nav2 · slam_toolbox · Python (rclpy) · URDF/SDF · RViz2

---

## Project 2 — Real-Time Vision Pipeline: FaceDetectorNode (ROS2)

A multi-threaded ROS2 perception pipeline performing real-time face detection on a live camera feed, publishing results as ROS2 topics for downstream consumption — the same pattern used in real robot perception stacks (camera node → detection node → decision-making node).

**What this demonstrates:** decoupling slow sensor I/O from the ROS2 processing loop using a dedicated capture thread and a mutex-protected shared frame buffer; a complete two-node pub/sub system (a perception node publishing detections, a monitor node subscribing to multiple topics and aggregating state independently of message arrival rate); measured and fixed a 4x frame-rate bottleneck caused by blocking camera reads inside the timer callback.

### Architecture

```
┌─────────────────────────────────────────┐
│           FaceDetectorNode                │
│  ┌────────────────┐    ┌───────────────┐ │
│  │ Capture Thread   │──▶│ Shared Frame   │ │
│  │ (cv2.VideoCapture)│  │ (mutex-locked) │ │
│  └────────────────┘    └───────┬───────┘ │
│                          ┌───────▼───────┐ │
│                          │ process_frame  │ │
│                          │  @ 30Hz timer  │ │
│                          │ Haar cascade   │ │
│                          └───────┬───────┘ │
└──────────────────────────────────┼─────────┘
              ┌─────────────────────┼─────────────────────┐
              ▼                     ▼                     ▼
       /vision/face_count    /vision/fps          /vision/alerts
         (Int32)              (Float64)             (String)
              └─────────────────────┼─────────────────────┘
                                    ▼
                          ┌──────────────────┐
                          │  VisionMonitor     │
                          │  prints status      │
                          │  report @ 1Hz        │
                          └──────────────────┘
```

### Why the threading matters

The first version read the webcam directly inside the ROS2 timer callback. The timer was set to 30Hz, but `cv2.VideoCapture.read()` blocked for ~130ms per frame, throttling the whole node to ~7.5Hz — invisible until measured with `ros2 topic hz`. Fixed by moving camera capture into a background thread that continuously fills a shared frame buffer behind a `threading.Lock`; the 30Hz timer callback only reads the latest available frame and never blocks on hardware I/O.

### Topics published

| Topic                | Type               | Description                         |
| -------------------- | ------------------ | ----------------------------------- |
| `/vision/face_count` | `std_msgs/Int32`   | Faces detected in the current frame |
| `/vision/fps`        | `std_msgs/Float64` | Live measured processing frame rate |
| `/vision/alerts`     | `std_msgs/String`  | `"FACE DETECTED"` or `"CLEAR"`      |

### Build & usage

```bash
pip3 install opencv-python --break-system-packages
cd ~/ros2_ws
colcon build --packages-select vision_node
source install/setup.bash

# Terminal 1
ros2 run vision_node face_detector
# Terminal 2
ros2 run vision_node vision_monitor
```

Expected monitor output, refreshed every second:

```
[VISION STATUS] FPS: 29.8 | Faces: 1 | Status: FACE DETECTED
```

### Engineering problems solved

**4x frame-rate bottleneck:** diagnosed via `ros2 topic hz` showing 7.5Hz against a configured 30Hz timer; root cause was a blocking `cap.read()` inside the callback; fixed with a dedicated capture thread and lock-protected shared frame.

**Hardcoded cascade path:** replaced an absolute, machine-specific path with one resolved relative to the package file (`os.path.dirname(__file__)`), falling back to `cv2.data.haarcascades` for portability.

**FPS overlay disappearing with zero faces:** the text draw call was originally inside the per-face loop; moved outside so the FPS readout is always visible.

**Known limitation:** Haar cascades are a 2001-era classical CV technique and degrade at extreme angles, low light, or partial occlusion — a natural extension is swapping in a lightweight deep-learning detector behind the same publisher interface.

**Tech stack:** ROS2 Jazzy · Python (rclpy) · OpenCV · Haar Cascade Classifier · Python `threading`

---
