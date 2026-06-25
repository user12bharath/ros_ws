# ROS2 Autonomous Robotics Portfolio

A perception-driven autonomous ground robot, built end-to-end in ROS2 Jazzy and Gazebo Harmonic. The robot autonomously navigates a mapped environment using Nav2, while a real-time camera-based detector fuses with lidar range data to override navigation and react to a visual target — arbitrated through priority-based command multiplexing. Three connected projects, built incrementally, debugging real integration failures along the way rather than following a single tutorial path.

## The centerpiece: fused perception + navigation

The robot runs full autonomous navigation (SLAM-built map, AMCL localization, Nav2 global/local planning) **and** a parallel vision-based safety behavior simultaneously. Both write velocity commands; `twist_mux` arbitrates between them by priority, so a visual detection can interrupt an in-progress navigation goal without modifying the navigation stack itself.

```
                    ┌─────────────────────┐
                    │     Nav2 Stack        │
                    │ (AMCL → Planner →     │
                    │  MPPI Controller)      │
                    └──────────┬────────────┘
                               │ /cmd_vel  (priority 10)
                               ▼
┌──────────────────┐   ┌──────────────┐   ┌──────────────────┐
│  Camera (sim)      │──▶│ Color Target  │   │   twist_mux        │
│  + Lidar            │   │ Detector       │──▶│ (priority         │──▶ /cmd_vel_out ──▶ Robot
└──────────────────┘   └──────┬───────┘   │  arbitration)       │
                               │ alert         └──────────────────┘
                               ▼                       ▲
                    ┌─────────────────────┐            │
                    │   Target Reactor      │────────────┘
                    │ (vision + lidar        │ /cmd_vel_safety
                    │  fusion → speed)        │ (priority 100)
                    └─────────────────────┘
```

When the camera detects the target and lidar confirms it's within range, `target_reactor` (priority 100) overrides Nav2 (priority 10) — slowing and stopping the robot even while the planner still believes it should keep moving toward the original goal.

## Repository structure

```
src/
├── my_robot_description/        # robot model, world, maps, launch
│   ├── urdf/my_robot.urdf        # base, wheels, caster, lidar, camera
│   ├── launch/spawn_robot.launch.py
│   ├── worlds/my_world.sdf       # walls + red target object
│   └── maps/my_world_map.{pgm,yaml}
│   └── config/twist_mux.yaml
├── my_robot/                    # control + fusion logic
│   └── my_robot/
│       ├── square_driver.py      # open-loop control (early milestone)
│       ├── obstacle_avoidance.py # reactive lidar-only avoidance
│       └── target_reactor.py     # vision + lidar fusion → safety override
├── vision_node/                 # perception pipeline
    └── vision_node/
        ├── face_detector_node.py     # webcam Haar-cascade detector
        └── vision_monitor_node.py
        └── sim_camera_detector.py    # HSV color detection on sim camera
└── ros2_internals/               # pub/sub mechanism in pure Python
    ├── topic_bus.py
    ├── ring_buffer.py
    ├── sensor_fusion.py
    └── decorators.py
```

---

## Full system bring-up

```
# 1. Simulation
ros2 launch my_robot_description spawn_robot.launch.py

# 2. Localization — wait for "Managed nodes are active" before continuing
ros2 launch nav2_bringup localization_launch.py \
  use_sim_time:=true \
  map:=~/ros2_ws/src/my_robot_description/maps/my_world_map.yaml

# 3. Set initial pose (required before navigation will accept goals)
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, \
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}"

# 4. Navigation stack
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=true \
  params_file:=/opt/ros/jazzy/share/nav2_bringup/params/nav2_params.yaml

# 5. Perception + fusion
ros2 run vision_node sim_camera_detector
ros2 run my_robot target_reactor

# 6. Arbitration — final output the robot actually listens to
ros2 run twist_mux twist_mux \
  --ros-args --params-file ~/ros2_ws/src/my_robot_description/config/twist_mux.yaml \
  -r cmd_vel_out:=cmd_vel_out

# 7. RViz, then send a goal via the Nav2 Goal panel or /goal_pose
ros2 launch nav2_bringup rviz_launch.py
```

Watch the fusion live:

```
ros2 topic echo /cmd_vel_out
```

## Engineering problems solved — the real debugging log

This project broke in more places than it worked on the first try. Documenting the actual failures because the debugging is the substance of the work, not the final diagram.

**Frame ID mismatch (Gazebo ↔ ROS2 bridge):** Gazebo's auto-generated frame naming didn't match the URDF link name SLAM expected, silently breaking mapping. Fixed with an explicit `` override in each sensor's Gazebo plugin block.

**SLAM node alive but not mapping:** `slam_toolbox`'s lifecycle node doesn't auto-activate — it sat with zero subscriptions until explicitly transitioned through `configure → activate`. Fixed by using the official `online_async_launch.py`, which manages this automatically.

**Recovery behaviors firing on first navigation attempt:** Early planning failed with `tolerance of: 0.5` because AMCL's pose hadn't converged. Nav2's behavior tree correctly triggered `spin → wait → backup`, letting AMCL refine localization before the planner succeeded — not a bug, the designed fault-recovery path.

**Two publishers, one topic, two message types:** `twist_mux` (v4.5.0) defaults to publishing `TwistStamped`; the Gazebo bridge was subscribing expecting plain `Twist`. `ros2 topic info --verbose` showed two conflicting types on the same topic — fixed by aligning every `cmd_vel*` topic in the chain to `TwistStamped`, including a corresponding change in `target_reactor`'s publisher.

**RViz map display silently empty despite `/map` publishing correctly:** a QoS durability mismatch — `map_server` publishes with `TRANSIENT_LOCAL`, RViz's Map display defaulted to `VOLATILE`. DDS refuses to connect incompatible QoS policies with no error, no crash — the display just never receives anything. Fixed by re-adding the display with matching durability.

**"Goal failed" that wasn't actually a planning failure:** the safety reactor was correctly publishing zero velocity (target already in range at the robot's resting position) and, at priority 100 versus navigation's priority 10, was completely masking Nav2's output. `ros2 topic echo /plan` confirmed the planner had succeeded the whole time — the architecture was working exactly as designed; the test setup (robot starting already inside the safety zone) was what needed fixing, not the code.

**Camera lost the target at close range while lidar still tracked it cleanly:** a real sensor-tradeoff, not a bug — narrow camera FOV loses a large, close, slightly off-center object well before an omnidirectional lidar loses contact with it. This is exactly the kind of limitation that motivates real sensor-fusion architectures in industry. Fixed for this demo with temporal smoothing: the reactor only releases its "target visible" state after several consecutive missed detections, rather than reacting to a single dropped frame.

## Tech stack

ROS2 Jazzy · Gazebo Harmonic · Nav2 (AMCL, MPPI controller, behavior trees) · slam_toolbox · twist_mux · OpenCV (Haar cascades, HSV color detection) · cv_bridge · Python (rclpy, threading) · URDF/SDF · RViz2

##

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
