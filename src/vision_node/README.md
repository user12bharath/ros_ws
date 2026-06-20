# Real-Time Vision Pipeline — FaceDetectorNode (ROS2)

A multi-threaded ROS2 perception pipeline that performs real-time face detection on a live camera feed and publishes results as ROS2 topics for downstream consumption — the same architectural pattern used in real robot perception stacks (camera node → detection node → decision-making node).

## What this project demonstrates

- Decoupling slow sensor I/O (camera capture) from the ROS2 processing loop using a dedicated capture thread and a mutex-protected shared frame buffer
- A complete two-node pub/sub system: a perception node that publishes detections, and a monitor node that subscribes to multiple topics and aggregates state independently of message arrival rate
- Real-time performance engineering — measured and fixed a 4x frame-rate bottleneck caused by blocking camera reads inside the ROS2 timer callback

## System architecture

```
┌─────────────────────────────────────────┐
│           FaceDetectorNode                │
│                                            │
│  ┌────────────────┐    ┌───────────────┐ │
│  │ Capture Thread  │───▶│ Shared Frame  │ │
│  │ (cv2.VideoCapture)│  │ (mutex-locked)│ │
│  └────────────────┘    └───────┬───────┘ │
│                                  │         │
│                          ┌───────▼───────┐ │
│                          │ process_frame │ │
│                          │   @ 30Hz timer│ │
│                          │ Haar cascade   │ │
│                          │ face detection │ │
│                          └───────┬───────┘ │
└──────────────────────────────────┼─────────┘
                                    │
              ┌─────────────────────┼─────────────────────┐
              ▼                     ▼                     ▼
       /vision/face_count    /vision/fps          /vision/alerts
         (Int32)              (Float64)             (String)
              │                     │                     │
              └─────────────────────┼─────────────────────┘
                                    ▼
                          ┌──────────────────┐
                          │  VisionMonitor    │
                          │  (subscriber node)│
                          │  prints status     │
                          │  report @ 1Hz       │
                          └──────────────────┘
```

## Why the threading matters

The first version of this node read the webcam directly inside the ROS2 timer callback. The timer was configured for 30Hz, but `cv2.VideoCapture.read()` blocked for ~130ms per frame, throttling the entire node to ~7.5Hz — a 4x slowdown invisible until measured with `ros2 topic hz`.

The fix: camera capture runs in a dedicated background thread that continuously fills a shared frame buffer, protected by a `threading.Lock`. The 30Hz timer callback only reads the latest available frame and never blocks on hardware I/O. This is the same pattern used for any slow sensor (cameras, some lidar drivers, serial-port sensors) in production ROS2 systems.

## Topics published

| Topic                | Type               | Description                                   |
| -------------------- | ------------------ | --------------------------------------------- |
| `/vision/face_count` | `std_msgs/Int32`   | Number of faces detected in the current frame |
| `/vision/fps`        | `std_msgs/Float64` | Live measured processing frame rate           |
| `/vision/alerts`     | `std_msgs/String`  | `"FACE DETECTED"` or `"CLEAR"`                |

## Repository structure

```
src/vision_node/
├── vision_node/
│   ├── face_detector_node.py      # capture thread + detection + publishers
│   ├── vision_monitor_node.py     # multi-topic subscriber + status reporter
│   └── haarcascade_frontalface_default.xml
├── package.xml
└── setup.py
```

## Prerequisites

- ROS2 Jazzy
- Python 3
- OpenCV (`opencv-python`)
- A connected webcam

```bash
pip3 install opencv-python --break-system-packages
```

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select vision_node
source install/setup.bash
```

## Usage

Terminal 1 — run the perception node:

```bash
ros2 run vision_node face_detector
```

Terminal 2 — run the monitor node:

```bash
ros2 run vision_node vision_monitor
```

Expected monitor output, refreshed every second:

```
[VISION STATUS] FPS: 29.8 | Faces: 1 | Status: FACE DETECTED
```

Inspect raw topics directly:

```bash
ros2 topic hz /vision/fps        # confirm ~30Hz after the threading fix
ros2 topic echo /vision/face_count
```

## Engineering notes — problems solved during development

**4x frame-rate bottleneck:** Diagnosed using `ros2 topic hz`, which showed 7.5Hz against a configured 30Hz timer. Root cause was a blocking `cap.read()` call inside the timer callback. Fixed with a dedicated capture thread and a `threading.Lock`-protected shared frame, restoring the full 30Hz.

**Hardcoded cascade path:** Original implementation used an absolute path tied to one machine. Replaced with a path resolved relative to the package file (`os.path.dirname(__file__)`), with a fallback to OpenCV's bundled cascade directory (`cv2.data.haarcascades`) for portability across machines.

**FPS overlay only rendering when a face was present:** The text draw call was originally inside the per-face bounding-box loop, so it silently disappeared whenever zero faces were detected. Moved outside the loop so the FPS readout is always visible regardless of detection state.

## Known limitation

Haar cascades are a 2001-era classical CV technique — they degrade at extreme head angles, in low light, and with partial occlusion. A natural extension of this project is swapping the detector for a lightweight deep-learning model (e.g. a YOLO-family detector) behind the same publisher interface, with no changes required downstream.

## Tech stack

ROS2 Jazzy · Python (rclpy) · OpenCV · Haar Cascade Classifier · Python `threading`

## Author

Pavan M — [github.com/Hangman-dot](https://github.com/Hangman-dot)
