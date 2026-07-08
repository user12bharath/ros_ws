# Workspace Overview

This directory is the source root for the ROS 2 workspace. It contains the main packages used by the robot, the simulation assets, and the supporting examples.

## Packages

- `my_robot_description`: robot URDF, Gazebo world, launch files, maps, and navigation configuration
- `my_robot`: robot control nodes, obstacle avoidance, and target reaction logic
- `vision_node`: camera-based perception nodes and vision monitoring
- `ros2_fundamentals_examples`: small ROS 2 examples for core concepts and patterns
- `system_monitor`: runtime monitoring utilities

## Typical workflow

```bash
cd ~/ros_ws
colcon build
source install/setup.bash
```

## Notes

- Keep package-specific usage instructions in the package README when available.
- This file is meant to help you find the right package quickly from the workspace root.
