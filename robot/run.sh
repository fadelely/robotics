#!/usr/bin/env bash
set -e

colcon build --packages-select mujoco_ros2
source install/setup.bash
ros2 launch mujoco_ros2 example.py
