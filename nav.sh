#!/bin/zsh

# Start navigation
source install/setup.sh

ros2 launch rm_nav_bringup sentry_nav_launch.py \
  robots:=" \
  red_standard_robot1={x: 0.0, y: 0.0, yaw: 0.0}; \
  blue_standard_robot1={x: 5.6, y: 1.4, yaw: 3.14}; \
  "