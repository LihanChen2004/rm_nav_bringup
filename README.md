# rm_nav_bringup

用于 [rmul24_gazebo_simulator](https://github.com/LihanChen2004/rmul24_gazebo_simulator.git) 的导航仿真启动包。

## Cloned

```sh
git clone https://github.com/LihanChen2004/rm_nav_bringup.git ros_ws/src
```

```sh
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

## Launch

多机器人仿真导航：

```sh
ros2 launch rm_nav_bringup sentry_nav_launch.py \
  robots:=" \
  red_standard_robot1={x: 0.0, y: 0.0, yaw: 0.0}; \
  blue_standard_robot1={x: 14.3, y: 7.85, yaw: 3.14}; \
  "
```

注意在两个 rviz2 中分别给定机器人的初始位姿
