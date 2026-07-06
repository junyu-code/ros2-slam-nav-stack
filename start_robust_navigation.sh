#!/usr/bin/env bash
set -eo pipefail

cd "$(dirname "$0")"
# 清掉当前终端里可能遗留的其他 ROS2 工作区覆盖层，避免混入旧工程的包。
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

exec ros2 launch slam_nav_bringup robust_navigation.launch.py use_sim_time:=true rviz:=true localization_mode:=static "$@"
