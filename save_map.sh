#!/usr/bin/env bash
set -eo pipefail

cd "$(dirname "$0")"
# 清掉当前终端里可能遗留的其他 ROS2 工作区覆盖层，避免混入旧工程的包。
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

mkdir -p src/slam_nav_bringup/map
name="${1:-nav_test_map}"
exec ros2 run nav2_map_server map_saver_cli -f "src/slam_nav_bringup/map/${name}"
