#!/usr/bin/env bash
set -eo pipefail

cd "$(dirname "$0")"
# 清掉当前终端里可能遗留的其他 ROS2 工作区覆盖层，避免诊断到旧工程的包。
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH
source /opt/ros/humble/setup.bash
source install/setup.bash 2>/dev/null || true
set -u

exec python3 src/slam_nav_bringup/scripts/diagnose_runtime.py "$@"
