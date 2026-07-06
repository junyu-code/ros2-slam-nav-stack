#!/usr/bin/env bash
# 清理可能来自其他 ROS2 工作区的 overlay，避免插件库和 Python 包串到当前工程。
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}/.."

unset AMENT_PREFIX_PATH
unset COLCON_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset ROS_PACKAGE_PATH
unset LD_LIBRARY_PATH
unset PYTHONPATH
unset GAZEBO_PLUGIN_PATH
unset GAZEBO_MODEL_PATH
unset GAZEBO_RESOURCE_PATH

source /opt/ros/humble/setup.bash
if [[ -f install/setup.bash ]]; then
  source install/setup.bash
fi
