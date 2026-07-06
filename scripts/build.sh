#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}/.."
# 清掉当前终端里可能遗留的其他 ROS2 工作区覆盖层，避免混入旧工程的包和插件库。
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH
unset LD_LIBRARY_PATH PYTHONPATH GAZEBO_PLUGIN_PATH GAZEBO_MODEL_PATH GAZEBO_RESOURCE_PATH
source /opt/ros/humble/setup.bash
set -u

# WSL 下优先保证稳定，需要提速时可以手动把两个 1 改成 2。
export MAKEFLAGS="${MAKEFLAGS:--j1}"
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-1}"

colcon build --symlink-install \
  --executor sequential \
  --parallel-workers "${COLCON_WORKERS:-1}" \
  --event-handlers console_direct+ \
  --cmake-args -DBUILD_TESTING=OFF
