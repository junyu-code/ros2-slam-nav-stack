#!/usr/bin/env bash
set -eo pipefail

cd "$(dirname "$0")"
# 清掉当前终端里可能遗留的其他 ROS2 工作区覆盖层，避免混入旧工程的包。
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH
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
