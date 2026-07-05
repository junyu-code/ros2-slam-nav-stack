#!/usr/bin/env bash
set -eo pipefail

cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash
source install/setup.bash 2>/dev/null || true
set -u

# WSL 下优先保证稳定，需要提速时可以手动把两个 1 改成 2。
export MAKEFLAGS="${MAKEFLAGS:--j1}"
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-1}"

colcon build --symlink-install \
  --executor sequential \
  --parallel-workers "${COLCON_WORKERS:-1}" \
  --event-handlers console_direct+ \
  --cmake-args -DBUILD_TESTING=OFF
