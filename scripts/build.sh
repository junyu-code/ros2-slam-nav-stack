#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}/.."
# 清掉当前终端里可能遗留的其他 ROS2 工作区覆盖层，避免混入旧工程的包和插件库。
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH
unset LD_LIBRARY_PATH PYTHONPATH GAZEBO_PLUGIN_PATH GAZEBO_MODEL_PATH GAZEBO_RESOURCE_PATH
source /opt/ros/humble/setup.bash
set -u

# 根据 CPU 和可用内存自适应编译并发；需要手动覆盖时可设置 BUILD_JOBS/COLCON_WORKERS/CMAKE_BUILD_PARALLEL_LEVEL。
cpu_count="$(nproc 2>/dev/null || echo 1)"
mem_available_mb="$(awk '/MemAvailable/ {print int($2 / 1024)}' /proc/meminfo 2>/dev/null || echo 2048)"

adaptive_total_jobs="${BUILD_JOBS:-}"
if [[ -z "${adaptive_total_jobs}" ]]; then
  mem_limited_jobs=$((mem_available_mb / 1800))
  if (( mem_limited_jobs < 1 )); then
    mem_limited_jobs=1
  fi
  adaptive_total_jobs="${cpu_count}"
  if (( mem_limited_jobs < adaptive_total_jobs )); then
    adaptive_total_jobs="${mem_limited_jobs}"
  fi
fi
if (( adaptive_total_jobs < 1 )); then
  adaptive_total_jobs=1
fi

default_colcon_workers=$((mem_available_mb / 3500))
if (( default_colcon_workers < 1 )); then
  default_colcon_workers=1
fi
if (( default_colcon_workers > adaptive_total_jobs )); then
  default_colcon_workers="${adaptive_total_jobs}"
fi
if (( default_colcon_workers > 4 )); then
  default_colcon_workers=4
fi

export COLCON_WORKERS="${COLCON_WORKERS:-${default_colcon_workers}}"
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-$(((adaptive_total_jobs + COLCON_WORKERS - 1) / COLCON_WORKERS))}"
export MAKEFLAGS="${MAKEFLAGS:--j${CMAKE_BUILD_PARALLEL_LEVEL}}"

echo "[build] CPU=${cpu_count}, MemAvailable=${mem_available_mb}MB, total_jobs=${adaptive_total_jobs}, colcon_workers=${COLCON_WORKERS}, cmake_jobs=${CMAKE_BUILD_PARALLEL_LEVEL}"

colcon build --symlink-install \
  --parallel-workers "${COLCON_WORKERS}" \
  --event-handlers console_direct+ \
  --cmake-args -DBUILD_TESTING=OFF
