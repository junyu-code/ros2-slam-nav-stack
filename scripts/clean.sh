#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}/.."

DRY_RUN=false
if [[ "${1:-}" == "--dry-run" ]]; then
  DRY_RUN=true
fi

# 清理本工作区常见的 ROS/Gazebo/RViz/Nav2 残留进程。
PATTERN="slam_nav_simulation|slam_nav_bringup|fastlio_mapping|livox_ros_driver2_node|complementary_filter_node|async_slam_toolbox_node|pointcloud_to_laserscan_node|adaptive_cloud_filter|localization_guard_node.py|robot_state_publisher|joint_state_publisher|spawn_entity.py|gzserver|gzclient|rviz2|nav2_|map_server|amcl|controller_server|planner_server|smoother_server|behavior_server|bt_navigator|waypoint_follower|velocity_smoother|lifecycle_manager|publish_initial_pose.py|mission_behavior_node.py|safe_cmd_bridge_node.py"

if [[ "${DRY_RUN}" == "true" ]]; then
  echo "[clean] dry-run: would terminate matching processes:"
  pgrep -af "$PATTERN" || echo "[clean] dry-run: no matching process"
  echo "[clean] dry-run: would stop ROS daemon"
  echo "[clean] dry-run: would remove FastDDS/FastRTPS shared-memory leftovers:"
  find /dev/shm -maxdepth 1 \( \
    -name 'fastrtps_*' -o \
    -name 'fastdds_*' -o \
    -name 'sem.fastrtps_*' -o \
    -name 'sem.fastdds_*' \
  \) -print 2>/dev/null || true
  echo "[clean] dry-run done"
  exit 0
fi

pkill -TERM -f "$PATTERN" 2>/dev/null || true
sleep 1
pkill -KILL -f "$PATTERN" 2>/dev/null || true

ros2 daemon stop 2>/dev/null || true

# FastDDS/FastRTPS 在 WSL 或异常退出后可能留下共享内存锁。
# 这些文件只能在停止 ROS/Gazebo 之后清理，避免影响正在通信的节点。
rm -f \
  /dev/shm/fastrtps_* \
  /dev/shm/fastdds_* \
  /dev/shm/sem.fastrtps_* \
  /dev/shm/sem.fastdds_* \
  2>/dev/null || true

echo "[clean] done"
