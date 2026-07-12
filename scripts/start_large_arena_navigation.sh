#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}/.."
source "${SCRIPT_DIR}/setup_workspace_env.sh"
set -u

# 大场地稳定入口：保留当前 AMCL 开局定位 + 原始雷达 PointCloud2 投影 scan
# + 故障触发 small_gicp 先验点云重定位与二维回退方案。
aligned_map="${SCRIPT_DIR}/../src/slam_nav_bringup/map/large_arena_aligned.yaml"
aligned_pcd="${SCRIPT_DIR}/../src/FAST_LIO/PCD/large_arena_aligned.pcd"
map_args=()
if [[ -s "${aligned_map}" && -s "${aligned_pcd}" ]]; then
  map_args+=("map:=${aligned_map}" "map_pcd_path:=${aligned_pcd}")
  echo "[large-arena-nav] using aligned map pair: ${aligned_map}, ${aligned_pcd}"
fi
exec ros2 launch slam_nav_bringup large_arena_robust_navigation.launch.py \
  use_sim_time:=true \
  rviz:=true \
  scan_cloud_topic:=/livox/lidar/pointcloud \
  "${map_args[@]}" \
  "$@"
