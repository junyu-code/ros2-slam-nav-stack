#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}/.."
source "${SCRIPT_DIR}/setup_workspace_env.sh"
set -u

# 大场地稳定入口：保留当前 AMCL 开局定位 + 原始雷达 PointCloud2 投影 scan
# + 故障触发 GICP 先验点云重定位方案。
exec ros2 launch slam_nav_bringup large_arena_robust_navigation.launch.py \
  use_sim_time:=true \
  rviz:=true \
  scan_cloud_topic:=/livox/lidar/pointcloud \
  "$@"
