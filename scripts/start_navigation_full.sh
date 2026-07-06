#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}/.."
source "${SCRIPT_DIR}/setup_workspace_env.sh"
set -u

params_file="$(ros2 pkg prefix slam_nav_bringup)/share/slam_nav_bringup/config/nav2_params_3d_rgbd.yaml"

# 顶配导航入口：
# - LiDAR 3D 地形分析与强度体素代价地图
# - RGB-D 松耦合近场障碍点云 /visual_obstacles
# - 行为树后退恢复
# - 定位健康监控与速度安全桥观测
exec ros2 launch slam_nav_bringup robust_navigation.launch.py \
  use_sim_time:=true \
  rviz:=true \
  localization_mode:=static \
  enable_terrain_analysis:=true \
  enable_rgbd_nav:=true \
  params_file:="${params_file}" \
  enable_localization_guard:=true \
  enable_safe_cmd_bridge:=true \
  "$@"
