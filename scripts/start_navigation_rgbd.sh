#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}/.."
source "${SCRIPT_DIR}/setup_workspace_env.sh"
set -u

params_file="$(ros2 pkg prefix slam_nav_bringup)/share/slam_nav_bringup/config/nav2_params_3d_rgbd.yaml"

exec ros2 launch slam_nav_bringup navigation_3d.launch.py \
  use_sim_time:=true \
  rviz:=true \
  localization_mode:=static \
  enable_rgbd_nav:=true \
  navigation_params_file:="${params_file}" \
  "$@"
