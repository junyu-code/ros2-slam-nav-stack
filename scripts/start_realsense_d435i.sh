#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}/.."
source "${SCRIPT_DIR}/setup_workspace_env.sh"
set -u

exec ros2 launch realsense2_camera rs_launch.py \
  camera_namespace:=nav_camera \
  camera_name:=d435i \
  device_type:=d435i \
  enable_depth:=true \
  enable_color:=true \
  align_depth.enable:=true \
  pointcloud.enable:=true \
  "$@"
