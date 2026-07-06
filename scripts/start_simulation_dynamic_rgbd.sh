#!/usr/bin/env bash
set -eo pipefail

cd "$(dirname "$0")"

# 顶配仿真场景：动态障碍物 + 导航用 RGB-D 深度相机。
exec "${SCRIPT_DIR}/start_simulation.sh" world:=dynamic enable_nav_rgbd_camera:=true "$@"
