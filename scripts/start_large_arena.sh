#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 大场地普通仿真：使用大场地 mesh，并把自动导航车放到旧工程常用起点附近。
exec "${SCRIPT_DIR}/start_simulation.sh" \
  world:=large_arena \
  enable_nav_rgbd_camera:=true \
  spawn_x:=6.95 \
  spawn_y:=3.90 \
  spawn_z:=0.06 \
  spawn_yaw:=1.5708 \
  "$@"
