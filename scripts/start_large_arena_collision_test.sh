#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 大场地碰撞扰动测试：自动导航车 + /manual_car/cmd_vel 手动干扰车。
exec "${SCRIPT_DIR}/start_simulation.sh" \
  world:=large_arena_collision \
  enable_nav_rgbd_camera:=true \
  spawn_x:=6.95 \
  spawn_y:=3.90 \
  spawn_z:=0.06 \
  spawn_yaw:=1.5708 \
  "$@"
