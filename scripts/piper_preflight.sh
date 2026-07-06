#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}/.."
source "${SCRIPT_DIR}/setup_workspace_env.sh"

for piper_moveit_local_setup in \
  "${SCRIPT_DIR}/../external/ros_humble_debs/overlay/opt/ros/humble/share/moveit_planners_ompl/local_setup.bash" \
  "${SCRIPT_DIR}/../external/ros_humble_debs/overlay/opt/ros/humble/share/moveit_simple_controller_manager/local_setup.bash"
do
  if [[ -f "${piper_moveit_local_setup}" ]]; then
    # 预检时也加载 Piper 专用本地 MoveIt2 插件，避免误报系统未 sudo 安装。
    source "${piper_moveit_local_setup}"
  fi
done

set -u
exec ros2 run slam_nav_piper_bringup piper_preflight_check.py "$@"
