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
    # 实机就绪报告会检查 MoveIt2 插件是否可见，但不会启动 move_group 或真实硬件。
    source "${piper_moveit_local_setup}"
  fi
done

set -u
exec ros2 run slam_nav_piper_bringup piper_real_readiness_report.py "$@"
