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
    # 只给 Piper MoveIt2 plan-only 冒烟测试叠加本地插件，不影响 task1 默认流程。
    source "${piper_moveit_local_setup}"
  fi
done

set -u
exec ros2 run slam_nav_piper_moveit_config piper_plan_smoke_test.py "$@"
