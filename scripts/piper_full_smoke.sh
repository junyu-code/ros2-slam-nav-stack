#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${WORKSPACE_DIR}"

source "${SCRIPT_DIR}/setup_workspace_env.sh"

for piper_moveit_local_setup in \
  "${WORKSPACE_DIR}/external/ros_humble_debs/overlay/opt/ros/humble/share/moveit_planners_ompl/local_setup.bash" \
  "${WORKSPACE_DIR}/external/ros_humble_debs/overlay/opt/ros/humble/share/moveit_simple_controller_manager/local_setup.bash"
do
  if [[ -f "${piper_moveit_local_setup}" ]]; then
    # 全链路烟测同样加载 Piper 专用 MoveIt2 本地插件，保持无需 sudo 的验证路径。
    source "${piper_moveit_local_setup}"
  fi
done

set -u

echo "[Piper Full] 1/20 安全配置检查..."
"${SCRIPT_DIR}/piper_safety_check.sh"

echo
echo "[Piper Full] 2/20 边界回归检查..."
"${SCRIPT_DIR}/piper_boundary_check.sh"

echo
echo "[Piper Full] 3/20 GitHub 体积和外部数据边界检查..."
"${SCRIPT_DIR}/piper_repo_size_check.sh"

echo
echo "[Piper Full] 4/20 依赖预检和官方包检查..."
"${SCRIPT_DIR}/piper_preflight.sh" --require-official

echo
echo "[Piper Full] 5/20 官方 URDF -> 项目侧 piper_* frame 审计..."
"${SCRIPT_DIR}/piper_official_frame_audit.sh"

echo
echo "[Piper Full] 6/20 项目侧 MoveIt2 配置映射审计..."
"${SCRIPT_DIR}/piper_moveit_config_audit.sh"

echo
echo "[Piper Full] 7/20 Piper 手眼标定配置边界检查..."
"${SCRIPT_DIR}/piper_hand_eye_check.sh"

echo
echo "[Piper Full] 8/20 Piper 真实 pick 手眼标定门禁烟测..."
"${SCRIPT_DIR}/piper_hand_eye_gate_smoke.sh"

echo
echo "[Piper Full] 9/20 Piper 真实运动底盘停止门禁烟测..."
"${SCRIPT_DIR}/piper_base_stop_gate_smoke.sh"

echo
echo "[Piper Full] 10/20 Piper 运行时 TF 链烟测..."
"${SCRIPT_DIR}/piper_tf_smoke.sh"

echo
echo "[Piper Full] 11/20 Piper runtime 命名空间边界烟测..."
"${SCRIPT_DIR}/piper_namespace_smoke.sh"

echo
echo "[Piper Full] 12/20 Piper 控制桥安全边界烟测..."
"${SCRIPT_DIR}/piper_control_smoke.sh"

echo
echo "[Piper Full] 13/20 Piper 实机入口 dry-run 安全拒绝烟测..."
"${SCRIPT_DIR}/piper_real_dry_run.sh"

echo
echo "[Piper Full] 14/20 Headless Gazebo 官方 Piper 组合模型烟测..."
"${SCRIPT_DIR}/piper_gazebo_smoke.sh"

echo
echo "[Piper Full] 15/20 Fake RGB-D -> grasp candidates -> pick/place action 烟测..."
"${SCRIPT_DIR}/piper_task_smoke.sh"

echo
echo "[Piper Full] 16/20 Piper 移动操作组合入口烟测..."
"${SCRIPT_DIR}/piper_mobile_sequence_smoke.sh"

echo
echo "[Piper Full] 17/20 mission_behavior 到 Piper task action 烟测..."
"${SCRIPT_DIR}/piper_mission_demo_smoke.sh"

echo
echo "[Piper Full] 18/20 Piper 学习层抓取候选排序烟测..."
"${SCRIPT_DIR}/piper_learning_smoke.sh"

echo
echo "[Piper Full] 19/20 任务层 ranked 候选消费门禁烟测..."
"${SCRIPT_DIR}/piper_ranked_candidate_gate_smoke.sh"

echo
echo "[Piper Full] 20/20 MoveIt2 plan-only 烟测..."
ROS_DOMAIN_ID="${PIPER_FULL_SMOKE_MOVEIT_DOMAIN_ID:-81}" \
  "${SCRIPT_DIR}/piper_moveit_smoke.sh"

echo
echo "[Piper Full] Piper 全链路烟测通过。"
