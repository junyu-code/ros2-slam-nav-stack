#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${WORKSPACE_DIR}"

source "${SCRIPT_DIR}/setup_workspace_env.sh"
set -u

# 独立 ROS domain，验证 mission_behavior 只通过 /piper/task/* action 调用机械臂任务。
export ROS_DOMAIN_ID="${PIPER_MISSION_DEMO_SMOKE_ROS_DOMAIN_ID:-90}"

LOG_DIR="${WORKSPACE_DIR}/log"
mkdir -p "${LOG_DIR}"
PIPER_LOG="${LOG_DIR}/piper_mission_demo_runtime_$(date +%Y%m%d_%H%M%S).log"
MISSION_LOG="${LOG_DIR}/piper_mission_demo_client_$(date +%Y%m%d_%H%M%S).log"
PIPER_PID=""

cleanup() {
  if [[ -n "${PIPER_PID}" ]]; then
    echo "[Piper Mission Demo] 清理本次启动的 Piper runtime 进程..."
    kill -TERM "-${PIPER_PID}" 2>/dev/null || kill -TERM "${PIPER_PID}" 2>/dev/null || true
    sleep 2
    kill -KILL "-${PIPER_PID}" 2>/dev/null || kill -KILL "${PIPER_PID}" 2>/dev/null || true
    wait "${PIPER_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

echo "[Piper Mission Demo] 使用 ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "[Piper Mission Demo] 启动 Piper fake runtime，日志：${PIPER_LOG}"
setsid ros2 launch slam_nav_piper_bringup piper_sim.launch.py \
  use_sim_time:=false \
  publish_joint_states:=true \
  >"${PIPER_LOG}" 2>&1 &
PIPER_PID="$!"

echo "[Piper Mission Demo] 启动 mission_behavior Piper demo，日志：${MISSION_LOG}"
set +e
timeout 45s ros2 launch mission_behavior piper_pick_place_demo.launch.py \
  use_sim_time:=false \
  auto_start:=true \
  target_frame:=piper_base_link \
  target_x:=0.30 \
  target_y:=0.0 \
  target_z:=0.22 \
  >"${MISSION_LOG}" 2>&1
MISSION_STATUS="$?"
set -e

if [[ "${MISSION_STATUS}" -ne 0 ]]; then
  echo "[Piper Mission Demo] mission_behavior demo 失败，Piper 日志最后 120 行：" >&2
  tail -n 120 "${PIPER_LOG}" >&2 || true
  echo "[Piper Mission Demo] mission 日志最后 160 行：" >&2
  tail -n 160 "${MISSION_LOG}" >&2 || true
  exit "${MISSION_STATUS}"
fi

if ! grep -q 'pick/place demo 完成' "${MISSION_LOG}"; then
  echo "[Piper Mission Demo] mission 日志缺少完成标记。" >&2
  tail -n 160 "${MISSION_LOG}" >&2 || true
  exit 2
fi

echo "[Piper Mission Demo] mission_behavior -> /piper/task/* action 烟测通过。"
