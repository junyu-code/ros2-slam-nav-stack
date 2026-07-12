#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
LOG_DIR="${WORKSPACE_DIR}/log/ui"
mkdir -p "${LOG_DIR}"

# UI 演示入口：先拉起静态 Gazebo，再启动 Nav2/RViz。
# 这样给行外人展示时只需要点一次按钮，不用记住多终端启动顺序。
SIM_LOG="${LOG_DIR}/demo_navigation_sim_$(date +%Y%m%d_%H%M%S).log"
WAIT_SECONDS="${SLAM_NAV_UI_DEMO_NAV_WAIT:-8}"

cleanup() {
  if [[ -n "${SIM_PID:-}" ]]; then
    kill "${SIM_PID}" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT INT TERM

echo "[demo-nav] 启动静态仿真，日志：${SIM_LOG}"
"${SCRIPT_DIR}/start_simulation_static.sh" >"${SIM_LOG}" 2>&1 &
SIM_PID="$!"

echo "[demo-nav] 等待 ${WAIT_SECONDS}s，让 Gazebo 和机器人模型先起来。"
sleep "${WAIT_SECONDS}"

echo "[demo-nav] 启动默认 Nav2 导航。"
"${SCRIPT_DIR}/start_navigation.sh" "$@"
