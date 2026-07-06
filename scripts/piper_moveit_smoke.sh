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
    # 冒烟测试优先使用 Piper 专用本地 MoveIt2 插件，避免要求必须 sudo 安装。
    source "${piper_moveit_local_setup}"
  fi
done

set -u

LOG_DIR="${WORKSPACE_DIR}/log"
mkdir -p "${LOG_DIR}"
LOG_FILE="${LOG_DIR}/piper_moveit_smoke_$(date +%Y%m%d_%H%M%S).log"
LAUNCH_PID=""
STARTED_LAUNCH=0

cleanup() {
  if [[ "${STARTED_LAUNCH}" == "1" && -n "${LAUNCH_PID}" ]]; then
    echo "[Piper Smoke] 清理本次启动的 MoveIt2 plan-only 进程..."
    kill -TERM "-${LAUNCH_PID}" 2>/dev/null || kill -TERM "${LAUNCH_PID}" 2>/dev/null || true
    sleep 2
    kill -KILL "-${LAUNCH_PID}" 2>/dev/null || kill -KILL "${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

service_ready() {
  ros2 service list 2>/dev/null | grep -qx '/piper/plan_kinematic_path'
}

if service_ready; then
  echo "[Piper Smoke] 检测到已有 /piper/plan_kinematic_path，复用当前 MoveIt2。"
else
  echo "[Piper Smoke] 启动 Piper 项目侧 MoveIt2 plan-only，日志：${LOG_FILE}"
  setsid ros2 launch slam_nav_piper_moveit_config piper_project_moveit_plan.launch.py \
    >"${LOG_FILE}" 2>&1 &
  LAUNCH_PID="$!"
  STARTED_LAUNCH=1

  deadline=$((SECONDS + 90))
  while ! service_ready; do
    if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
      echo "[Piper Smoke] MoveIt2 启动进程提前退出，最后 80 行日志如下：" >&2
      tail -n 80 "${LOG_FILE}" >&2 || true
      exit 2
    fi
    if (( SECONDS >= deadline )); then
      echo "[Piper Smoke] 等待 /piper/plan_kinematic_path 超时，最后 80 行日志如下：" >&2
      tail -n 80 "${LOG_FILE}" >&2 || true
      exit 2
    fi
    sleep 2
  done
fi

echo "[Piper Smoke] 发送一次 plan-only 规划请求，不执行轨迹。"
ros2 run slam_nav_piper_moveit_config piper_plan_smoke_test.py "$@"
