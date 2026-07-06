#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${WORKSPACE_DIR}"

source "${SCRIPT_DIR}/setup_workspace_env.sh"
set -u

# 使用独立 ROS domain 和 Gazebo master，避免影响正在运行的 task1/Gazebo 仿真。
export ROS_DOMAIN_ID="${PIPER_GAZEBO_SMOKE_ROS_DOMAIN_ID:-78}"
export GAZEBO_MASTER_URI="${PIPER_GAZEBO_SMOKE_MASTER_URI:-http://127.0.0.1:11357}"

LOG_DIR="${WORKSPACE_DIR}/log"
mkdir -p "${LOG_DIR}"
LOG_FILE="${LOG_DIR}/piper_gazebo_smoke_$(date +%Y%m%d_%H%M%S).log"
LAUNCH_PID=""

cleanup() {
  if [[ -n "${LAUNCH_PID}" ]]; then
    echo "[Piper Gazebo] 清理本次启动的 headless Gazebo 进程..."
    kill -TERM "-${LAUNCH_PID}" 2>/dev/null || kill -TERM "${LAUNCH_PID}" 2>/dev/null || true
    sleep 2
    kill -KILL "-${LAUNCH_PID}" 2>/dev/null || kill -KILL "${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

echo "[Piper Gazebo] 使用 ROS_DOMAIN_ID=${ROS_DOMAIN_ID}, GAZEBO_MASTER_URI=${GAZEBO_MASTER_URI}"
echo "[Piper Gazebo] 启动移动底盘 + 官方 Piper headless 仿真，日志：${LOG_FILE}"
setsid ros2 launch slam_nav_simulation simulation.launch.py \
  use_sim_time:=true \
  gui:=false \
  world:=static \
  enable_piper_arm:=true \
  piper_arm_model:=official \
  "$@" >"${LOG_FILE}" 2>&1 &
LAUNCH_PID="$!"

set +e
python3 - <<'PY'
import subprocess
import sys
import time

import rclpy
from gazebo_msgs.srv import GetModelList


def command(args, timeout=8.0):
    return subprocess.run(
        args,
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=timeout,
    )


def wait_for_official_description(timeout_s=90.0):
    deadline = time.time() + timeout_s
    last_output = ''
    while time.time() < deadline:
        result = command(['ros2', 'param', 'get', '/robot_state_publisher', 'robot_description'])
        last_output = result.stdout
        if result.returncode == 0:
            has_official_chain = all(
                name in result.stdout
                for name in ('piper_base_link', 'piper_joint1', 'piper_joint8', 'piper_arm_camera_optical_frame')
            )
            has_placeholder_chain = 'piper_joint1_placeholder' in result.stdout
            if has_official_chain and not has_placeholder_chain:
                print('[Piper Gazebo] robot_description 已加载官方 Piper 适配链。')
                return
        time.sleep(2.0)
    print('[Piper Gazebo] 等待官方 Piper robot_description 超时。最后一次输出：', file=sys.stderr)
    print(last_output[-2000:], file=sys.stderr)
    raise SystemExit(2)


def check_gazebo_entity(timeout_s=180.0):
    rclpy.init()
    node = rclpy.create_node('piper_gazebo_smoke_client')
    client = node.create_client(GetModelList, '/get_model_list')
    try:
        if not client.wait_for_service(timeout_sec=timeout_s):
            print('[Piper Gazebo] 等待 /get_model_list 超时。', file=sys.stderr)
            raise SystemExit(2)

        deadline = time.time() + timeout_s
        model_names = []
        while time.time() < deadline:
            request = GetModelList.Request()
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
            if future.done():
                response = future.result()
                if response is not None and response.success:
                    model_names = list(response.model_names)
                    if 'mobile_robot' in model_names:
                        print('[Piper Gazebo] Gazebo 已生成 mobile_robot 实体。')
                        return
            time.sleep(2.0)

        if 'mobile_robot' not in model_names:
            print(f'[Piper Gazebo] Gazebo 中未找到 mobile_robot，当前模型：{model_names}', file=sys.stderr)
            raise SystemExit(2)
    finally:
        node.destroy_node()
        rclpy.shutdown()


wait_for_official_description()
check_gazebo_entity()
print('[Piper Gazebo] headless Gazebo + 官方 Piper 适配链烟测通过。')
PY
SMOKE_STATUS="$?"
set -e

if [[ "${SMOKE_STATUS}" -ne 0 ]]; then
  echo "[Piper Gazebo] 烟测失败，最后 100 行 Gazebo 日志如下：" >&2
  tail -n 100 "${LOG_FILE}" >&2 || true
  exit "${SMOKE_STATUS}"
fi
