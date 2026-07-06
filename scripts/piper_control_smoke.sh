#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${WORKSPACE_DIR}"

source "${SCRIPT_DIR}/setup_workspace_env.sh"
set -u

# 独立 ROS domain，避免和正在运行的 task1 导航/仿真互相发现。
export ROS_DOMAIN_ID="${PIPER_CONTROL_SMOKE_ROS_DOMAIN_ID:-82}"

LOG_DIR="${WORKSPACE_DIR}/log"
mkdir -p "${LOG_DIR}"
LOG_FILE="${LOG_DIR}/piper_control_smoke_$(date +%Y%m%d_%H%M%S).log"
LAUNCH_PID=""

cleanup() {
  if [[ -n "${LAUNCH_PID}" ]]; then
    echo "[Piper Control] 清理本次启动的控制桥进程..."
    kill -TERM "-${LAUNCH_PID}" 2>/dev/null || kill -TERM "${LAUNCH_PID}" 2>/dev/null || true
    sleep 2
    kill -KILL "-${LAUNCH_PID}" 2>/dev/null || kill -KILL "${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

echo "[Piper Control] 使用 ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "[Piper Control] 启动 Piper 控制桥安全边界，日志：${LOG_FILE}"
setsid ros2 launch slam_nav_piper_control piper_control.launch.py \
  use_sim_time:=false \
  backend:=moveit \
  initial_owner:=disabled \
  auto_enable:=false \
  "$@" >"${LOG_FILE}" 2>&1 &
LAUNCH_PID="$!"

set +e
python3 - <<'PY'
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


class PiperControlSmoke(Node):
    """验证 Piper 控制桥的 owner、使能和急停边界。"""

    def __init__(self):
        super().__init__('piper_control_smoke_client')
        self.latest_state = {}
        self.owner_pub = self.create_publisher(String, '/piper/control/owner_request', 10)
        self.create_subscription(String, '/piper/control/state', self.state_cb, 10)
        self.enable_client = self.create_client(Trigger, '/piper/control/enable')
        self.disable_client = self.create_client(Trigger, '/piper/control/disable')
        self.estop_client = self.create_client(Trigger, '/piper/control/estop')
        self.clear_estop_client = self.create_client(Trigger, '/piper/control/clear_estop')
        self.home_client = self.create_client(Trigger, '/piper/control/home')

    def state_cb(self, msg):
        self.latest_state = self.parse_state(msg.data)

    @staticmethod
    def parse_state(text):
        state = {}
        for item in text.split(';'):
            if '=' not in item:
                continue
            key, value = item.split('=', 1)
            state[key.strip()] = value.strip()
        return state

    def wait_for_service(self, client, label):
        if client.wait_for_service(timeout_sec=15.0):
            self.get_logger().info(f'{label} 服务已就绪。')
            return True
        self.get_logger().error(f'等待 {label} 服务超时。')
        return False

    def wait_state(self, label, predicate, timeout_s=8.0):
        deadline = time.time() + timeout_s
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.latest_state and predicate(self.latest_state):
                self.get_logger().info(f'{label} 状态符合预期：{self.latest_state}')
                return True
        self.get_logger().error(f'等待 {label} 状态超时，最后状态：{self.latest_state}')
        return False

    def call_trigger(self, client, label, expect_success=True):
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=8.0)
        if not future.done() or future.result() is None:
            self.get_logger().error(f'{label} 调用超时。')
            return False
        response = future.result()
        if bool(response.success) != bool(expect_success):
            self.get_logger().error(
                f'{label} 返回不符合预期：success={response.success}, message={response.message}'
            )
            return False
        self.get_logger().info(f'{label} 返回：success={response.success}, message={response.message}')
        return True

    def publish_owner(self, owner):
        msg = String()
        msg.data = owner
        # 发布几次，避免启动初期发现阶段丢第一包。
        for _ in range(5):
            self.owner_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)

    def run(self):
        services = [
            (self.enable_client, '/piper/control/enable'),
            (self.disable_client, '/piper/control/disable'),
            (self.estop_client, '/piper/control/estop'),
            (self.clear_estop_client, '/piper/control/clear_estop'),
            (self.home_client, '/piper/control/home'),
        ]
        for client, label in services:
            if not self.wait_for_service(client, label):
                return 2

        if not self.wait_state(
            '初始 disabled',
            lambda state: (
                state.get('backend') == 'moveit'
                and state.get('owner') == 'disabled'
                and state.get('enabled') == 'false'
                and state.get('estop') == 'false'
            ),
        ):
            return 2

        if not self.call_trigger(self.enable_client, 'enable'):
            return 2
        self.publish_owner('moveit')
        if not self.wait_state(
            'MoveIt owner 已持有',
            lambda state: state.get('owner') == 'moveit' and state.get('enabled') == 'true',
        ):
            return 2

        if not self.call_trigger(self.estop_client, 'estop'):
            return 2
        if not self.wait_state(
            '急停后失能',
            lambda state: (
                state.get('owner') == 'disabled'
                and state.get('enabled') == 'false'
                and state.get('estop') == 'true'
            ),
        ):
            return 2

        self.publish_owner('moveit')
        if not self.wait_state('急停状态拒绝 owner 切换', lambda state: state.get('owner') == 'disabled'):
            return 2

        if not self.call_trigger(self.enable_client, '急停中 enable 应失败', expect_success=False):
            return 2
        if not self.call_trigger(self.clear_estop_client, 'clear_estop'):
            return 2
        if not self.call_trigger(self.disable_client, 'disable'):
            return 2
        if not self.wait_state(
            '最终 disabled',
            lambda state: (
                state.get('owner') == 'disabled'
                and state.get('enabled') == 'false'
                and state.get('estop') == 'false'
            ),
        ):
            return 2

        self.get_logger().info('Piper 控制桥安全边界烟测通过。')
        return 0


def main():
    rclpy.init()
    node = PiperControlSmoke()
    try:
        exit_code = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
PY
SMOKE_STATUS="$?"
set -e

if [[ "${SMOKE_STATUS}" -ne 0 ]]; then
  echo "[Piper Control] 烟测失败，最后 120 行日志如下：" >&2
  tail -n 120 "${LOG_FILE}" >&2 || true
  exit "${SMOKE_STATUS}"
fi
