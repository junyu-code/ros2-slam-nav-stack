#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${WORKSPACE_DIR}"

source "${SCRIPT_DIR}/setup_workspace_env.sh"
set -u

# 独立 ROS domain，验证实机入口默认安全拒绝，不连接 SDK 或真实 MoveIt2 执行后端。
export ROS_DOMAIN_ID="${PIPER_REAL_DRY_RUN_ROS_DOMAIN_ID:-85}"

LOG_DIR="${WORKSPACE_DIR}/log"
mkdir -p "${LOG_DIR}"
LOG_FILE="${LOG_DIR}/piper_real_dry_run_$(date +%Y%m%d_%H%M%S).log"
LAUNCH_PID=""

cleanup() {
  if [[ -n "${LAUNCH_PID}" ]]; then
    echo "[Piper Real DryRun] 清理本次启动的实机 dry-run 进程..."
    kill -TERM "-${LAUNCH_PID}" 2>/dev/null || kill -TERM "${LAUNCH_PID}" 2>/dev/null || true
    sleep 2
    kill -KILL "-${LAUNCH_PID}" 2>/dev/null || kill -KILL "${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

echo "[Piper Real DryRun] 使用 ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "[Piper Real DryRun] 启动 Piper 实机入口默认安全配置，日志：${LOG_FILE}"
setsid ros2 launch slam_nav_piper_bringup piper_real.launch.py \
  use_sim_time:=false \
  backend:=moveit \
  arm_model:=official \
  real_backend_connected:=false \
  "$@" >"${LOG_FILE}" 2>&1 &
LAUNCH_PID="$!"

set +e
python3 - <<'PY'
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from slam_nav_piper_interfaces.action import PickObject, PlaceObject
from std_msgs.msg import String
from std_srvs.srv import Trigger


class PiperRealDryRun(Node):
    """验证 Piper 实机入口默认不会假装真实执行成功。"""

    EXPECTED_REJECT_TEXT = '真实 MoveIt2/SDK 后端尚未接入'

    def __init__(self):
        super().__init__('piper_real_dry_run_client')
        self.latest_state = {}
        self.create_subscription(String, '/piper/control/state', self.state_cb, 10)
        self.home_client = self.create_client(Trigger, '/piper/control/home')
        self.disable_client = self.create_client(Trigger, '/piper/control/disable')
        self.pick_client = ActionClient(self, PickObject, '/piper/task/pick_object')
        self.place_client = ActionClient(self, PlaceObject, '/piper/task/place_object')

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

    def wait_state(self, label, predicate, timeout_s=15.0):
        deadline = time.time() + timeout_s
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.latest_state and predicate(self.latest_state):
                self.get_logger().info(f'{label} 状态符合预期：{self.latest_state}')
                return True
        self.get_logger().error(f'等待 {label} 状态超时，最后状态：{self.latest_state}')
        return False

    def call_trigger(self, client, label, expect_success):
        if not client.wait_for_service(timeout_sec=15.0):
            self.get_logger().error(f'等待 {label} 服务超时。')
            return False
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

    def make_pose(self):
        pose = PoseStamped()
        pose.header.frame_id = 'piper_base_link'
        pose.pose.position.x = 0.30
        pose.pose.position.y = 0.00
        pose.pose.position.z = 0.20
        pose.pose.orientation.w = 1.0
        return pose

    def send_pick_expect_reject(self):
        goal = PickObject.Goal()
        goal.object_id = 'dry_run_target'
        goal.object_class = 'dry_run'
        goal.target_pose = self.make_pose()
        goal.allow_redetect = False
        goal.approach_distance_m = 0.10
        goal.gripper_width_m = 0.06
        return self.send_goal_expect_reject(self.pick_client, goal, 'pick')

    def send_place_expect_reject(self):
        goal = PlaceObject.Goal()
        goal.object_id = 'dry_run_target'
        goal.target_pose = self.make_pose()
        goal.open_gripper = True
        goal.retreat_distance_m = 0.10
        return self.send_goal_expect_reject(self.place_client, goal, 'place')

    def send_goal_expect_reject(self, client, goal, name):
        if not client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error(f'等待 /piper/task/{name}_object action server 超时。')
            return False
        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if not future.done() or future.result() is None:
            self.get_logger().error(f'{name} goal 发送超时。')
            return False
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'{name} goal 不应在发送阶段被拒绝；应由任务层返回安全失败。')
            return False
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=20.0)
        if not result_future.done() or result_future.result() is None:
            self.get_logger().error(f'{name} result 等待超时。')
            return False
        wrapped = result_future.result()
        result = wrapped.result
        if wrapped.status != GoalStatus.STATUS_ABORTED:
            self.get_logger().error(f'{name} action 状态应为 ABORTED，实际 status={wrapped.status}。')
            return False
        if result.success:
            self.get_logger().error(f'{name} action 不应成功：message={result.message}')
            return False
        if self.EXPECTED_REJECT_TEXT not in result.message:
            self.get_logger().error(f'{name} action 拒绝原因不符合预期：{result.message}')
            return False
        self.get_logger().info(f'{name} action 按预期安全拒绝：{result.message}')
        return True

    def run(self):
        if not self.wait_state(
            '实机入口初始禁用',
            lambda state: (
                state.get('backend') == 'moveit'
                and state.get('owner') == 'disabled'
                and state.get('enabled') == 'false'
                and state.get('estop') == 'false'
            ),
        ):
            return 2

        if not self.call_trigger(self.home_client, '默认禁用状态 home 应失败', expect_success=False):
            return 2
        if not self.send_pick_expect_reject():
            return 2
        if not self.send_place_expect_reject():
            return 2
        if not self.call_trigger(self.disable_client, 'dry-run 结束 disable', expect_success=True):
            return 2

        self.get_logger().info('Piper 实机入口 dry-run 安全拒绝烟测通过。')
        return 0


def main():
    rclpy.init()
    node = PiperRealDryRun()
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
  echo "[Piper Real DryRun] 烟测失败，最后 160 行日志如下：" >&2
  tail -n 160 "${LOG_FILE}" >&2 || true
  exit "${SMOKE_STATUS}"
fi
