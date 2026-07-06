#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${WORKSPACE_DIR}"

source "${SCRIPT_DIR}/setup_workspace_env.sh"
set -u

# 独立 ROS domain，验证真实 pick 路径必须先通过手眼标定验收。
export ROS_DOMAIN_ID="${PIPER_HAND_EYE_GATE_ROS_DOMAIN_ID:-87}"

LOG_DIR="${WORKSPACE_DIR}/log"
mkdir -p "${LOG_DIR}"
CONTROL_LOG="${LOG_DIR}/piper_hand_eye_gate_control_$(date +%Y%m%d_%H%M%S).log"
TASK_LOG="${LOG_DIR}/piper_hand_eye_gate_task_$(date +%Y%m%d_%H%M%S).log"
CONTROL_PID=""
TASK_PID=""

cleanup() {
  for pid in "${TASK_PID}" "${CONTROL_PID}"; do
    if [[ -n "${pid}" ]]; then
      kill -TERM "-${pid}" 2>/dev/null || kill -TERM "${pid}" 2>/dev/null || true
    fi
  done
  sleep 2
  for pid in "${TASK_PID}" "${CONTROL_PID}"; do
    if [[ -n "${pid}" ]]; then
      kill -KILL "-${pid}" 2>/dev/null || kill -KILL "${pid}" 2>/dev/null || true
      wait "${pid}" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT INT TERM

echo "[Piper HandEye Gate] 使用 ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "[Piper HandEye Gate] 启动控制桥，日志：${CONTROL_LOG}"
setsid ros2 launch slam_nav_piper_control piper_control.launch.py \
  use_sim_time:=false \
  backend:=moveit \
  initial_owner:=moveit \
  auto_enable:=true \
  >"${CONTROL_LOG}" 2>&1 &
CONTROL_PID="$!"

echo "[Piper HandEye Gate] 启动真实后端声明但未标定的任务层，日志：${TASK_LOG}"
setsid ros2 launch slam_nav_piper_manipulation piper_manipulation.launch.py \
  use_sim_time:=false \
  fake_execution:=false \
  real_backend_connected:=true \
  publish_base_stop:=false \
  hand_eye_calibrated:=false \
  >"${TASK_LOG}" 2>&1 &
TASK_PID="$!"

set +e
python3 - <<'PY'
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from slam_nav_piper_interfaces.action import PickObject
from std_msgs.msg import String


class PiperHandEyeGateSmoke(Node):
    """验证真实 pick 路径缺少手眼标定时必须安全拒绝。"""

    def __init__(self):
        super().__init__('piper_hand_eye_gate_smoke_client')
        self.latest_state = {}
        self.create_subscription(String, '/piper/control/state', self.state_cb, 10)
        self.pick_client = ActionClient(self, PickObject, '/piper/task/pick_object')

    def state_cb(self, msg):
        parsed = {}
        for item in msg.data.split(';'):
            if '=' not in item:
                continue
            key, value = item.split('=', 1)
            parsed[key.strip()] = value.strip()
        self.latest_state = parsed

    def wait_control_ready(self):
        deadline = time.time() + 20.0
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            if (
                self.latest_state.get('owner') == 'moveit'
                and self.latest_state.get('enabled') == 'true'
                and self.latest_state.get('estop') == 'false'
            ):
                self.get_logger().info(f'控制桥已处于真实 pick 可检查状态：{self.latest_state}')
                return True
        self.get_logger().error(f'等待控制桥 ready 超时，最后状态：{self.latest_state}')
        return False

    @staticmethod
    def make_pose():
        pose = PoseStamped()
        pose.header.frame_id = 'piper_base_link'
        pose.pose.position.x = 0.30
        pose.pose.position.y = 0.00
        pose.pose.position.z = 0.22
        pose.pose.orientation.w = 1.0
        return pose

    def run(self):
        if not self.wait_control_ready():
            return 2
        if not self.pick_client.wait_for_server(timeout_sec=20.0):
            self.get_logger().error('等待 /piper/task/pick_object action server 超时。')
            return 2

        goal = PickObject.Goal()
        goal.object_id = 'hand_eye_gate_target'
        goal.object_class = 'smoke'
        goal.target_pose = self.make_pose()
        goal.allow_redetect = False
        goal.approach_distance_m = 0.10
        goal.gripper_width_m = 0.06

        future = self.pick_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if not future.done() or future.result() is None:
            self.get_logger().error('pick goal 发送超时。')
            return 2
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('pick goal 不应在发送阶段被拒绝；应由任务层返回安全失败。')
            return 2

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=20.0)
        if not result_future.done() or result_future.result() is None:
            self.get_logger().error('pick result 等待超时。')
            return 2

        wrapped = result_future.result()
        result = wrapped.result
        if wrapped.status != GoalStatus.STATUS_ABORTED:
            self.get_logger().error(f'pick action 应 ABORT，实际 status={wrapped.status}。')
            return 2
        if result.success:
            self.get_logger().error(f'pick action 不应成功：{result.message}')
            return 2
        if '手眼标定' not in result.message:
            self.get_logger().error(f'pick action 拒绝原因应包含手眼标定，实际：{result.message}')
            return 2

        self.get_logger().info(f'真实 pick 缺少手眼标定时按预期拒绝：{result.message}')
        return 0


def main():
    rclpy.init()
    node = PiperHandEyeGateSmoke()
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
  echo "[Piper HandEye Gate] 烟测失败，控制桥日志最后 80 行：" >&2
  tail -n 80 "${CONTROL_LOG}" >&2 || true
  echo "[Piper HandEye Gate] 任务层日志最后 120 行：" >&2
  tail -n 120 "${TASK_LOG}" >&2 || true
  exit "${SMOKE_STATUS}"
fi

echo "[Piper HandEye Gate] 手眼标定真实 pick 门禁烟测通过。"
