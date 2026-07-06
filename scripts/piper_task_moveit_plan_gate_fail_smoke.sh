#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${WORKSPACE_DIR}"

source "${SCRIPT_DIR}/setup_workspace_env.sh"
set -u

# 独立 ROS domain，不启动 MoveIt2；验证打开 plan-only 门禁后不会继续 fake 成功。
export ROS_DOMAIN_ID="${PIPER_TASK_MOVEIT_GATE_FAIL_ROS_DOMAIN_ID:-93}"

LOG_DIR="${WORKSPACE_DIR}/log"
mkdir -p "${LOG_DIR}"
TASK_LOG="${LOG_DIR}/piper_task_moveit_gate_fail_$(date +%Y%m%d_%H%M%S).log"
TASK_PID=""

cleanup() {
  if [[ -n "${TASK_PID}" ]]; then
    kill -TERM "-${TASK_PID}" 2>/dev/null || kill -TERM "${TASK_PID}" 2>/dev/null || true
    sleep 2
    kill -KILL "-${TASK_PID}" 2>/dev/null || kill -KILL "${TASK_PID}" 2>/dev/null || true
    wait "${TASK_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

echo "[Piper Task MoveIt Gate Fail] 使用 ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "[Piper Task MoveIt Gate Fail] 启动仅任务层并打开 plan-only 门禁，日志：${TASK_LOG}"
setsid ros2 launch slam_nav_piper_manipulation piper_manipulation.launch.py \
  use_sim_time:=false \
  fake_execution:=true \
  require_moveit_plan_before_fake_execution:=true \
  moveit_plan_service:=/piper/missing_plan_kinematic_path \
  moveit_plan_service_timeout_s:=1.0 \
  "$@" >"${TASK_LOG}" 2>&1 &
TASK_PID="$!"

set +e
python3 - <<'PY'
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from slam_nav_piper_interfaces.action import PickObject, PlaceObject


class PiperTaskMoveItGateFailSmoke(Node):
    """验证 MoveIt2 plan-only 服务缺失时，任务 action 必须失败。"""

    def __init__(self):
        super().__init__('piper_task_moveit_gate_fail_client')
        self.pick_client = ActionClient(self, PickObject, '/piper/task/pick_object')
        self.place_client = ActionClient(self, PlaceObject, '/piper/task/place_object')

    def make_pose(self):
        pose = PoseStamped()
        pose.header.frame_id = 'piper_base_link'
        pose.pose.position.x = 0.42
        pose.pose.position.y = 0.0
        pose.pose.position.z = 0.32
        pose.pose.orientation.w = 1.0
        return pose

    def run(self):
        if not self.pick_client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error('等待 /piper/task/pick_object action server 超时。')
            return 2
        if not self.place_client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error('等待 /piper/task/place_object action server 超时。')
            return 2
        if not self.expect_pick_rejected():
            return 2
        if not self.expect_place_rejected():
            return 2
        self.get_logger().info('MoveIt2 plan-only 服务缺失时，任务层按预期安全拒绝。')
        return 0

    def expect_pick_rejected(self):
        goal = PickObject.Goal()
        goal.object_id = 'missing_moveit_plan_target'
        goal.object_class = 'smoke'
        goal.target_pose = self.make_pose()
        goal.allow_redetect = False
        goal.approach_distance_m = 0.10
        goal.gripper_width_m = 0.06
        return self.expect_rejected(self.pick_client, goal, 'pick')

    def expect_place_rejected(self):
        goal = PlaceObject.Goal()
        goal.object_id = 'missing_moveit_plan_target'
        goal.target_pose = self.make_pose()
        goal.open_gripper = True
        goal.retreat_distance_m = 0.10
        return self.expect_rejected(self.place_client, goal, 'place')

    def expect_rejected(self, client, goal, name):
        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if not future.done() or future.result() is None:
            self.get_logger().error(f'{name} goal 发送超时。')
            return False
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'{name} goal 不应在接收阶段被拒绝，应在执行阶段报告门禁失败。')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=10.0)
        if not result_future.done() or result_future.result() is None:
            self.get_logger().error(f'{name} result 等待超时。')
            return False
        result = result_future.result().result
        if result.success:
            self.get_logger().error(f'{name} action 在 MoveIt2 服务缺失时不应成功：{result.message}')
            return False
        required_tokens = ['MoveIt2 plan-only 门禁失败', '等待规划服务超时']
        if not all(token in result.message for token in required_tokens):
            self.get_logger().error(f'{name} action 拒绝原因不符合预期：{result.message}')
            return False
        self.get_logger().info(f'{name} action 按预期拒绝：{result.message}')
        return True


def main():
    rclpy.init()
    node = PiperTaskMoveItGateFailSmoke()
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
  echo "[Piper Task MoveIt Gate Fail] 烟测失败，任务层最后 120 行日志如下：" >&2
  tail -n 120 "${TASK_LOG}" >&2 || true
  exit "${SMOKE_STATUS}"
fi

echo "[Piper Task MoveIt Gate Fail] 缺少 MoveIt2 plan-only 服务时的安全拒绝烟测通过。"
