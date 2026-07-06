#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${WORKSPACE_DIR}"

source "${SCRIPT_DIR}/setup_workspace_env.sh"
set -u

# 独立 ROS domain，只验证任务层显式消费 ranked 候选，不接学习模型或真实机械臂。
export ROS_DOMAIN_ID="${PIPER_RANKED_GATE_ROS_DOMAIN_ID:-91}"

LOG_DIR="${WORKSPACE_DIR}/log"
mkdir -p "${LOG_DIR}"
LOG_FILE="${LOG_DIR}/piper_ranked_candidate_gate_$(date +%Y%m%d_%H%M%S).log"
LAUNCH_PID=""

cleanup() {
  if [[ -n "${LAUNCH_PID}" ]]; then
    echo "[Piper Ranked Gate] 清理本次启动的任务层进程..."
    kill -TERM "-${LAUNCH_PID}" 2>/dev/null || kill -TERM "${LAUNCH_PID}" 2>/dev/null || true
    sleep 2
    kill -KILL "-${LAUNCH_PID}" 2>/dev/null || kill -KILL "${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

echo "[Piper Ranked Gate] 使用 ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "[Piper Ranked Gate] 启动显式消费 ranked 候选的任务层，日志：${LOG_FILE}"
setsid ros2 launch slam_nav_piper_manipulation piper_manipulation.launch.py \
  use_sim_time:=false \
  fake_execution:=true \
  use_ranked_grasp_candidates:=true \
  publish_base_stop:=false \
  "$@" >"${LOG_FILE}" 2>&1 &
LAUNCH_PID="$!"

set +e
python3 - <<'PY'
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from slam_nav_piper_interfaces.action import PickObject
from slam_nav_piper_interfaces.msg import GraspCandidate, GraspCandidateArray


class PiperRankedGateSmoke(Node):
    """验证任务层只有显式打开时才消费 ranked 抓取候选。"""

    def __init__(self):
        super().__init__('piper_ranked_candidate_gate_client')
        self.publisher = self.create_publisher(
            GraspCandidateArray,
            '/piper/learning/grasp_candidates_ranked',
            10,
        )
        self.pick_client = ActionClient(self, PickObject, '/piper/task/pick_object')

    def make_pose(self, x):
        pose = PoseStamped()
        pose.header.frame_id = 'piper_base_link'
        pose.pose.position.x = float(x)
        pose.pose.position.y = 0.12
        pose.pose.position.z = 0.34
        pose.pose.orientation.w = 1.0
        return pose

    def publish_ranked_candidate(self):
        candidate = GraspCandidate()
        candidate.header.frame_id = 'piper_base_link'
        candidate.object_id = 'ranked_high'
        candidate.object_class = 'ranked_test_object'
        candidate.grasp_pose = self.make_pose(0.44)
        candidate.pre_grasp_pose = self.make_pose(0.31)
        candidate.score = 0.93
        candidate.gripper_width_m = 0.055
        candidate.approach_distance_m = 0.13
        candidate.source_frame = 'piper_base_link'
        candidate.tags = ['rank_0', 'learning_backend_heuristic', 'ranked_gate_smoke']

        msg = GraspCandidateArray()
        msg.header.frame_id = 'piper_base_link'
        msg.candidates.append(candidate)
        self.publisher.publish(msg)

    def run(self):
        if not self.pick_client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error('等待 /piper/task/pick_object action server 超时。')
            return 2

        for _ in range(12):
            self.publish_ranked_candidate()
            rclpy.spin_once(self, timeout_sec=0.1)

        goal = PickObject.Goal()
        goal.object_id = 'ranked_high'
        goal.object_class = 'ranked_test_object'
        goal.allow_redetect = False
        goal.approach_distance_m = 0.20
        goal.gripper_width_m = 0.04

        future = self.pick_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if not future.done() or future.result() is None:
            self.get_logger().error('ranked pick goal 发送超时。')
            return 2
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('ranked pick goal 被拒绝。')
            return 2

        result_future = goal_handle.get_result_async()
        deadline = time.time() + 20.0
        while rclpy.ok() and not result_future.done() and time.time() < deadline:
            self.publish_ranked_candidate()
            rclpy.spin_once(self, timeout_sec=0.1)
        if not result_future.done() or result_future.result() is None:
            self.get_logger().error('ranked pick result 等待超时。')
            return 2

        result = result_future.result().result
        if not result.success:
            self.get_logger().error(f'ranked pick 返回失败：{result.message}')
            return 2
        executed = result.executed_pose
        if executed.header.frame_id != 'piper_base_link' or not math.isclose(
            executed.pose.position.x,
            0.31,
            abs_tol=1e-4,
        ):
            self.get_logger().error(
                'pick 未使用 ranked pre_grasp_pose：'
                f'frame={executed.header.frame_id}, x={executed.pose.position.x:.4f}'
            )
            return 2

        self.get_logger().info('ranked 候选门禁通过：pick 使用了 ranked pre_grasp_pose。')
        return 0


def main():
    rclpy.init()
    node = PiperRankedGateSmoke()
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
  echo "[Piper Ranked Gate] 烟测失败，最后 120 行日志如下：" >&2
  tail -n 120 "${LOG_FILE}" >&2 || true
  exit "${SMOKE_STATUS}"
fi

echo "[Piper Ranked Gate] ranked 候选消费门禁烟测通过。"
