#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${WORKSPACE_DIR}"

source "${SCRIPT_DIR}/setup_workspace_env.sh"
set -u

# 独立 ROS domain，学习层只做排序冒烟，不接入任务层或真实机械臂。
export ROS_DOMAIN_ID="${PIPER_LEARNING_SMOKE_ROS_DOMAIN_ID:-83}"

LOG_DIR="${WORKSPACE_DIR}/log"
mkdir -p "${LOG_DIR}"
LOG_FILE="${LOG_DIR}/piper_learning_smoke_$(date +%Y%m%d_%H%M%S).log"
LAUNCH_PID=""

cleanup() {
  if [[ -n "${LAUNCH_PID}" ]]; then
    echo "[Piper Learning] 清理本次启动的学习层进程..."
    kill -TERM "-${LAUNCH_PID}" 2>/dev/null || kill -TERM "${LAUNCH_PID}" 2>/dev/null || true
    sleep 2
    kill -KILL "-${LAUNCH_PID}" 2>/dev/null || kill -KILL "${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

echo "[Piper Learning] 使用 ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "[Piper Learning] 启动 Piper 学习候选排序旁路，日志：${LOG_FILE}"
setsid ros2 launch slam_nav_piper_learning piper_learning.launch.py \
  use_sim_time:=false \
  enable_learning:=true \
  policy_backend:=heuristic \
  "$@" >"${LOG_FILE}" 2>&1 &
LAUNCH_PID="$!"

set +e
python3 - <<'PY'
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from slam_nav_piper_interfaces.msg import GraspCandidate, GraspCandidateArray


class PiperLearningSmoke(Node):
    """验证学习层只对抓取候选排序，不触碰控制后端。"""

    def __init__(self):
        super().__init__('piper_learning_smoke_client')
        self.ranked = None
        self.publisher = self.create_publisher(GraspCandidateArray, '/piper/grasp_candidates', 10)
        self.create_subscription(
            GraspCandidateArray,
            '/piper/learning/grasp_candidates_ranked',
            self.ranked_cb,
            10,
        )

    def ranked_cb(self, msg):
        self.ranked = msg

    def make_candidate(self, object_id, score):
        candidate = GraspCandidate()
        candidate.header.frame_id = 'piper_base_link'
        candidate.object_id = object_id
        candidate.object_class = 'smoke'
        candidate.grasp_pose = PoseStamped()
        candidate.grasp_pose.header.frame_id = 'piper_base_link'
        candidate.pre_grasp_pose = PoseStamped()
        candidate.pre_grasp_pose.header.frame_id = 'piper_base_link'
        candidate.score = float(score)
        candidate.gripper_width_m = 0.06
        candidate.approach_distance_m = 0.10
        candidate.source_frame = 'piper_base_link'
        candidate.tags = ['smoke_input']
        return candidate

    def publish_candidates(self):
        msg = GraspCandidateArray()
        msg.header.frame_id = 'piper_base_link'
        msg.candidates = [
            self.make_candidate('low', 0.10),
            self.make_candidate('high', 0.90),
            self.make_candidate('mid', 0.50),
        ]
        for _ in range(8):
            self.publisher.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)

    def run(self):
        deadline = time.time() + 20.0
        while rclpy.ok() and time.time() < deadline:
            self.publish_candidates()
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.ranked is None:
                continue
            ids = [candidate.object_id for candidate in self.ranked.candidates]
            scores = [float(candidate.score) for candidate in self.ranked.candidates]
            tags = [tag for candidate in self.ranked.candidates for tag in candidate.tags]
            if ids[:3] != ['high', 'mid', 'low']:
                self.get_logger().error(f'排序结果错误：ids={ids}, scores={scores}')
                return 2
            if 'learning_backend_heuristic' not in tags or 'rank_0' not in tags:
                self.get_logger().error(f'排序标签缺失：tags={tags}')
                return 2
            self.get_logger().info(f'学习层排序烟测通过：ids={ids}, scores={scores}')
            return 0

        self.get_logger().error('等待 /piper/learning/grasp_candidates_ranked 超时。')
        return 2


def main():
    rclpy.init()
    node = PiperLearningSmoke()
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
  echo "[Piper Learning] 烟测失败，最后 120 行日志如下：" >&2
  tail -n 120 "${LOG_FILE}" >&2 || true
  exit "${SMOKE_STATUS}"
fi
