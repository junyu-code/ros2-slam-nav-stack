#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${WORKSPACE_DIR}"

source "${SCRIPT_DIR}/setup_workspace_env.sh"
set -u

# 独立 ROS domain，验证 Piper 运行时图不会污染 task1/Nav2 或 /nav_camera。
export ROS_DOMAIN_ID="${PIPER_NAMESPACE_SMOKE_ROS_DOMAIN_ID:-86}"

LOG_DIR="${WORKSPACE_DIR}/log"
mkdir -p "${LOG_DIR}"
LOG_FILE="${LOG_DIR}/piper_namespace_smoke_$(date +%Y%m%d_%H%M%S).log"
LAUNCH_PID=""

cleanup() {
  if [[ -n "${LAUNCH_PID}" ]]; then
    echo "[Piper Namespace] 清理本次启动的 Piper namespace 进程..."
    kill -TERM "-${LAUNCH_PID}" 2>/dev/null || kill -TERM "${LAUNCH_PID}" 2>/dev/null || true
    sleep 2
    kill -KILL "-${LAUNCH_PID}" 2>/dev/null || kill -KILL "${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

echo "[Piper Namespace] 使用 ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "[Piper Namespace] 启动 Piper fake runtime 图，日志：${LOG_FILE}"
setsid ros2 launch slam_nav_piper_bringup piper_sim.launch.py \
  use_sim_time:=false \
  publish_joint_states:=true \
  "$@" >"${LOG_FILE}" 2>&1 &
LAUNCH_PID="$!"

set +e
python3 - <<'PY'
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node


class PiperNamespaceSmoke(Node):
    """验证 Piper 运行时 topic/action/node 命名空间边界。"""

    REQUIRED_TOPICS = {
        '/piper/arm_camera/color/image_raw',
        '/piper/arm_camera/color/camera_info',
        '/piper/arm_camera/depth/image_raw',
        '/piper/arm_camera/depth/camera_info',
        '/piper/perception/target_pose',
        '/piper/perception/detections_2d',
        '/piper/perception/detections_3d',
        '/piper/grasp_candidates',
        '/piper/control/state',
    }

    REQUIRED_ACTIONS = {
        '/piper/task/pick_object',
        '/piper/task/place_object',
    }

    FORBIDDEN_NODE_NAMES = {
        '/amcl',
        '/bt_navigator',
        '/controller_server',
        '/planner_server',
        '/behavior_server',
        '/waypoint_follower',
        '/map_server',
        '/lifecycle_manager_navigation',
    }

    def __init__(self):
        super().__init__('piper_namespace_smoke_client')

    def topic_names(self):
        return {name for name, _types in self.get_topic_names_and_types()}

    def node_names(self):
        return {name for name, _namespace in self.get_node_names_and_namespaces()}

    def wait_for_topics(self, timeout_s=45.0):
        deadline = time.time() + timeout_s
        missing = sorted(self.REQUIRED_TOPICS)
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            topics = self.topic_names()
            missing = sorted(self.REQUIRED_TOPICS.difference(topics))
            if not missing:
                self.get_logger().info('Piper 运行时必需 topic 已全部出现。')
                return True
        self.get_logger().error(f'等待 Piper 必需 topic 超时，缺少：{missing}')
        self.get_logger().error(f'当前 topic：{sorted(self.topic_names())}')
        return False

    def action_names(self):
        result = subprocess.run(
            ['ros2', 'action', 'list'],
            check=False,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=10.0,
        )
        if result.returncode != 0:
            self.get_logger().error(f'ros2 action list 失败：{result.stdout}')
            return set()
        return {line.strip() for line in result.stdout.splitlines() if line.strip()}

    def wait_for_actions(self, timeout_s=30.0):
        deadline = time.time() + timeout_s
        missing = sorted(self.REQUIRED_ACTIONS)
        actions = set()
        while time.time() < deadline:
            actions = self.action_names()
            missing = sorted(self.REQUIRED_ACTIONS.difference(actions))
            if not missing:
                self.get_logger().info('Piper task action 已全部出现。')
                return True
            time.sleep(1.0)
        self.get_logger().error(f'等待 Piper task action 超时，缺少：{missing}，当前 action：{sorted(actions)}')
        return False

    def validate_forbidden_runtime_graph(self):
        topics = sorted(self.topic_names())
        actions = sorted(self.action_names())
        nodes = sorted(self.node_names())
        failures = []

        nav_camera_topics = [topic for topic in topics if topic.startswith('/nav_camera') or '/nav_camera/' in topic]
        if nav_camera_topics:
            failures.append(f'不应出现 /nav_camera topic：{nav_camera_topics}')

        costmap_topics = [topic for topic in topics if 'costmap' in topic.lower()]
        if costmap_topics:
            failures.append(f'Piper 独立运行图不应出现 costmap topic：{costmap_topics}')

        nav_actions = [action for action in actions if action.startswith('/navigate') or action.startswith('/follow_')]
        if nav_actions:
            failures.append(f'Piper 独立运行图不应出现 Nav2 action：{nav_actions}')

        forbidden_nodes = sorted(self.FORBIDDEN_NODE_NAMES.intersection(nodes))
        if forbidden_nodes:
            failures.append(f'Piper 独立运行图不应启动 Nav2/AMCL 节点：{forbidden_nodes}')

        piper_camera_topics = [topic for topic in topics if topic.startswith('/piper/arm_camera/')]
        bad_piper_camera_topics = [
            topic
            for topic in piper_camera_topics
            if not (
                topic.startswith('/piper/arm_camera/color/')
                or topic.startswith('/piper/arm_camera/depth/')
            )
        ]
        if bad_piper_camera_topics:
            failures.append(f'Piper 相机 topic 路径不符合 color/depth 约定：{bad_piper_camera_topics}')

        non_piper_actions = [action for action in actions if action.startswith('/piper') is False]
        if non_piper_actions:
            failures.append(f'Piper 独立 action 图不应出现非 /piper action：{non_piper_actions}')

        if failures:
            for failure in failures:
                self.get_logger().error(failure)
            self.get_logger().error(f'当前 topic：{topics}')
            self.get_logger().error(f'当前 action：{actions}')
            self.get_logger().error(f'当前 node：{nodes}')
            return False

        self.get_logger().info('Piper runtime namespace 边界检查通过。')
        return True

    def run(self):
        if not self.wait_for_topics():
            return 2
        if not self.wait_for_actions():
            return 2
        if not self.validate_forbidden_runtime_graph():
            return 2
        return 0


def main():
    rclpy.init()
    node = PiperNamespaceSmoke()
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
  echo "[Piper Namespace] 烟测失败，最后 160 行日志如下：" >&2
  tail -n 160 "${LOG_FILE}" >&2 || true
  exit "${SMOKE_STATUS}"
fi
