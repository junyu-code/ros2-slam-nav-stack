#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${WORKSPACE_DIR}"

source "${SCRIPT_DIR}/setup_workspace_env.sh"
set -u

# 独立 ROS domain，验证移动操作组合入口，不接入 task1 导航主链路。
export ROS_DOMAIN_ID="${PIPER_MOBILE_SEQUENCE_SMOKE_ROS_DOMAIN_ID:-89}"

LOG_DIR="${WORKSPACE_DIR}/log"
mkdir -p "${LOG_DIR}"
LOG_FILE="${LOG_DIR}/piper_mobile_sequence_smoke_$(date +%Y%m%d_%H%M%S).log"
LAUNCH_PID=""

cleanup() {
  if [[ -n "${LAUNCH_PID}" ]]; then
    echo "[Piper Mobile Sequence] 清理本次启动的移动操作组合进程..."
    kill -TERM "-${LAUNCH_PID}" 2>/dev/null || kill -TERM "${LAUNCH_PID}" 2>/dev/null || true
    sleep 2
    kill -KILL "-${LAUNCH_PID}" 2>/dev/null || kill -KILL "${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

echo "[Piper Mobile Sequence] 使用 ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "[Piper Mobile Sequence] 启动 Piper 移动操作组合入口，日志：${LOG_FILE}"
setsid ros2 launch slam_nav_piper_bringup piper_mobile_manipulation.launch.py \
  use_sim_time:=false \
  start_description:=true \
  publish_joint_states:=true \
  fake_camera:=true \
  fake_execution:=true \
  publish_base_stop:=true \
  "$@" >"${LOG_FILE}" 2>&1 &
LAUNCH_PID="$!"

set +e
python3 - <<'PY'
import math
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import Image
from slam_nav_piper_interfaces.action import PickObject, PlaceObject
from slam_nav_piper_interfaces.msg import GraspCandidateArray
from std_msgs.msg import String


class PiperMobileSequenceSmoke(Node):
    """验证 Piper 移动操作组合入口的假相机、停车、pick/place action 顺序。"""

    def __init__(self):
        super().__init__('piper_mobile_sequence_smoke_client')
        self.color_seen = False
        self.depth_seen = False
        self.target_pose = None
        self.grasp_candidates = None
        self.control_state = {}
        self.zero_cmd_count = 0
        self.pick_client = ActionClient(self, PickObject, '/piper/task/pick_object')
        self.place_client = ActionClient(self, PlaceObject, '/piper/task/place_object')
        self.create_subscription(Image, '/piper/arm_camera/color/image_raw', self.color_cb, 10)
        self.create_subscription(Image, '/piper/arm_camera/depth/image_raw', self.depth_cb, 10)
        self.create_subscription(PoseStamped, '/piper/perception/target_pose', self.target_cb, 10)
        self.create_subscription(GraspCandidateArray, '/piper/grasp_candidates', self.grasp_cb, 10)
        self.create_subscription(String, '/piper/control/state', self.control_cb, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)

    def color_cb(self, _msg):
        self.color_seen = True

    def depth_cb(self, _msg):
        self.depth_seen = True

    def target_cb(self, msg):
        self.target_pose = msg

    def grasp_cb(self, msg):
        if msg.candidates:
            self.grasp_candidates = msg

    def control_cb(self, msg):
        parsed = {}
        for item in msg.data.split(';'):
            if '=' not in item:
                continue
            key, value = item.split('=', 1)
            parsed[key.strip()] = value.strip()
        self.control_state = parsed

    def cmd_vel_cb(self, msg):
        values = (
            msg.linear.x,
            msg.linear.y,
            msg.linear.z,
            msg.angular.x,
            msg.angular.y,
            msg.angular.z,
        )
        if all(math.isclose(value, 0.0, abs_tol=1e-9) for value in values):
            self.zero_cmd_count += 1

    def wait_until(self, label, predicate, timeout_s=45.0):
        deadline = time.time() + timeout_s
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            if predicate():
                self.get_logger().info(f'{label} 已就绪。')
                return True
        self.get_logger().error(f'等待 {label} 超时。')
        return False

    def ensure_no_nav2_actions(self):
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
            return False
        actions = {line.strip() for line in result.stdout.splitlines() if line.strip()}
        nav_actions = sorted(
            action
            for action in actions
            if action.startswith('/navigate') or action.startswith('/follow_')
        )
        if nav_actions:
            self.get_logger().error(f'移动操作组合入口不应启动 Nav2 action：{nav_actions}')
            return False
        return True

    def run(self):
        checks = [
            ('Piper 彩色图像', lambda: self.color_seen),
            ('Piper 深度图像', lambda: self.depth_seen),
            ('Piper 目标位姿', lambda: self.target_pose is not None),
            ('Piper 抓取候选', lambda: self.grasp_candidates is not None),
            ('Piper 控制状态', lambda: bool(self.control_state)),
        ]
        for label, predicate in checks:
            if not self.wait_until(label, predicate):
                return 2

        if not self.ensure_no_nav2_actions():
            return 2
        if not self.pick_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('等待 /piper/task/pick_object action server 超时。')
            return 2
        if not self.place_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('等待 /piper/task/place_object action server 超时。')
            return 2

        if not self.send_pick_goal():
            return 2
        if not self.wait_until('Piper 停车零速度命令', lambda: self.zero_cmd_count >= 1, timeout_s=5.0):
            return 2
        if not self.send_place_goal():
            return 2

        if self.control_state.get('owner') != 'moveit':
            self.get_logger().error(f'任务执行后控制 owner 应切到 moveit，实际：{self.control_state}')
            return 2

        self.get_logger().info(
            f'移动操作组合烟测通过：zero_cmd_count={self.zero_cmd_count}, control={self.control_state}'
        )
        return 0

    def send_pick_goal(self):
        goal = PickObject.Goal()
        goal.object_id = 'mobile_sequence_target'
        goal.object_class = 'smoke'
        goal.target_pose = self.target_pose
        goal.allow_redetect = False
        goal.approach_distance_m = 0.10
        goal.gripper_width_m = 0.06
        return self.send_goal(self.pick_client, goal, 'pick')

    def send_place_goal(self):
        goal = PlaceObject.Goal()
        goal.object_id = 'mobile_sequence_target'
        goal.target_pose = self.target_pose
        goal.open_gripper = True
        goal.retreat_distance_m = 0.10
        return self.send_goal(self.place_client, goal, 'place')

    def send_goal(self, client, goal, name):
        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if not future.done() or future.result() is None:
            self.get_logger().error(f'{name} goal 发送超时。')
            return False
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'{name} goal 被拒绝。')
            return False
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=20.0)
        if not result_future.done() or result_future.result() is None:
            self.get_logger().error(f'{name} result 等待超时。')
            return False
        result = result_future.result().result
        if not result.success:
            self.get_logger().error(f'{name} action 返回失败：{result.message}')
            return False
        self.get_logger().info(f'{name} action 成功：{result.message}')
        return True


def main():
    rclpy.init()
    node = PiperMobileSequenceSmoke()
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
  echo "[Piper Mobile Sequence] 烟测失败，最后 160 行日志如下：" >&2
  tail -n 160 "${LOG_FILE}" >&2 || true
  exit "${SMOKE_STATUS}"
fi

echo "[Piper Mobile Sequence] Piper 移动操作组合烟测通过。"
