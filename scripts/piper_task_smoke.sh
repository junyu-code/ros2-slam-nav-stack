#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${WORKSPACE_DIR}"

source "${SCRIPT_DIR}/setup_workspace_env.sh"
set -u

# 独立 ROS domain，避免和正在运行的 task1 导航/仿真互相发现。
export ROS_DOMAIN_ID="${PIPER_TASK_SMOKE_ROS_DOMAIN_ID:-79}"

LOG_DIR="${WORKSPACE_DIR}/log"
mkdir -p "${LOG_DIR}"
LOG_FILE="${LOG_DIR}/piper_task_smoke_$(date +%Y%m%d_%H%M%S).log"
LAUNCH_PID=""

cleanup() {
  if [[ -n "${LAUNCH_PID}" ]]; then
    echo "[Piper Task] 清理本次启动的 Piper fake task 进程..."
    kill -TERM "-${LAUNCH_PID}" 2>/dev/null || kill -TERM "${LAUNCH_PID}" 2>/dev/null || true
    sleep 2
    kill -KILL "-${LAUNCH_PID}" 2>/dev/null || kill -KILL "${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

echo "[Piper Task] 使用 ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "[Piper Task] 启动 Piper fake 感知/任务链路，日志：${LOG_FILE}"
setsid ros2 launch slam_nav_piper_bringup piper_sim.launch.py \
  use_sim_time:=false \
  publish_joint_states:=true \
  "$@" >"${LOG_FILE}" 2>&1 &
LAUNCH_PID="$!"

set +e
python3 - <<'PY'
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray
from vision_msgs.msg import Detection2DArray, Detection3DArray

from slam_nav_piper_interfaces.action import PickObject, PlaceObject
from slam_nav_piper_interfaces.msg import GraspCandidateArray


class PiperTaskSmoke(Node):
    """验证 Piper fake 感知、抓取候选和 pick/place action 链路。"""

    def __init__(self):
        super().__init__('piper_task_smoke_client')
        self.color_seen = False
        self.depth_seen = False
        self.debug_image_seen = False
        self.detections_2d = None
        self.detections_3d = None
        self.target_pose = None
        self.grasp_candidates = None
        self.grasp_markers = None
        self.pick_client = ActionClient(self, PickObject, '/piper/task/pick_object')
        self.place_client = ActionClient(self, PlaceObject, '/piper/task/place_object')
        self.create_subscription(Image, '/piper/arm_camera/color/image_raw', self.color_cb, 10)
        self.create_subscription(Image, '/piper/arm_camera/depth/image_raw', self.depth_cb, 10)
        self.create_subscription(Image, '/piper/perception/debug_image', self.debug_image_cb, 10)
        self.create_subscription(Detection2DArray, '/piper/perception/detections_2d', self.detections_2d_cb, 10)
        self.create_subscription(Detection3DArray, '/piper/perception/detections_3d', self.detections_3d_cb, 10)
        self.create_subscription(PoseStamped, '/piper/perception/target_pose', self.target_cb, 10)
        self.create_subscription(GraspCandidateArray, '/piper/grasp_candidates', self.grasp_cb, 10)
        self.create_subscription(MarkerArray, '/piper/visualization/grasp_candidates', self.marker_cb, 10)

    def color_cb(self, _msg):
        self.color_seen = True

    def depth_cb(self, _msg):
        self.depth_seen = True

    def debug_image_cb(self, msg):
        if msg.encoding not in ('rgb8', 'bgr8'):
            return
        # 假检测调试图必须包含画框颜色，避免只发布原始彩色图也误判通过。
        expected = bytes([255, 80, 0]) if msg.encoding == 'rgb8' else bytes([0, 80, 255])
        self.debug_image_seen = expected in bytes(msg.data)

    def detections_2d_cb(self, msg):
        if msg.detections:
            self.detections_2d = msg

    def detections_3d_cb(self, msg):
        if msg.detections:
            self.detections_3d = msg

    def target_cb(self, msg):
        self.target_pose = msg

    def grasp_cb(self, msg):
        if msg.candidates:
            self.grasp_candidates = msg

    def marker_cb(self, msg):
        if msg.markers:
            self.grasp_markers = msg

    def wait_until(self, label, predicate, timeout_s=45.0):
        deadline = time.time() + timeout_s
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            if predicate():
                self.get_logger().info(f'{label} 已就绪。')
                return True
        self.get_logger().error(f'等待 {label} 超时。')
        return False

    def run(self):
        checks = [
            ('Piper 彩色图像 /piper/arm_camera/color/image_raw', lambda: self.color_seen),
            ('Piper 深度图像 /piper/arm_camera/depth/image_raw', lambda: self.depth_seen),
            ('Piper 2D 检测 /piper/perception/detections_2d', lambda: self.detections_2d is not None),
            ('Piper 3D 检测 /piper/perception/detections_3d', lambda: self.detections_3d is not None),
            ('Piper 调试图像 /piper/perception/debug_image', lambda: self.debug_image_seen),
            ('Piper 目标位姿 /piper/perception/target_pose', lambda: self.target_pose is not None),
            ('Piper 抓取候选 /piper/grasp_candidates', self.grasp_candidate_metadata_ready),
            ('Piper 抓取候选可视化 /piper/visualization/grasp_candidates', self.grasp_markers_ready),
        ]
        for label, predicate in checks:
            if not self.wait_until(label, predicate):
                return 2
        if not self.check_grasp_candidate_metadata():
            return 2

        if not self.pick_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('等待 /piper/task/pick_object action server 超时。')
            return 2
        if not self.place_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('等待 /piper/task/place_object action server 超时。')
            return 2

        if not self.send_pick_goal():
            return 2
        if not self.send_place_goal():
            return 2
        self.get_logger().info('Piper fake pick/place action 烟测通过。')
        return 0

    def check_grasp_candidate_metadata(self):
        if self.grasp_candidates is None or not self.grasp_candidates.candidates:
            self.get_logger().error('尚未收到抓取候选。')
            return False
        candidate = self.grasp_candidates.candidates[0]
        tags = set(candidate.tags)
        if candidate.object_class != 'center_depth_target':
            self.get_logger().error(f'抓取候选未继承 3D detection class：{candidate.object_class}')
            return False
        if candidate.object_id != 'center_depth_target':
            self.get_logger().error(f'抓取候选未继承 3D detection id：{candidate.object_id}')
            return False
        if 'detection_3d' not in tags or 'class:center_depth_target' not in tags:
            self.get_logger().error(f'抓取候选缺少 detection_3d 元数据 tag：{candidate.tags}')
            return False
        if candidate.score < 0.49:
            self.get_logger().error(f'抓取候选分数未继承 detection score：{candidate.score}')
            return False
        self.get_logger().info(
            f'抓取候选已继承 3D detection 元数据：id={candidate.object_id}, '
            f'class={candidate.object_class}, score={candidate.score:.2f}'
        )
        return True

    def grasp_markers_ready(self):
        if self.grasp_markers is None:
            return False
        namespaces = {marker.ns for marker in self.grasp_markers.markers}
        return {'grasp_pose', 'pre_grasp_pose', 'approach_vector', 'grasp_label'}.issubset(namespaces)

    def grasp_candidate_metadata_ready(self):
        if self.grasp_candidates is None or not self.grasp_candidates.candidates:
            return False
        candidate = self.grasp_candidates.candidates[0]
        tags = set(candidate.tags)
        return (
            candidate.object_id == 'center_depth_target'
            and candidate.object_class == 'center_depth_target'
            and 'detection_3d' in tags
            and 'class:center_depth_target' in tags
            and candidate.score >= 0.49
        )

    def send_pick_goal(self):
        goal = PickObject.Goal()
        goal.object_id = 'smoke_target'
        goal.object_class = 'unknown'
        goal.target_pose = self.target_pose
        goal.allow_redetect = False
        goal.approach_distance_m = 0.10
        goal.gripper_width_m = 0.06
        return self.send_goal(self.pick_client, goal, 'pick')

    def send_place_goal(self):
        goal = PlaceObject.Goal()
        goal.object_id = 'smoke_target'
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
    node = PiperTaskSmoke()
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
  echo "[Piper Task] 烟测失败，最后 120 行日志如下：" >&2
  tail -n 120 "${LOG_FILE}" >&2 || true
  exit "${SMOKE_STATUS}"
fi
