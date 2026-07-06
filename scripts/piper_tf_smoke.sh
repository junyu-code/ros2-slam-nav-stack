#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${WORKSPACE_DIR}"

source "${SCRIPT_DIR}/setup_workspace_env.sh"
set -u

# 独立 ROS domain，只验证 Piper TF，不接入 task1 的 map/odom/base_footprint 权威链路。
export ROS_DOMAIN_ID="${PIPER_TF_SMOKE_ROS_DOMAIN_ID:-84}"

LOG_DIR="${WORKSPACE_DIR}/log"
mkdir -p "${LOG_DIR}"
LOG_FILE="${LOG_DIR}/piper_tf_smoke_$(date +%Y%m%d_%H%M%S).log"
LAUNCH_PID=""

cleanup() {
  if [[ -n "${LAUNCH_PID}" ]]; then
    echo "[Piper TF] 清理本次启动的 Piper TF 进程..."
    kill -TERM "-${LAUNCH_PID}" 2>/dev/null || kill -TERM "${LAUNCH_PID}" 2>/dev/null || true
    sleep 2
    kill -KILL "-${LAUNCH_PID}" 2>/dev/null || kill -KILL "${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

echo "[Piper TF] 使用 ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "[Piper TF] 启动 Piper 官方描述 TF，日志：${LOG_FILE}"
setsid ros2 launch slam_nav_piper_description piper_description.launch.py \
  use_sim_time:=false \
  arm_model:=official \
  publish_joint_states:=true \
  "$@" >"${LOG_FILE}" 2>&1 &
LAUNCH_PID="$!"

set +e
python3 - <<'PY'
import sys
import time

import rclpy
import tf2_ros
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from tf2_msgs.msg import TFMessage


class PiperTfSmoke(Node):
    """验证 Piper 运行时 TF 链和 task1 TF 隔离边界。"""

    REQUIRED_TRANSFORMS = [
        ('piper_base_link', 'base_link', 'base_link -> piper_base_link'),
        ('piper_tcp', 'piper_base_link', 'piper_base_link -> piper_tcp'),
        ('piper_arm_camera_optical_frame', 'piper_tcp', 'piper_tcp -> piper_arm_camera_optical_frame'),
        ('piper_arm_camera_optical_frame', 'base_link', 'base_link -> piper_arm_camera_optical_frame'),
    ]

    FORBIDDEN_EDGES = {
        ('map', 'odom'),
        ('odom', 'base_footprint'),
    }

    def __init__(self):
        super().__init__('piper_tf_smoke_client')
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)
        self.seen_edges = set()
        self.seen_frames = set()
        self.create_subscription(TFMessage, '/tf', self.tf_callback, 50)
        static_qos = QoSProfile(depth=50)
        static_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        static_qos.reliability = QoSReliabilityPolicy.RELIABLE
        self.create_subscription(TFMessage, '/tf_static', self.tf_callback, static_qos)

    def tf_callback(self, msg):
        for transform in msg.transforms:
            parent = transform.header.frame_id
            child = transform.child_frame_id
            self.seen_edges.add((parent, child))
            self.seen_frames.add(parent)
            self.seen_frames.add(child)

    def wait_for_transform(self, target_frame, source_frame, label, timeout_s=20.0):
        deadline = time.time() + timeout_s
        last_error = ''
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            try:
                transform = self.buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    Time(),
                    timeout=Duration(seconds=0.2),
                )
                xyz = transform.transform.translation
                self.get_logger().info(
                    f'{label} 可查：translation=({xyz.x:.3f}, {xyz.y:.3f}, {xyz.z:.3f})'
                )
                return True
            except Exception as exc:  # noqa: BLE001 - 冒烟脚本需要把最后一次 TF 错误直接打印出来。
                last_error = str(exc)
        self.get_logger().error(f'等待 {label} 超时：{last_error}')
        return False

    def run(self):
        for target_frame, source_frame, label in self.REQUIRED_TRANSFORMS:
            if not self.wait_for_transform(target_frame, source_frame, label):
                return 2

        # 再多 spin 一小段时间，收集 /tf 与 /tf_static 中可能出现的边界违规 frame。
        end_time = time.time() + 2.0
        while rclpy.ok() and time.time() < end_time:
            rclpy.spin_once(self, timeout_sec=0.2)

        forbidden_seen = sorted(self.FORBIDDEN_EDGES.intersection(self.seen_edges))
        if forbidden_seen:
            self.get_logger().error(f'Piper 独立 TF 不应发布 task1 权威边：{forbidden_seen}')
            return 2

        nav_camera_frames = sorted(frame for frame in self.seen_frames if frame.startswith('nav_camera'))
        if nav_camera_frames:
            self.get_logger().error(f'Piper TF 中不应出现 nav_camera frame：{nav_camera_frames}')
            return 2

        expected_frames = {
            'base_link',
            'piper_mount_link',
            'piper_base_link',
            'piper_tcp',
            'piper_arm_camera_link',
            'piper_arm_camera_optical_frame',
        }
        missing = sorted(expected_frames.difference(self.seen_frames))
        if missing:
            self.get_logger().error(f'Piper TF 缺少关键 frame：{missing}')
            return 2

        self.get_logger().info('Piper 运行时 TF 链和隔离边界烟测通过。')
        return 0


def main():
    rclpy.init()
    node = PiperTfSmoke()
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
  echo "[Piper TF] 烟测失败，最后 120 行日志如下：" >&2
  tail -n 120 "${LOG_FILE}" >&2 || true
  exit "${SMOKE_STATUS}"
fi
