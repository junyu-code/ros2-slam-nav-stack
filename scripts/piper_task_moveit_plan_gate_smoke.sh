#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${WORKSPACE_DIR}"

source "${SCRIPT_DIR}/setup_workspace_env.sh"

for piper_moveit_local_setup in \
  "${WORKSPACE_DIR}/external/ros_humble_debs/overlay/opt/ros/humble/share/moveit_planners_ompl/local_setup.bash" \
  "${WORKSPACE_DIR}/external/ros_humble_debs/overlay/opt/ros/humble/share/moveit_simple_controller_manager/local_setup.bash"
do
  if [[ -f "${piper_moveit_local_setup}" ]]; then
    # 任务层 plan-only 门禁同样使用 Piper 专用本地 MoveIt2 插件，不要求 sudo 安装。
    source "${piper_moveit_local_setup}"
  fi
done

set -u

# 独立 ROS domain，验证任务层能调用 MoveIt2 plan-only，但不执行轨迹、不接真实 SDK。
export ROS_DOMAIN_ID="${PIPER_TASK_MOVEIT_GATE_ROS_DOMAIN_ID:-92}"

LOG_DIR="${WORKSPACE_DIR}/log"
mkdir -p "${LOG_DIR}"
MOVEIT_LOG="${LOG_DIR}/piper_task_moveit_gate_moveit_$(date +%Y%m%d_%H%M%S).log"
TASK_LOG="${LOG_DIR}/piper_task_moveit_gate_task_$(date +%Y%m%d_%H%M%S).log"
MOVEIT_PID=""
TASK_PID=""

cleanup() {
  for pid in "${TASK_PID}" "${MOVEIT_PID}"; do
    if [[ -n "${pid}" ]]; then
      kill -TERM "-${pid}" 2>/dev/null || kill -TERM "${pid}" 2>/dev/null || true
    fi
  done
  sleep 2
  for pid in "${TASK_PID}" "${MOVEIT_PID}"; do
    if [[ -n "${pid}" ]]; then
      kill -KILL "-${pid}" 2>/dev/null || kill -KILL "${pid}" 2>/dev/null || true
      wait "${pid}" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT INT TERM

service_ready() {
  ros2 service list 2>/dev/null | grep -qx '/piper/plan_kinematic_path'
}

echo "[Piper Task MoveIt Gate] 使用 ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "[Piper Task MoveIt Gate] 启动 MoveIt2 plan-only，日志：${MOVEIT_LOG}"
setsid ros2 launch slam_nav_piper_moveit_config piper_project_moveit_plan.launch.py \
  use_sim_time:=false \
  >"${MOVEIT_LOG}" 2>&1 &
MOVEIT_PID="$!"

deadline=$((SECONDS + 90))
while ! service_ready; do
  if ! kill -0 "${MOVEIT_PID}" 2>/dev/null; then
    echo "[Piper Task MoveIt Gate] MoveIt2 进程提前退出，最后 100 行日志如下：" >&2
    tail -n 100 "${MOVEIT_LOG}" >&2 || true
    exit 2
  fi
  if (( SECONDS >= deadline )); then
    echo "[Piper Task MoveIt Gate] 等待 /piper/plan_kinematic_path 超时，最后 100 行日志如下：" >&2
    tail -n 100 "${MOVEIT_LOG}" >&2 || true
    exit 2
  fi
  sleep 2
done

echo "[Piper Task MoveIt Gate] 启动 Piper fake 任务链并打开 plan-only 门禁，日志：${TASK_LOG}"
setsid ros2 launch slam_nav_piper_bringup piper_mobile_manipulation.launch.py \
  use_sim_time:=false \
  start_description:=false \
  fake_camera:=true \
  fake_execution:=true \
  publish_base_stop:=false \
  require_moveit_plan_before_fake_execution:=true \
  "$@" >"${TASK_LOG}" 2>&1 &
TASK_PID="$!"

set +e
python3 - <<'PY'
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from slam_nav_piper_interfaces.action import PickObject, PlaceObject


class PiperTaskMoveItGateSmoke(Node):
    """验证 /piper/task/* action 会显式通过 MoveIt2 plan-only 门禁。"""

    def __init__(self):
        super().__init__('piper_task_moveit_gate_smoke_client')
        self.target_pose = None
        self.pick_client = ActionClient(self, PickObject, '/piper/task/pick_object')
        self.place_client = ActionClient(self, PlaceObject, '/piper/task/place_object')
        self.create_subscription(
            PoseStamped,
            '/piper/perception/target_pose',
            self.target_cb,
            10,
        )

    def target_cb(self, msg):
        self.target_pose = msg

    def wait_until(self, label, predicate, timeout_s=60.0):
        deadline = time.time() + timeout_s
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            if predicate():
                self.get_logger().info(f'{label} 已就绪。')
                return True
        self.get_logger().error(f'等待 {label} 超时。')
        return False

    def run(self):
        if not self.wait_until('Piper 目标位姿', lambda: self.target_pose is not None):
            return 2
        if not self.pick_client.wait_for_server(timeout_sec=20.0):
            self.get_logger().error('等待 /piper/task/pick_object action server 超时。')
            return 2
        if not self.place_client.wait_for_server(timeout_sec=20.0):
            self.get_logger().error('等待 /piper/task/place_object action server 超时。')
            return 2
        if not self.send_pick_goal():
            return 2
        if not self.send_place_goal():
            return 2
        self.get_logger().info('任务层 MoveIt2 plan-only 门禁烟测通过。')
        return 0

    def send_pick_goal(self):
        goal = PickObject.Goal()
        goal.object_id = 'moveit_gate_target'
        goal.object_class = 'smoke'
        goal.target_pose = self.target_pose
        goal.allow_redetect = False
        goal.approach_distance_m = 0.10
        goal.gripper_width_m = 0.06
        return self.send_goal(self.pick_client, goal, 'pick')

    def send_place_goal(self):
        goal = PlaceObject.Goal()
        goal.object_id = 'moveit_gate_target'
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
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=40.0)
        if not result_future.done() or result_future.result() is None:
            self.get_logger().error(f'{name} result 等待超时。')
            return False
        result = result_future.result().result
        if not result.success:
            self.get_logger().error(f'{name} action 返回失败：{result.message}')
            return False
        if 'MoveIt2 plan-only 门禁已通过' not in result.message:
            self.get_logger().error(f'{name} action 未报告 MoveIt2 plan-only 门禁：{result.message}')
            return False
        self.get_logger().info(f'{name} action 已通过 MoveIt2 plan-only 门禁：{result.message}')
        return True


def main():
    rclpy.init()
    node = PiperTaskMoveItGateSmoke()
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
  echo "[Piper Task MoveIt Gate] 烟测失败，MoveIt2 最后 80 行日志如下：" >&2
  tail -n 80 "${MOVEIT_LOG}" >&2 || true
  echo "[Piper Task MoveIt Gate] 任务链最后 120 行日志如下：" >&2
  tail -n 120 "${TASK_LOG}" >&2 || true
  exit "${SMOKE_STATUS}"
fi

echo "[Piper Task MoveIt Gate] 任务层 MoveIt2 plan-only 门禁烟测通过。"
