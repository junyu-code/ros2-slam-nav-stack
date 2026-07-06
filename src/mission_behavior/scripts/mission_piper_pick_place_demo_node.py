#!/usr/bin/env python3

import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from slam_nav_piper_interfaces.action import PickObject, PlaceObject


class MissionPiperPickPlaceDemoNode(Node):
    """任务层 Piper demo：只调用 /piper/task/* action，不直接依赖 MoveIt2 或 SDK。"""

    def __init__(self):
        super().__init__('mission_piper_pick_place_demo_node')
        self.declare_parameter('auto_start', False)
        self.declare_parameter('pick_action', '/piper/task/pick_object')
        self.declare_parameter('place_action', '/piper/task/place_object')
        self.declare_parameter('target_frame', 'piper_base_link')
        self.declare_parameter('object_id', 'mission_demo_target')
        self.declare_parameter('object_class', 'demo')
        self.declare_parameter('target_x', 0.30)
        self.declare_parameter('target_y', 0.0)
        self.declare_parameter('target_z', 0.22)
        self.declare_parameter('target_yaw', 0.0)
        self.declare_parameter('approach_distance_m', 0.10)
        self.declare_parameter('gripper_width_m', 0.06)
        self.declare_parameter('retreat_distance_m', 0.10)
        self.declare_parameter('wait_action_timeout_s', 20.0)
        self.declare_parameter('result_timeout_s', 30.0)
        self.declare_parameter('shutdown_after_done', True)

        self.auto_start = bool(self.get_parameter('auto_start').value)
        self.wait_action_timeout_s = float(self.get_parameter('wait_action_timeout_s').value)
        self.result_timeout_s = float(self.get_parameter('result_timeout_s').value)
        self.shutdown_after_done = bool(self.get_parameter('shutdown_after_done').value)
        self.finished = False
        self.exit_code = 0

        self.pick_client = ActionClient(
            self,
            PickObject,
            str(self.get_parameter('pick_action').value),
        )
        self.place_client = ActionClient(
            self,
            PlaceObject,
            str(self.get_parameter('place_action').value),
        )

        self.get_logger().info(
            'Mission Piper demo 已就绪；只会调用 /piper/task/* action，'
            f'auto_start={self.auto_start}。'
        )
        if self.auto_start:
            self.create_timer(0.1, self._auto_start_once)

    def _auto_start_once(self):
        if self.finished:
            return
        self.finished = True
        self.exit_code = 0 if self.run_once() else 2
        if self.shutdown_after_done:
            rclpy.shutdown()

    def run_once(self):
        self.get_logger().info('[Mission Piper] 等待 Piper pick/place action server。')
        if not self.pick_client.wait_for_server(timeout_sec=self.wait_action_timeout_s):
            self.get_logger().error('[Mission Piper] /piper/task/pick_object action server 不可用。')
            return False
        if not self.place_client.wait_for_server(timeout_sec=self.wait_action_timeout_s):
            self.get_logger().error('[Mission Piper] /piper/task/place_object action server 不可用。')
            return False

        target_pose = self.make_target_pose()
        if not self.send_pick(target_pose):
            return False
        if not self.send_place(target_pose):
            return False

        self.get_logger().info('[Mission Piper] pick/place demo 完成。')
        return True

    def send_pick(self, target_pose):
        goal = PickObject.Goal()
        goal.object_id = str(self.get_parameter('object_id').value)
        goal.object_class = str(self.get_parameter('object_class').value)
        goal.target_pose = target_pose
        goal.allow_redetect = False
        goal.approach_distance_m = float(self.get_parameter('approach_distance_m').value)
        goal.gripper_width_m = float(self.get_parameter('gripper_width_m').value)
        return self.send_goal(self.pick_client, goal, 'pick')

    def send_place(self, target_pose):
        goal = PlaceObject.Goal()
        goal.object_id = str(self.get_parameter('object_id').value)
        goal.target_pose = target_pose
        goal.open_gripper = True
        goal.retreat_distance_m = float(self.get_parameter('retreat_distance_m').value)
        return self.send_goal(self.place_client, goal, 'place')

    def send_goal(self, client, goal, label):
        self.get_logger().info(f'[Mission Piper] 发送 {label} goal。')
        future = client.send_goal_async(goal, feedback_callback=lambda msg: self.feedback_cb(label, msg))
        goal_handle = self.wait_for_future(future, 10.0)
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f'[Mission Piper] {label} goal 被拒绝或发送超时。')
            return False

        result_future = goal_handle.get_result_async()
        wrapped = self.wait_for_future(result_future, self.result_timeout_s)
        if wrapped is None:
            self.get_logger().error(f'[Mission Piper] {label} result 等待超时。')
            return False
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED or not wrapped.result.success:
            self.get_logger().error(
                f'[Mission Piper] {label} 失败: status={wrapped.status}, message={wrapped.result.message}'
            )
            return False

        self.get_logger().info(f'[Mission Piper] {label} 成功: {wrapped.result.message}')
        return True

    def feedback_cb(self, label, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'[Mission Piper] {label} feedback: {feedback.stage} ({feedback.progress:.0%})',
            throttle_duration_sec=1.0,
        )

    def wait_for_future(self, future, timeout_s):
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        while rclpy.ok() and time.monotonic() <= deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if future.done():
                return future.result()
        return None

    def make_target_pose(self):
        pose = PoseStamped()
        pose.header.frame_id = str(self.get_parameter('target_frame').value)
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(self.get_parameter('target_x').value)
        pose.pose.position.y = float(self.get_parameter('target_y').value)
        pose.pose.position.z = float(self.get_parameter('target_z').value)

        yaw = float(self.get_parameter('target_yaw').value)
        pose.pose.orientation.z = math.sin(yaw * 0.5)
        pose.pose.orientation.w = math.cos(yaw * 0.5)
        return pose


def main():
    rclpy.init()
    node = MissionPiperPickPlaceDemoNode()
    try:
        if node.auto_start:
            rclpy.spin(node)
        else:
            node.get_logger().info('设置 auto_start:=true 后会顺序调用 Piper pick/place。')
            rclpy.spin(node)
    finally:
        exit_code = node.exit_code
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
