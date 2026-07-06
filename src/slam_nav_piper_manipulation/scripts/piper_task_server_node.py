#!/usr/bin/env python3

import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.action import ActionServer
from rclpy.node import Node
from slam_nav_piper_interfaces.action import PickObject, PlaceObject
from slam_nav_piper_interfaces.msg import GraspCandidate, GraspCandidateArray
from std_msgs.msg import String


class PiperTaskServerNode(Node):
    """Piper 抓取/放置任务层，占位执行 MoveIt2 或 SDK 后端。"""

    def __init__(self):
        super().__init__('piper_task_server_node')
        self.declare_parameter('target_pose_topic', '/piper/perception/target_pose')
        self.declare_parameter('grasp_candidates_topic', '/piper/grasp_candidates')
        self.declare_parameter('control_state_topic', '/piper/control/state')
        self.declare_parameter('owner_request_topic', '/piper/control/owner_request')
        self.declare_parameter('base_cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('publish_base_stop', False)
        self.declare_parameter('fake_execution', True)
        self.declare_parameter('default_approach_distance_m', 0.10)
        self.declare_parameter('default_gripper_width_m', 0.06)
        self.declare_parameter('use_ranked_grasp_candidates', False)
        self.declare_parameter('ranked_grasp_candidates_topic', '/piper/learning/grasp_candidates_ranked')
        self.declare_parameter('real_backend_connected', False)
        self.declare_parameter('control_ready_timeout_s', 2.0)

        self.latest_target_pose = None
        self.control_state = ''
        self.fake_execution = bool(self.get_parameter('fake_execution').value)
        self.real_backend_connected = bool(self.get_parameter('real_backend_connected').value)
        self.control_ready_timeout_s = float(self.get_parameter('control_ready_timeout_s').value)
        self.publish_base_stop = bool(self.get_parameter('publish_base_stop').value)
        self.default_approach_distance_m = float(
            self.get_parameter('default_approach_distance_m').value
        )
        self.default_gripper_width_m = float(self.get_parameter('default_gripper_width_m').value)
        self.use_ranked_grasp_candidates = bool(
            self.get_parameter('use_ranked_grasp_candidates').value
        )

        self.create_subscription(
            PoseStamped,
            str(self.get_parameter('target_pose_topic').value),
            self.target_pose_callback,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('control_state_topic').value),
            self.control_state_callback,
            10,
        )
        self.grasp_pub = self.create_publisher(
            GraspCandidateArray,
            str(self.get_parameter('grasp_candidates_topic').value),
            10,
        )
        self.owner_pub = self.create_publisher(
            String,
            str(self.get_parameter('owner_request_topic').value),
            10,
        )
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            str(self.get_parameter('base_cmd_vel_topic').value),
            10,
        )

        self.pick_server = ActionServer(
            self,
            PickObject,
            '/piper/task/pick_object',
            self.execute_pick,
        )
        self.place_server = ActionServer(
            self,
            PlaceObject,
            '/piper/task/place_object',
            self.execute_place,
        )
        self.candidate_timer = self.create_timer(0.5, self.publish_grasp_candidate)

        self.get_logger().info(
            f'Piper 任务 action server 已启动，fake_execution={self.fake_execution}。'
        )
        if not self.use_ranked_grasp_candidates:
            self.get_logger().info('Piper 任务层未接入学习排序结果，继续使用原始 grasp candidates。')
        if not self.fake_execution and not self.real_backend_connected:
            self.get_logger().warn('Piper fake_execution=false，但真实后端未声明接入；任务会安全拒绝执行。')

    def target_pose_callback(self, msg):
        self.latest_target_pose = msg

    def control_state_callback(self, msg):
        self.control_state = msg.data

    def publish_grasp_candidate(self):
        if self.latest_target_pose is None:
            return
        array = GraspCandidateArray()
        array.header = self.latest_target_pose.header
        candidate = GraspCandidate()
        candidate.header = self.latest_target_pose.header
        candidate.object_id = 'latest_target'
        candidate.object_class = 'unknown'
        candidate.grasp_pose = self.latest_target_pose
        candidate.pre_grasp_pose = self.make_pre_grasp_pose(
            self.latest_target_pose,
            self.default_approach_distance_m,
        )
        candidate.score = 0.50
        candidate.gripper_width_m = self.default_gripper_width_m
        candidate.approach_distance_m = self.default_approach_distance_m
        candidate.source_frame = self.latest_target_pose.header.frame_id
        candidate.tags = ['depth_center_placeholder']
        array.candidates.append(candidate)
        self.grasp_pub.publish(array)

    def execute_pick(self, goal_handle):
        goal = goal_handle.request
        feedback = PickObject.Feedback()
        result = PickObject.Result()

        self.publish_feedback(goal_handle, feedback, '停止底盘', 0.05)
        self.stop_base_for_arm_motion()

        self.publish_feedback(goal_handle, feedback, '申请 MoveIt2 控制 owner', 0.15)
        self.request_owner('moveit')
        if not self.check_execution_allowed(goal_handle, result):
            return result

        target_pose = goal.target_pose
        if not target_pose.header.frame_id:
            if self.latest_target_pose is None:
                goal_handle.abort()
                result.success = False
                result.message = '没有目标位姿，无法执行 pick。'
                return result
            target_pose = self.latest_target_pose

        self.publish_feedback(goal_handle, feedback, '生成抓取候选', 0.35)
        pre_grasp = self.make_pre_grasp_pose(
            target_pose,
            goal.approach_distance_m or self.default_approach_distance_m,
        )

        self.publish_feedback(goal_handle, feedback, '规划到预抓取位姿', 0.55)
        self.sleep_step(0.4)
        self.publish_feedback(goal_handle, feedback, '执行抓取闭合', 0.80)
        self.sleep_step(0.4)
        self.publish_feedback(goal_handle, feedback, '抓取完成', 1.00)

        goal_handle.succeed()
        result.success = True
        result.message = (
            'Piper pick 占位流程完成；真实执行需接入 MoveIt2 或厂家 SDK 后端。'
            if self.fake_execution else
            'Piper pick 请求已转交真实后端。'
        )
        result.executed_pose = pre_grasp
        return result

    def execute_place(self, goal_handle):
        goal = goal_handle.request
        feedback = PlaceObject.Feedback()
        result = PlaceObject.Result()

        self.publish_feedback(goal_handle, feedback, '停止底盘', 0.05)
        self.stop_base_for_arm_motion()
        self.publish_feedback(goal_handle, feedback, '申请 MoveIt2 控制 owner', 0.15)
        self.request_owner('moveit')
        if not self.check_execution_allowed(goal_handle, result):
            return result
        self.publish_feedback(goal_handle, feedback, '规划到放置位姿', 0.55)
        self.sleep_step(0.4)
        if goal.open_gripper:
            self.publish_feedback(goal_handle, feedback, '打开夹爪', 0.80)
            self.sleep_step(0.3)
        self.publish_feedback(goal_handle, feedback, '放置完成', 1.00)

        goal_handle.succeed()
        result.success = True
        result.message = (
            'Piper place 占位流程完成；真实执行需接入 MoveIt2 或厂家 SDK 后端。'
            if self.fake_execution else
            'Piper place 请求已转交真实后端。'
        )
        result.executed_pose = goal.target_pose
        return result

    def request_owner(self, owner):
        msg = String()
        msg.data = owner
        self.owner_pub.publish(msg)

    def check_execution_allowed(self, goal_handle, result):
        state = self.parse_control_state()
        if state.get('estop') == 'true':
            goal_handle.abort()
            result.success = False
            result.message = 'Piper 处于急停状态，拒绝执行任务。'
            return False
        if self.fake_execution:
            return True
        if not self.real_backend_connected:
            goal_handle.abort()
            result.success = False
            result.message = 'Piper 真实 MoveIt2/SDK 后端尚未接入，拒绝假装执行成功。'
            return False
        if not self.wait_for_moveit_owner_ready():
            goal_handle.abort()
            result.success = False
            result.message = 'Piper 控制桥未进入 enabled=true 且 owner=moveit 状态。'
            return False
        return True

    def wait_for_moveit_owner_ready(self):
        deadline = time.monotonic() + max(0.0, self.control_ready_timeout_s)
        while rclpy.ok() and time.monotonic() <= deadline:
            state = self.parse_control_state()
            if (
                state.get('owner') == 'moveit'
                and state.get('enabled') == 'true'
                and state.get('estop') != 'true'
            ):
                return True
            time.sleep(0.05)
        return False

    def parse_control_state(self):
        parsed = {}
        for item in self.control_state.split(';'):
            if '=' not in item:
                continue
            key, value = item.split('=', 1)
            parsed[key.strip()] = value.strip()
        return parsed

    def stop_base_for_arm_motion(self):
        if not self.publish_base_stop:
            return
        zero = Twist()
        for _ in range(5):
            self.cmd_vel_pub.publish(zero)
            time.sleep(0.02)

    def publish_feedback(self, goal_handle, feedback, stage, progress):
        feedback.stage = stage
        feedback.progress = float(progress)
        goal_handle.publish_feedback(feedback)
        self.get_logger().info(f'Piper 任务阶段: {stage} ({progress:.0%})')

    def sleep_step(self, seconds):
        end_time = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end_time:
            time.sleep(0.05)

    @staticmethod
    def make_pre_grasp_pose(target_pose, approach_distance):
        pose = PoseStamped()
        pose.header = target_pose.header
        pose.pose = target_pose.pose
        # 占位策略：沿 piper_base_link 的 x 负方向后撤，真实项目应使用 TCP approach 向量。
        pose.pose.position.x -= abs(float(approach_distance))
        return pose


def main():
    rclpy.init()
    node = PiperTaskServerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
