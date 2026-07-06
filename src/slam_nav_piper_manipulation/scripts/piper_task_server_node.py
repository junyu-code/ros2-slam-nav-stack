#!/usr/bin/env python3

import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Point, PoseStamped, Twist
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from moveit_msgs.srv import GetMotionPlan
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from slam_nav_piper_interfaces.action import PickObject, PlaceObject
from slam_nav_piper_interfaces.msg import GraspCandidate, GraspCandidateArray
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from vision_msgs.msg import Detection3DArray


class PiperTaskServerNode(Node):
    """Piper 抓取/放置任务层，占位执行 MoveIt2 或 SDK 后端。"""

    def __init__(self):
        super().__init__('piper_task_server_node')
        self.declare_parameter('target_pose_topic', '/piper/perception/target_pose')
        self.declare_parameter('detections_3d_topic', '/piper/perception/detections_3d')
        self.declare_parameter('grasp_candidates_topic', '/piper/grasp_candidates')
        self.declare_parameter('visualization_markers_topic', '/piper/visualization/grasp_candidates')
        self.declare_parameter('control_state_topic', '/piper/control/state')
        self.declare_parameter('owner_request_topic', '/piper/control/owner_request')
        self.declare_parameter('base_cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('publish_base_stop', False)
        self.declare_parameter('require_base_stop_before_motion', True)
        self.declare_parameter('base_stop_confirmed', False)
        self.declare_parameter('fake_execution', True)
        self.declare_parameter('default_approach_distance_m', 0.10)
        self.declare_parameter('default_gripper_width_m', 0.06)
        self.declare_parameter('use_ranked_grasp_candidates', False)
        self.declare_parameter('ranked_grasp_candidates_topic', '/piper/learning/grasp_candidates_ranked')
        self.declare_parameter('real_backend_connected', False)
        self.declare_parameter('control_ready_timeout_s', 2.0)
        self.declare_parameter('require_hand_eye_calibration_before_pick', True)
        self.declare_parameter('hand_eye_calibrated', False)
        self.declare_parameter('hand_eye_result_must_exist', True)
        self.declare_parameter('hand_eye_result_path', 'datasets/piper_hand_eye/piper_eye_in_hand.yaml')
        self.declare_parameter('require_moveit_plan_before_fake_execution', False)
        self.declare_parameter('moveit_plan_service', '/piper/plan_kinematic_path')
        self.declare_parameter('moveit_planning_group', 'piper_arm')
        self.declare_parameter('moveit_planner_id', 'RRTConnect')
        self.declare_parameter('moveit_plan_joint_names', [
            'piper_joint1',
            'piper_joint2',
            'piper_joint3',
            'piper_joint4',
            'piper_joint5',
            'piper_joint6',
        ])
        self.declare_parameter('moveit_plan_target_positions', [0.0, 0.4164, -0.5409, 0.0, 0.0, 0.0])
        self.declare_parameter('moveit_plan_joint_tolerance', 0.02)
        self.declare_parameter('moveit_plan_allowed_time_s', 5.0)
        self.declare_parameter('moveit_plan_attempts', 5)
        self.declare_parameter('moveit_plan_service_timeout_s', 10.0)

        self.latest_target_pose = None
        self.latest_detection_3d = None
        self.latest_ranked_candidates = None
        self.control_state = ''
        self.last_plan_only_gate_summary = ''
        self.fake_execution = bool(self.get_parameter('fake_execution').value)
        self.real_backend_connected = bool(self.get_parameter('real_backend_connected').value)
        self.control_ready_timeout_s = float(self.get_parameter('control_ready_timeout_s').value)
        self.require_hand_eye_calibration_before_pick = bool(
            self.get_parameter('require_hand_eye_calibration_before_pick').value
        )
        self.hand_eye_calibrated = bool(self.get_parameter('hand_eye_calibrated').value)
        self.hand_eye_result_must_exist = bool(self.get_parameter('hand_eye_result_must_exist').value)
        self.hand_eye_result_path = str(self.get_parameter('hand_eye_result_path').value)
        self.publish_base_stop = bool(self.get_parameter('publish_base_stop').value)
        self.require_base_stop_before_motion = bool(
            self.get_parameter('require_base_stop_before_motion').value
        )
        self.base_stop_confirmed = bool(self.get_parameter('base_stop_confirmed').value)
        self.default_approach_distance_m = float(
            self.get_parameter('default_approach_distance_m').value
        )
        self.default_gripper_width_m = float(self.get_parameter('default_gripper_width_m').value)
        self.use_ranked_grasp_candidates = bool(
            self.get_parameter('use_ranked_grasp_candidates').value
        )
        self.require_moveit_plan_before_fake_execution = bool(
            self.get_parameter('require_moveit_plan_before_fake_execution').value
        )
        self.moveit_plan_service = str(self.get_parameter('moveit_plan_service').value)
        self.moveit_plan_timeout_s = float(self.get_parameter('moveit_plan_service_timeout_s').value)
        self.callback_group = ReentrantCallbackGroup()
        self.moveit_plan_client = self.create_client(
            GetMotionPlan,
            self.moveit_plan_service,
            callback_group=self.callback_group,
        )

        self.create_subscription(
            PoseStamped,
            str(self.get_parameter('target_pose_topic').value),
            self.target_pose_callback,
            10,
        )
        self.create_subscription(
            Detection3DArray,
            str(self.get_parameter('detections_3d_topic').value),
            self.detections_3d_callback,
            10,
        )
        if self.use_ranked_grasp_candidates:
            self.create_subscription(
                GraspCandidateArray,
                str(self.get_parameter('ranked_grasp_candidates_topic').value),
                self.ranked_grasp_candidates_callback,
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
        self.marker_pub = self.create_publisher(
            MarkerArray,
            str(self.get_parameter('visualization_markers_topic').value),
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
            callback_group=self.callback_group,
        )
        self.place_server = ActionServer(
            self,
            PlaceObject,
            '/piper/task/place_object',
            self.execute_place,
            callback_group=self.callback_group,
        )
        self.candidate_timer = self.create_timer(0.5, self.publish_grasp_candidate)

        self.get_logger().info(
            f'Piper 任务 action server 已启动，fake_execution={self.fake_execution}。'
        )
        if self.use_ranked_grasp_candidates:
            self.get_logger().warn(
                'Piper 任务层已显式打开学习排序候选消费；确认当前只用于仿真/离线验收。'
            )
        else:
            self.get_logger().info('Piper 任务层未接入学习排序结果，继续使用原始 grasp candidates。')
        if not self.fake_execution and not self.real_backend_connected:
            self.get_logger().warn('Piper fake_execution=false，但真实后端未声明接入；任务会安全拒绝执行。')
        if self.require_moveit_plan_before_fake_execution:
            self.get_logger().warn(
                'Piper 已打开 MoveIt2 plan-only 门禁：fake pick/place 会先请求规划，但仍不执行轨迹。'
            )
        if self.require_hand_eye_calibration_before_pick and not self.hand_eye_calibrated:
            self.get_logger().info('Piper 真实 pick 前要求手眼标定验收；当前 hand_eye_calibrated=false。')
        if self.require_base_stop_before_motion and not (self.base_stop_confirmed or self.publish_base_stop):
            self.get_logger().info('Piper 真实执行前要求底盘停稳或显式发布停车；当前尚未确认。')

    def target_pose_callback(self, msg):
        self.latest_target_pose = msg

    def detections_3d_callback(self, msg):
        if msg.detections:
            self.latest_detection_3d = msg.detections[0]

    def ranked_grasp_candidates_callback(self, msg):
        if msg.candidates:
            self.latest_ranked_candidates = msg

    def control_state_callback(self, msg):
        self.control_state = msg.data

    def publish_grasp_candidate(self):
        if self.latest_target_pose is None:
            return
        array = GraspCandidateArray()
        array.header = self.latest_target_pose.header
        detection_metadata = self.latest_detection_metadata()
        grasp_pose = self.pose_from_latest_detection() or self.latest_target_pose
        candidate = GraspCandidate()
        candidate.header = grasp_pose.header
        candidate.object_id = detection_metadata['object_id']
        candidate.object_class = detection_metadata['object_class']
        candidate.grasp_pose = grasp_pose
        candidate.pre_grasp_pose = self.make_pre_grasp_pose(
            grasp_pose,
            self.default_approach_distance_m,
        )
        candidate.score = detection_metadata['score']
        candidate.gripper_width_m = self.default_gripper_width_m
        candidate.approach_distance_m = self.default_approach_distance_m
        candidate.source_frame = grasp_pose.header.frame_id
        candidate.tags = detection_metadata['tags']
        array.candidates.append(candidate)
        self.grasp_pub.publish(array)
        self.publish_grasp_markers(array)

    def publish_grasp_markers(self, candidates):
        marker_array = MarkerArray()
        for index, candidate in enumerate(candidates.candidates[:5]):
            base_id = index * 10
            marker_array.markers.append(
                self.make_pose_marker(
                    candidate.grasp_pose,
                    'grasp_pose',
                    base_id,
                    Marker.SPHERE,
                    (0.055, 0.055, 0.055),
                    (0.1, 0.95, 0.35, 0.85),
                )
            )
            marker_array.markers.append(
                self.make_pose_marker(
                    candidate.pre_grasp_pose,
                    'pre_grasp_pose',
                    base_id + 1,
                    Marker.SPHERE,
                    (0.040, 0.040, 0.040),
                    (0.1, 0.45, 1.0, 0.75),
                )
            )
            marker_array.markers.append(self.make_approach_marker(candidate, base_id + 2))
            marker_array.markers.append(self.make_label_marker(candidate, base_id + 3))
        self.marker_pub.publish(marker_array)

    def make_pose_marker(self, pose_stamped, namespace, marker_id, marker_type, scale, color):
        marker = Marker()
        marker.header = pose_stamped.header
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose = pose_stamped.pose
        marker.scale.x = float(scale[0])
        marker.scale.y = float(scale[1])
        marker.scale.z = float(scale[2])
        marker.color.r = float(color[0])
        marker.color.g = float(color[1])
        marker.color.b = float(color[2])
        marker.color.a = float(color[3])
        marker.lifetime.sec = 1
        return marker

    def make_approach_marker(self, candidate, marker_id):
        marker = Marker()
        marker.header = candidate.header
        marker.ns = 'approach_vector'
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.points = [
            self.point_from_pose(candidate.pre_grasp_pose),
            self.point_from_pose(candidate.grasp_pose),
        ]
        marker.scale.x = 0.012
        marker.scale.y = 0.030
        marker.scale.z = 0.050
        marker.color.r = 0.0
        marker.color.g = 0.85
        marker.color.b = 1.0
        marker.color.a = 0.85
        marker.lifetime.sec = 1
        return marker

    def make_label_marker(self, candidate, marker_id):
        marker = self.make_pose_marker(
            candidate.grasp_pose,
            'grasp_label',
            marker_id,
            Marker.TEXT_VIEW_FACING,
            (0.0, 0.0, 0.040),
            (1.0, 1.0, 1.0, 0.95),
        )
        marker.pose.position.z += 0.08
        marker.text = f'{candidate.object_class}:{candidate.score:.2f}'
        return marker

    def latest_detection_metadata(self):
        detection = self.latest_detection_3d
        metadata = {
            'object_id': 'latest_target',
            'object_class': 'unknown',
            'score': 0.50,
            'tags': ['target_pose_fallback'],
        }
        if detection is None:
            return metadata

        object_class = 'unknown'
        score = 0.50
        if detection.results:
            hypothesis = detection.results[0].hypothesis
            object_class = hypothesis.class_id or object_class
            score = float(hypothesis.score)

        metadata['object_id'] = detection.id or object_class or 'latest_target'
        metadata['object_class'] = object_class
        metadata['score'] = score
        # 这些 tag 让学习/调试层能知道候选来自 3D 检测，而不是纯 target_pose fallback。
        metadata['tags'] = ['detection_3d', f'class:{object_class}', f'source:{detection.header.frame_id}']
        return metadata

    def pose_from_latest_detection(self):
        detection = self.latest_detection_3d
        if detection is None or not detection.header.frame_id:
            return None
        pose = PoseStamped()
        pose.header = detection.header
        pose.pose = detection.bbox.center
        return pose

    def best_ranked_candidate(self):
        if not self.use_ranked_grasp_candidates or self.latest_ranked_candidates is None:
            return None
        if not self.latest_ranked_candidates.candidates:
            return None
        return self.latest_ranked_candidates.candidates[0]

    def resolve_pick_target(self, requested_pose):
        if requested_pose.header.frame_id:
            return requested_pose, None
        ranked_candidate = self.best_ranked_candidate()
        if ranked_candidate is not None and ranked_candidate.grasp_pose.header.frame_id:
            return ranked_candidate.grasp_pose, ranked_candidate
        return self.latest_target_pose, None

    def execute_pick(self, goal_handle):
        goal = goal_handle.request
        feedback = PickObject.Feedback()
        result = PickObject.Result()

        self.publish_feedback(goal_handle, feedback, '停止底盘', 0.05)
        self.stop_base_for_arm_motion()

        self.publish_feedback(goal_handle, feedback, '申请 MoveIt2 控制 owner', 0.15)
        self.request_owner('moveit')
        if not self.check_execution_allowed(goal_handle, result, operation='pick'):
            return result

        target_pose, ranked_candidate = self.resolve_pick_target(goal.target_pose)
        if target_pose is None or not target_pose.header.frame_id:
            goal_handle.abort()
            result.success = False
            result.message = '没有目标位姿或 ranked 抓取候选，无法执行 pick。'
            return result

        self.publish_feedback(goal_handle, feedback, '生成抓取候选', 0.35)
        if ranked_candidate is not None and ranked_candidate.pre_grasp_pose.header.frame_id:
            pre_grasp = ranked_candidate.pre_grasp_pose
            self.get_logger().info(
                f'Piper pick 使用 ranked 抓取候选：id={ranked_candidate.object_id}, '
                f'score={ranked_candidate.score:.2f}'
            )
        else:
            pre_grasp = self.make_pre_grasp_pose(
                target_pose,
                goal.approach_distance_m or self.default_approach_distance_m,
            )

        self.publish_feedback(goal_handle, feedback, '规划到预抓取位姿', 0.55)
        if not self.ensure_plan_only_gate(goal_handle, result, 'pick'):
            return result
        self.sleep_step(0.4)
        self.publish_feedback(goal_handle, feedback, '执行抓取闭合', 0.80)
        self.sleep_step(0.4)
        self.publish_feedback(goal_handle, feedback, '抓取完成', 1.00)

        goal_handle.succeed()
        result.success = True
        result.message = self.success_message('pick')
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
        if not self.check_execution_allowed(goal_handle, result, operation='place'):
            return result
        self.publish_feedback(goal_handle, feedback, '规划到放置位姿', 0.55)
        if not self.ensure_plan_only_gate(goal_handle, result, 'place'):
            return result
        self.sleep_step(0.4)
        if goal.open_gripper:
            self.publish_feedback(goal_handle, feedback, '打开夹爪', 0.80)
            self.sleep_step(0.3)
        self.publish_feedback(goal_handle, feedback, '放置完成', 1.00)

        goal_handle.succeed()
        result.success = True
        result.message = self.success_message('place')
        result.executed_pose = goal.target_pose
        return result

    def ensure_plan_only_gate(self, goal_handle, result, operation):
        if not self.require_moveit_plan_before_fake_execution:
            return True

        success, message = self.request_plan_only()
        if not success:
            goal_handle.abort()
            result.success = False
            result.message = f'Piper {operation} MoveIt2 plan-only 门禁失败：{message}'
            return False
        self.last_plan_only_gate_summary = message
        self.get_logger().info(f'Piper {operation} MoveIt2 plan-only 门禁通过：{message}')
        return True

    def success_message(self, operation):
        if self.fake_execution:
            message = f'Piper {operation} 占位流程完成；真实执行需接入 MoveIt2 或厂家 SDK 后端。'
        else:
            message = f'Piper {operation} 请求已转交真实后端。'
        if self.require_moveit_plan_before_fake_execution:
            message += f' MoveIt2 plan-only 门禁已通过：{self.last_plan_only_gate_summary}'
        return message

    def request_plan_only(self):
        if not self.moveit_plan_client.wait_for_service(timeout_sec=self.moveit_plan_timeout_s):
            return False, f'等待规划服务超时: {self.moveit_plan_service}'

        try:
            request = self.build_moveit_plan_request()
        except Exception as exc:
            return False, f'构造规划请求失败: {exc}'

        future = self.moveit_plan_client.call_async(request)
        deadline = time.monotonic() + self.moveit_plan_timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            if future.done():
                break
            time.sleep(0.02)

        if not future.done():
            return False, '规划服务响应超时。'
        response = future.result()
        if response is None:
            return False, '规划服务没有返回结果。'

        motion_response = response.motion_plan_response
        error_code = motion_response.error_code.val
        point_count = len(motion_response.trajectory.joint_trajectory.points)
        if error_code != MoveItErrorCodes.SUCCESS:
            return False, f'MoveIt error_code={error_code}'
        if point_count == 0:
            return False, '规划成功但轨迹点为空。'
        return True, f'轨迹点数={point_count}, planning_time={motion_response.planning_time:.3f}s'

    def build_moveit_plan_request(self):
        joint_names = [str(item) for item in self.get_parameter('moveit_plan_joint_names').value]
        target_positions = [
            float(item)
            for item in self.get_parameter('moveit_plan_target_positions').value
        ]
        if len(joint_names) != len(target_positions):
            raise RuntimeError('moveit_plan_joint_names 与 moveit_plan_target_positions 长度必须一致。')

        request = GetMotionPlan.Request()
        motion_request = request.motion_plan_request
        motion_request.group_name = str(self.get_parameter('moveit_planning_group').value)
        motion_request.pipeline_id = 'ompl'
        motion_request.planner_id = str(self.get_parameter('moveit_planner_id').value)
        motion_request.num_planning_attempts = int(self.get_parameter('moveit_plan_attempts').value)
        motion_request.allowed_planning_time = float(self.get_parameter('moveit_plan_allowed_time_s').value)
        motion_request.max_velocity_scaling_factor = 0.1
        motion_request.max_acceleration_scaling_factor = 0.1
        # 只做 plan-only：起点来自 MoveIt2 当前状态，返回轨迹不会被执行。
        motion_request.start_state.is_diff = True
        motion_request.goal_constraints = [self.build_moveit_goal_constraints(joint_names, target_positions)]
        return request

    def build_moveit_goal_constraints(self, joint_names, target_positions):
        tolerance = float(self.get_parameter('moveit_plan_joint_tolerance').value)
        constraints = Constraints()
        constraints.name = 'piper_task_plan_only_gate'
        for joint_name, target_position in zip(joint_names, target_positions):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = target_position
            joint_constraint.tolerance_above = tolerance
            joint_constraint.tolerance_below = tolerance
            joint_constraint.weight = 1.0
            constraints.joint_constraints.append(joint_constraint)
        return constraints

    def request_owner(self, owner):
        msg = String()
        msg.data = owner
        self.owner_pub.publish(msg)

    def check_execution_allowed(self, goal_handle, result, operation):
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
        if operation == 'pick' and not self.hand_eye_ready_for_pick():
            goal_handle.abort()
            result.success = False
            result.message = (
                'Piper 手眼标定尚未验收，拒绝真实 pick；'
                '请先完成 /piper/arm_camera eye-in-hand 标定并人工确认结果。'
            )
            return False
        if not self.base_ready_for_arm_motion():
            goal_handle.abort()
            result.success = False
            result.message = (
                'Piper 底盘停止/导航暂停状态尚未确认，拒绝真实机械臂运动；'
                '请先暂停 Nav2 或显式打开 publish_base_stop。'
            )
            return False
        if not self.wait_for_moveit_owner_ready():
            goal_handle.abort()
            result.success = False
            result.message = 'Piper 控制桥未进入 enabled=true 且 owner=moveit 状态。'
            return False
        return True

    def hand_eye_ready_for_pick(self):
        if not self.require_hand_eye_calibration_before_pick:
            self.get_logger().warn('真实 pick 已关闭手眼标定前置检查，请确认这是隔离测试。')
            return True
        if not self.hand_eye_calibrated:
            return False
        if not self.hand_eye_result_must_exist:
            return True

        result_path = Path(self.hand_eye_result_path).expanduser()
        if not result_path.is_absolute():
            result_path = Path.cwd() / result_path
        if result_path.exists():
            return True
        self.get_logger().warn(f'手眼标定结果文件不存在: {result_path}')
        return False

    def base_ready_for_arm_motion(self):
        if not self.require_base_stop_before_motion:
            self.get_logger().warn('真实机械臂运动已关闭底盘停止前置检查，请确认这是隔离测试。')
            return True
        if self.base_stop_confirmed:
            return True
        if self.publish_base_stop:
            # 发布停车指令代表任务层显式持有底盘停止意图，实机仍应由上层暂停 Nav2。
            return True
        return False

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

    @staticmethod
    def point_from_pose(pose_stamped):
        point = Point()
        point.x = pose_stamped.pose.position.x
        point.y = pose_stamped.pose.position.y
        point.z = pose_stamped.pose.position.z
        return point


def main():
    rclpy.init()
    node = PiperTaskServerNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
