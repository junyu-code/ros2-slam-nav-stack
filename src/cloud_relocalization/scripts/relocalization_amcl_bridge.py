#!/usr/bin/env python3

import math
import time
from collections import deque
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped, PoseWithCovarianceStamped, TransformStamped
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import Trigger
import tf2_ros


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def quaternion_to_matrix(x: float, y: float, z: float, w: float):
    # 四元数转旋转矩阵，避免额外依赖 tf_transformations。
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-9:
        return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return [
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ]


def matrix_to_quaternion(m):
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2][1] - m[1][2]) / s
        y = (m[0][2] - m[2][0]) / s
        z = (m[1][0] - m[0][1]) / s
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
        w = (m[2][1] - m[1][2]) / s
        x = 0.25 * s
        y = (m[0][1] + m[1][0]) / s
        z = (m[0][2] + m[2][0]) / s
    elif m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
        w = (m[0][2] - m[2][0]) / s
        x = (m[0][1] + m[1][0]) / s
        y = 0.25 * s
        z = (m[1][2] + m[2][1]) / s
    else:
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
        w = (m[1][0] - m[0][1]) / s
        x = (m[0][2] + m[2][0]) / s
        y = (m[1][2] + m[2][1]) / s
        z = 0.25 * s
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-9:
        return 0.0, 0.0, 0.0, 1.0
    return x / norm, y / norm, z / norm, w / norm


def yaw_from_matrix(t) -> float:
    return math.atan2(t[1][0], t[0][0])


def identity_transform():
    return [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def multiply(a, b):
    out = identity_transform()
    for r in range(4):
        for c in range(4):
            out[r][c] = sum(a[r][k] * b[k][c] for k in range(4))
    return out


def inverse(t):
    out = identity_transform()
    for r in range(3):
        for c in range(3):
            out[r][c] = t[c][r]
    for r in range(3):
        out[r][3] = -sum(out[r][c] * t[c][3] for c in range(3))
    return out


def geometry_pose_to_transform(pose):
    q = pose.orientation
    p = pose.position
    rot = quaternion_to_matrix(q.x, q.y, q.z, q.w)
    out = identity_transform()
    for r in range(3):
        for c in range(3):
            out[r][c] = rot[r][c]
    out[0][3] = p.x
    out[1][3] = p.y
    out[2][3] = p.z
    return out


def pose_to_transform(msg: PoseStamped):
    return geometry_pose_to_transform(msg.pose)


def tf_to_transform(msg: TransformStamped):
    q = msg.transform.rotation
    p = msg.transform.translation
    rot = quaternion_to_matrix(q.x, q.y, q.z, q.w)
    out = identity_transform()
    for r in range(3):
        for c in range(3):
            out[r][c] = rot[r][c]
    out[0][3] = p.x
    out[1][3] = p.y
    out[2][3] = p.z
    return out


def transform_delta(a, b) -> Tuple[float, float]:
    delta = multiply(inverse(a), b)
    translation = math.hypot(delta[0][3], delta[1][3])
    yaw = abs(normalize_angle(yaw_from_matrix(delta)))
    return translation, yaw


class RelocalizationAmclBridge(Node):
    """把点云重定位结果接入导航定位链路。

    默认模式仍然兼容把结果回灌给 AMCL 的 /initialpose；增强模式下，节点会在
    AMCL 启动收敛后接管并冻结 map->odom，后续只在异常恢复时用 GICP/ICP/NDT
    结果更新该变换，避免 AMCL 在正常行驶过程中持续拉扯 FAST-LIO 位姿。
    """

    def __init__(self):
        super().__init__('relocalization_amcl_bridge')

        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_footprint').value
        self.pose_topic = self.declare_parameter('relocalization_pose_topic', '/relocalization/pose').value
        self.status_topic = self.declare_parameter('relocalization_status_topic', '/relocalization/status').value
        self.fault_topic = self.declare_parameter('fault_topic', '/localization_fault').value
        self.initialpose_topic = self.declare_parameter('initialpose_topic', '/initialpose').value
        self.trigger_service = self.declare_parameter('trigger_service', '/relocalization/trigger').value
        self.odom_topic = self.declare_parameter('odom_topic', '/Odometry').value
        self.amcl_converged_topic = self.declare_parameter(
            'amcl_converged_topic', '/amcl_converged'
        ).value
        self.amcl_score_topic = self.declare_parameter(
            'amcl_convergence_score_topic', '/amcl_convergence_score'
        ).value
        self.map_topic = self.declare_parameter('map_topic', '/map').value
        self.output_mode = str(self.declare_parameter('output_mode', 'initialpose').value).lower()
        self.manage_map_to_odom_tf = bool(
            self.declare_parameter('manage_map_to_odom_tf', False).value
        )
        self.bootstrap_requires_amcl_convergence = bool(
            self.declare_parameter('bootstrap_requires_amcl_convergence', True).value
        )
        self.bootstrap_timeout_sec = float(
            self.declare_parameter('bootstrap_timeout_sec', 45.0).value
        )
        self.bootstrap_min_age_sec = float(
            self.declare_parameter('bootstrap_min_age_sec', 0.0).value
        )
        self.require_still_for_bootstrap_handoff = bool(
            self.declare_parameter('require_still_for_bootstrap_handoff', True).value
        )
        self.bootstrap_linear_threshold = float(
            self.declare_parameter('bootstrap_linear_threshold', 0.03).value
        )
        self.bootstrap_angular_threshold = float(
            self.declare_parameter('bootstrap_angular_threshold', 0.05).value
        )
        self.deactivate_amcl_after_bootstrap = bool(
            self.declare_parameter('deactivate_amcl_after_bootstrap', False).value
        )
        self.amcl_node_name = self.declare_parameter('amcl_node_name', '/amcl').value
        self.tf_publish_rate_hz = float(self.declare_parameter('tf_publish_rate_hz', 20.0).value)
        self.use_amcl_quality_for_low_speed = bool(
            self.declare_parameter('use_amcl_quality_for_low_speed', True).value
        )
        self.particle_cloud_topic = self.declare_parameter(
            'particle_cloud_topic', '/particle_cloud'
        ).value
        self.clear_particle_cloud_after_bootstrap = bool(
            self.declare_parameter('clear_particle_cloud_after_bootstrap', True).value
        )
        self.particle_clear_publish_count = int(
            self.declare_parameter('particle_clear_publish_count', 3).value
        )

        self.trigger_period_sec = float(self.declare_parameter('trigger_period_sec', 8.0).value)
        self.trigger_on_fault = bool(self.declare_parameter('trigger_on_fault', True).value)
        self.publish_on_fault = bool(self.declare_parameter('publish_on_fault', True).value)
        self.publish_on_correction = bool(self.declare_parameter('publish_on_correction', True).value)
        self.correction_translation_threshold = float(
            self.declare_parameter('correction_translation_threshold', 0.35).value
        )
        self.correction_yaw_threshold = float(
            self.declare_parameter('correction_yaw_threshold', 0.20).value
        )
        self.max_correction_translation = float(
            self.declare_parameter('max_correction_translation', 2.5).value
        )
        self.max_correction_yaw = float(self.declare_parameter('max_correction_yaw', 1.2).value)
        self.min_publish_interval_sec = float(
            self.declare_parameter('min_publish_interval_sec', 8.0).value
        )
        self.xy_stddev = float(self.declare_parameter('xy_stddev', 0.30).value)
        self.yaw_stddev = float(self.declare_parameter('yaw_stddev', 0.45).value)

        self.trigger_on_low_speed = bool(
            self.declare_parameter('trigger_on_low_speed', False).value
        )
        self.low_speed_linear_threshold = float(
            self.declare_parameter('low_speed_linear_threshold', 0.12).value
        )
        self.low_speed_angular_threshold = float(
            self.declare_parameter('low_speed_angular_threshold', 0.20).value
        )
        self.low_speed_hold_sec = float(
            self.declare_parameter('low_speed_hold_sec', 1.5).value
        )
        self.low_speed_cooldown_sec = float(
            self.declare_parameter('low_speed_cooldown_sec', 12.0).value
        )
        self.low_speed_score_threshold = float(
            self.declare_parameter('low_speed_score_threshold', 85.0).value
        )
        self.low_speed_start_delay_sec = float(
            self.declare_parameter('low_speed_start_delay_sec', 12.0).value
        )
        self.odom_timeout_sec = float(self.declare_parameter('odom_timeout_sec', 1.0).value)
        self.sensor_frame = self.declare_parameter('sensor_frame', 'livox_frame').value
        self.sensor_still_reference_frame = self.declare_parameter(
            'sensor_still_reference_frame', self.odom_frame
        ).value
        self.require_sensor_still_for_low_speed = bool(
            self.declare_parameter('require_sensor_still_for_low_speed', True).value
        )
        self.sensor_still_window_sec = float(
            self.declare_parameter('sensor_still_window_sec', 1.5).value
        )
        self.sensor_still_translation_threshold = float(
            self.declare_parameter('sensor_still_translation_threshold', 0.03).value
        )
        self.sensor_still_yaw_threshold = float(
            self.declare_parameter('sensor_still_yaw_threshold', 0.03).value
        )
        self.sensor_still_min_samples = int(
            self.declare_parameter('sensor_still_min_samples', 3).value
        )
        self.require_sensor_still_for_bootstrap = bool(
            self.declare_parameter(
                'require_sensor_still_for_bootstrap',
                self.require_sensor_still_for_low_speed,
            ).value
        )
        self.require_known_area_for_relocalization = bool(
            self.declare_parameter('require_known_area_for_relocalization', True).value
        )
        self.known_area_check_radius_m = float(
            self.declare_parameter('known_area_check_radius_m', 0.80).value
        )
        self.known_area_min_known_fraction = float(
            self.declare_parameter('known_area_min_known_fraction', 0.75).value
        )

        self.node_start_time = time.monotonic()
        self.last_trigger_time = 0.0
        self.last_publish_time = 0.0
        self.last_low_speed_trigger_time = 0.0
        self.fault_active = False
        self.last_status = ''
        self.last_linear_speed = None
        self.last_angular_speed = None
        self.last_odom_time = None
        self.low_speed_since = None
        self.amcl_converged = False
        self.amcl_converged_seen = False
        self.last_amcl_score = None
        self.last_amcl_score_time = None
        self.sensor_tf_samples = deque()
        self.sensor_still_ready = False
        self.sensor_still = False
        self.sensor_translation_span = None
        self.sensor_yaw_span = None
        self.map_msg = None
        self.managed_map_odom = None
        self.managed_tf_active = False
        self.bootstrap_started = time.monotonic()
        self.bootstrap_map_odom_captured = False
        self.amcl_deactivate_future = None
        self.amcl_deactivate_started = None
        self.amcl_get_state_future = None
        self.bootstrap_failed = False
        self.particle_cloud_cleared = False

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.trigger_client = self.create_client(Trigger, self.trigger_service)
        self.amcl_change_state_client = self.create_client(
            ChangeState, self._change_state_service_name(self.amcl_node_name)
        )
        self.amcl_get_state_client = self.create_client(
            GetState, self._get_state_service_name(self.amcl_node_name)
        )
        self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, self.initialpose_topic, 10)
        self.particle_cloud_pub = self.create_publisher(PoseArray, self.particle_cloud_topic, 10)

        self.create_subscription(PoseStamped, self.pose_topic, self._pose_callback, 10)
        self.create_subscription(String, self.status_topic, self._status_callback, 10)
        self.create_subscription(Bool, self.fault_topic, self._fault_callback, 10)
        self.create_subscription(
            Odometry, self.odom_topic, self._odom_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            Bool, self.amcl_converged_topic, self._amcl_converged_callback, 10
        )
        self.create_subscription(Float32, self.amcl_score_topic, self._amcl_score_callback, 10)
        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, self.map_topic, self._map_callback, map_qos)
        self.create_timer(0.5, self._timer_callback)
        self.create_timer(1.0 / max(self.tf_publish_rate_hz, 1.0), self._publish_managed_tf)

        self.get_logger().info(
            'Relocalization bridge started: '
            f'pose={self.pose_topic}, trigger={self.trigger_service}, '
            f'output_mode={self.output_mode}, managed_tf={self.manage_map_to_odom_tf}, '
            f'low_speed_gate={self.trigger_on_low_speed}'
        )

    def _status_callback(self, msg: String):
        self.last_status = msg.data

    def _fault_callback(self, msg: Bool):
        was_fault = self.fault_active
        self.fault_active = bool(msg.data)
        if self.fault_active and not was_fault and self.trigger_on_fault:
            self._trigger_relocalization('localization_fault')

    def _odom_callback(self, msg: Odometry):
        now = time.monotonic()
        linear = msg.twist.twist.linear
        angular = msg.twist.twist.angular
        self.last_linear_speed = math.sqrt(
            linear.x * linear.x + linear.y * linear.y + linear.z * linear.z
        )
        self.last_angular_speed = abs(angular.z)
        self.last_odom_time = now

    def _amcl_converged_callback(self, msg: Bool):
        self.amcl_converged = bool(msg.data)
        self.amcl_converged_seen = True

    def _amcl_score_callback(self, msg: Float32):
        self.last_amcl_score = float(msg.data)
        self.last_amcl_score_time = time.monotonic()

    def _map_callback(self, msg: OccupancyGrid):
        self.map_msg = msg

    def _timer_callback(self):
        now = time.monotonic()
        self._update_sensor_stillness(now)
        self._maybe_bootstrap_managed_tf(now)
        if self.trigger_period_sec > 0.0 and now - self.last_trigger_time >= self.trigger_period_sec:
            self._trigger_relocalization('periodic_tracking')
        self._maybe_trigger_low_speed_relocalization(now)


    def _maybe_bootstrap_managed_tf(self, now: float):
        if self.output_mode != 'tf' or not self.manage_map_to_odom_tf:
            return
        if self.managed_tf_active or self.bootstrap_failed:
            return
        if self.bootstrap_requires_amcl_convergence and not self.amcl_converged:
            if now - self.bootstrap_started > self.bootstrap_timeout_sec:
                self.get_logger().warn(
                    'Waiting for AMCL bootstrap convergence before freezing map->odom',
                    throttle_duration_sec=5.0,
                )
            return
        if now - self.bootstrap_started < self.bootstrap_min_age_sec:
            self.get_logger().info(
                'Waiting for bootstrap grace period before freezing map->odom.',
                throttle_duration_sec=5.0,
            )
            return
        if not self._bootstrap_handoff_motion_is_still(now):
            return

        if self.managed_map_odom is None:
            transform = self._lookup_transform(self.map_frame, self.odom_frame, log_wait=False)
            if transform is None:
                self.get_logger().info(
                    f'Waiting for bootstrap TF {self.map_frame}->{self.odom_frame}',
                    throttle_duration_sec=5.0,
                )
                return
            self.managed_map_odom = transform
            self.bootstrap_map_odom_captured = True
            self.get_logger().info('Captured AMCL bootstrap map->odom; preparing to freeze TF.')

        if self.deactivate_amcl_after_bootstrap:
            if self.amcl_deactivate_future is None:
                if not self.amcl_change_state_client.service_is_ready():
                    self.get_logger().info(
                        f'Waiting for AMCL lifecycle service {self.amcl_node_name}/change_state',
                        throttle_duration_sec=5.0,
                    )
                    return
                request = ChangeState.Request()
                request.transition.id = Transition.TRANSITION_DEACTIVATE
                request.transition.label = 'deactivate'
                self.amcl_deactivate_future = self.amcl_change_state_client.call_async(request)
                self.amcl_deactivate_started = now
                self.get_logger().info('Requested AMCL deactivate after bootstrap convergence.')
                return

            if not self.amcl_deactivate_future.done():
                self.get_logger().info('Waiting for AMCL deactivate response.', throttle_duration_sec=5.0)
                return

            try:
                response = self.amcl_deactivate_future.result()
            except Exception as exc:  # noqa: BLE001
                self.bootstrap_failed = True
                self.get_logger().warn(f'AMCL deactivate failed; keep AMCL as TF owner: {exc}')
                return
            if not response.success:
                self.bootstrap_failed = True
                self.get_logger().warn('AMCL deactivate was rejected; keep AMCL as TF owner.')
                return

            if self.amcl_get_state_future is None:
                if not self.amcl_get_state_client.service_is_ready():
                    self.get_logger().info(
                        f'Waiting for AMCL lifecycle service {self.amcl_node_name}/get_state',
                        throttle_duration_sec=5.0,
                    )
                    return
                self.amcl_get_state_future = self.amcl_get_state_client.call_async(GetState.Request())
                self.get_logger().info('Requested AMCL lifecycle state confirmation after deactivate.')
                return

            if not self.amcl_get_state_future.done():
                self.get_logger().info('Waiting for AMCL lifecycle state confirmation.', throttle_duration_sec=5.0)
                return

            try:
                state_response = self.amcl_get_state_future.result()
            except Exception as exc:  # noqa: BLE001
                self.bootstrap_failed = True
                self.get_logger().warn(f'AMCL state confirmation failed; keep AMCL as TF owner: {exc}')
                return

            state = state_response.current_state
            if state.id != State.PRIMARY_STATE_INACTIVE:
                self.bootstrap_failed = True
                self.get_logger().warn(
                    'AMCL deactivate did not reach inactive state; keep AMCL as TF owner: '
                    f'id={state.id}, label={state.label}'
                )
                return

            latest_transform = self._lookup_transform(self.map_frame, self.odom_frame, log_wait=False)
            if latest_transform is not None:
                old_transform = self.managed_map_odom
                self.managed_map_odom = latest_transform
                self.bootstrap_map_odom_captured = True
                if old_transform is not None:
                    shift_m, shift_yaw = transform_delta(old_transform, latest_transform)
                    self.get_logger().info(
                        'Refreshed bootstrap map->odom after AMCL became inactive: '
                        f'd={shift_m:.3f}m yaw={shift_yaw:.3f}rad'
                    )
                else:
                    self.get_logger().info('Captured final bootstrap map->odom after AMCL became inactive.')
            elif self.managed_map_odom is None:
                self.bootstrap_failed = True
                self.get_logger().warn(
                    'Cannot capture map->odom after AMCL became inactive; keep bootstrap handoff disabled.'
                )
                return

        self.managed_tf_active = True
        self._publish_managed_tf()
        self._clear_particle_cloud()
        self.get_logger().warn(
            'AMCL bootstrap finished. map->odom is now frozen and owned by relocalization bridge.'
        )

    def _publish_managed_tf(self):
        if self.output_mode != 'tf' or not self.manage_map_to_odom_tf:
            return
        if not self.managed_tf_active or self.managed_map_odom is None:
            return
        msg = self._matrix_to_transform_msg(self.managed_map_odom)
        self.tf_broadcaster.sendTransform(msg)

    def _matrix_to_transform_msg(self, matrix):
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.child_frame_id = self.odom_frame
        msg.transform.translation.x = float(matrix[0][3])
        msg.transform.translation.y = float(matrix[1][3])
        msg.transform.translation.z = float(matrix[2][3])
        qx, qy, qz, qw = matrix_to_quaternion(matrix)
        msg.transform.rotation.x = qx
        msg.transform.rotation.y = qy
        msg.transform.rotation.z = qz
        msg.transform.rotation.w = qw
        return msg

    @staticmethod
    def _change_state_service_name(node_name: str) -> str:
        node_name = str(node_name).strip()
        if not node_name.startswith('/'):
            node_name = '/' + node_name
        return node_name.rstrip('/') + '/change_state'

    @staticmethod
    def _get_state_service_name(node_name: str) -> str:
        node_name = str(node_name).strip()
        if not node_name.startswith('/'):
            node_name = '/' + node_name
        return node_name.rstrip('/') + '/get_state'

    def _bootstrap_handoff_motion_is_still(self, now: float) -> bool:
        if not self.require_still_for_bootstrap_handoff:
            return True
        if self.last_odom_time is None or self.last_linear_speed is None:
            self.get_logger().info(
                'Waiting for odometry before AMCL bootstrap handoff.',
                throttle_duration_sec=5.0,
            )
            return False
        if now - self.last_odom_time > self.odom_timeout_sec:
            self.get_logger().info(
                'Waiting for fresh odometry before AMCL bootstrap handoff.',
                throttle_duration_sec=5.0,
            )
            return False
        if (
            self.last_linear_speed > self.bootstrap_linear_threshold
            or self.last_angular_speed > self.bootstrap_angular_threshold
        ):
            self.get_logger().info(
                'Waiting for robot to stop before AMCL bootstrap handoff: '
                f'v={self.last_linear_speed:.3f}/{self.bootstrap_linear_threshold:.3f}m/s, '
                f'w={self.last_angular_speed:.3f}/{self.bootstrap_angular_threshold:.3f}rad/s',
                throttle_duration_sec=5.0,
            )
            return False
        return self._sensor_still_ok_for_bootstrap()

    def _sensor_still_ok_for_bootstrap(self) -> bool:
        if not self.require_sensor_still_for_bootstrap:
            return True
        if not self.sensor_still_ready:
            self.get_logger().info(
                'Waiting for LiDAR stillness window before AMCL bootstrap handoff.',
                throttle_duration_sec=5.0,
            )
            return False
        if not self.sensor_still:
            self.get_logger().info(
                'Waiting for LiDAR frame to stop before AMCL bootstrap handoff: '
                f'd={self._format_optional(self.sensor_translation_span)}m/'
                f'{self.sensor_still_translation_threshold:.3f}m, '
                f'yaw={self._format_optional(self.sensor_yaw_span)}rad/'
                f'{self.sensor_still_yaw_threshold:.3f}rad',
                throttle_duration_sec=5.0,
            )
            return False
        return True

    def _clear_particle_cloud(self):
        if not self.clear_particle_cloud_after_bootstrap or self.particle_cloud_cleared:
            return
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        for _ in range(max(self.particle_clear_publish_count, 1)):
            self.particle_cloud_pub.publish(msg)
        self.particle_cloud_cleared = True
        self.get_logger().info(
            f'Published empty {self.particle_cloud_topic} to clear stale AMCL particles in RViz.'
        )

    def _maybe_trigger_low_speed_relocalization(self, now: float):
        if not self.trigger_on_low_speed:
            return
        if now - self.node_start_time < self.low_speed_start_delay_sec:
            return
        if self.last_odom_time is None or self.last_linear_speed is None:
            return
        if now - self.last_odom_time > self.odom_timeout_sec:
            self.low_speed_since = None
            return

        is_low_speed = (
            self.last_linear_speed <= self.low_speed_linear_threshold
            and self.last_angular_speed <= self.low_speed_angular_threshold
        )
        if not is_low_speed:
            self.low_speed_since = None
            return
        if not self._sensor_still_ok_for_low_speed():
            self.low_speed_since = None
            return
        if self.low_speed_since is None:
            self.low_speed_since = now
            return
        if now - self.low_speed_since < self.low_speed_hold_sec:
            return
        if now - self.last_low_speed_trigger_time < self.low_speed_cooldown_sec:
            return

        reason = self._low_speed_relocalization_reason(now)
        if reason is None:
            return
        if self._trigger_relocalization(reason):
            self.last_low_speed_trigger_time = now
            self.get_logger().info(
                'Low-speed relocalization gate triggered: '
                f'reason={reason}, v={self.last_linear_speed:.3f}m/s, '
                f'w={self.last_angular_speed:.3f}rad/s, '
                f'sensor_d={self._format_optional(self.sensor_translation_span)}m, '
                f'sensor_yaw={self._format_optional(self.sensor_yaw_span)}rad, '
                f'score={self.last_amcl_score}'
            )

    def _update_sensor_stillness(self, now: float):
        if not self.trigger_on_low_speed or not self.require_sensor_still_for_low_speed:
            self.sensor_still_ready = True
            self.sensor_still = True
            return

        try:
            msg = self.tf_buffer.lookup_transform(
                self.sensor_still_reference_frame, self.sensor_frame, Time()
            )
        except Exception as exc:  # noqa: BLE001
            self.sensor_tf_samples.clear()
            self.sensor_still_ready = False
            self.sensor_still = False
            self.sensor_translation_span = None
            self.sensor_yaw_span = None
            self.get_logger().info(
                'Waiting for sensor stillness TF '
                f'{self.sensor_still_reference_frame}->{self.sensor_frame}: {exc}',
                throttle_duration_sec=5.0,
            )
            return

        self.sensor_tf_samples.append((now, tf_to_transform(msg)))
        while (
            self.sensor_tf_samples
            and now - self.sensor_tf_samples[0][0] > self.sensor_still_window_sec
        ):
            self.sensor_tf_samples.popleft()

        window_age = now - self.sensor_tf_samples[0][0] if self.sensor_tf_samples else 0.0
        if (
            len(self.sensor_tf_samples) < self.sensor_still_min_samples
            or window_age < self.sensor_still_window_sec * 0.8
        ):
            self.sensor_still_ready = False
            self.sensor_still = False
            return

        # 低速重定位以雷达本体是否近似静止为准，避免底盘速度为 0 但云台仍在转时误触发。
        reference_tf = self.sensor_tf_samples[0][1]
        max_translation = 0.0
        max_yaw = 0.0
        for _, sample_tf in self.sensor_tf_samples:
            translation, yaw = transform_delta(reference_tf, sample_tf)
            max_translation = max(max_translation, translation)
            max_yaw = max(max_yaw, yaw)

        self.sensor_translation_span = max_translation
        self.sensor_yaw_span = max_yaw
        self.sensor_still_ready = True
        self.sensor_still = (
            max_translation <= self.sensor_still_translation_threshold
            and max_yaw <= self.sensor_still_yaw_threshold
        )

    def _sensor_still_ok_for_low_speed(self) -> bool:
        if not self.require_sensor_still_for_low_speed:
            return True
        if not self.sensor_still_ready:
            self.get_logger().info(
                'Low-speed relocalization waiting: sensor stillness window is not ready',
                throttle_duration_sec=5.0,
            )
            return False
        if not self.sensor_still:
            self.get_logger().info(
                'Skip low-speed relocalization: LiDAR frame is still moving '
                f'd={self._format_optional(self.sensor_translation_span)}m/'
                f'{self.sensor_still_translation_threshold:.3f}m, '
                f'yaw={self._format_optional(self.sensor_yaw_span)}rad/'
                f'{self.sensor_still_yaw_threshold:.3f}rad',
                throttle_duration_sec=5.0,
            )
            return False
        return True

    def _format_optional(self, value) -> str:
        if value is None:
            return 'n/a'
        return f'{value:.3f}'

    def _low_speed_relocalization_reason(self, now: float):
        # 低速只是允许条件；必须同时满足定位质量不足，避免停住时反复无意义配准。
        if self.fault_active:
            return 'low_speed_localization_fault'
        if not self.use_amcl_quality_for_low_speed:
            return None
        if self.amcl_converged_seen and not self.amcl_converged:
            return 'low_speed_amcl_not_converged'
        if self.last_amcl_score is None or self.last_amcl_score_time is None:
            return None
        if now - self.last_amcl_score_time > self.odom_timeout_sec * 3.0:
            return None
        if self.last_amcl_score < self.low_speed_score_threshold:
            return 'low_speed_amcl_score_low'
        return None

    def _trigger_relocalization(self, reason: str):
        now = time.monotonic()
        if now - self.last_trigger_time < 1.0:
            return False
        if not self._known_area_ok_for_relocalization(reason):
            return False
        if not self.trigger_client.service_is_ready():
            self.get_logger().info(
                f'Waiting for relocalization service {self.trigger_service}',
                throttle_duration_sec=5.0,
            )
            return False
        self.last_trigger_time = now
        future = self.trigger_client.call_async(Trigger.Request())
        future.add_done_callback(lambda done: self._trigger_done(done, reason))
        return True

    def _known_area_ok_for_relocalization(self, reason: str) -> bool:
        if not self.require_known_area_for_relocalization:
            return True
        if self.map_msg is None:
            self.get_logger().info(
                f'Skip relocalization trigger ({reason}): occupancy map is not ready',
                throttle_duration_sec=5.0,
            )
            return False

        map_base = self._lookup_transform(self.map_frame, self.base_frame, log_wait=False)
        if map_base is None:
            self.get_logger().info(
                f'Skip relocalization trigger ({reason}): map->{self.base_frame} TF is unavailable',
                throttle_duration_sec=5.0,
            )
            return False

        ok, message = self._is_pose_inside_known_map(map_base[0][3], map_base[1][3])
        if not ok:
            self.get_logger().warn(
                f'Skip relocalization trigger ({reason}): {message}',
                throttle_duration_sec=5.0,
            )
            return False
        return True

    def _is_pose_inside_known_map(self, x: float, y: float):
        info = self.map_msg.info
        resolution = float(info.resolution)
        if resolution <= 0.0 or info.width == 0 or info.height == 0:
            return False, 'occupancy map metadata is invalid'

        origin_tf = geometry_pose_to_transform(self.map_msg.info.origin)
        local = multiply(inverse(origin_tf), self._point_transform(x, y))
        grid_x = int(math.floor(local[0][3] / resolution))
        grid_y = int(math.floor(local[1][3] / resolution))
        if grid_x < 0 or grid_x >= info.width or grid_y < 0 or grid_y >= info.height:
            return False, f'robot is outside occupancy map bounds: x={x:.2f}, y={y:.2f}'

        radius_cells = max(1, int(math.ceil(self.known_area_check_radius_m / resolution)))
        known = 0
        total = 0
        out_of_bounds = 0
        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                if dx * dx + dy * dy > radius_cells * radius_cells:
                    continue
                cx = grid_x + dx
                cy = grid_y + dy
                total += 1
                if cx < 0 or cx >= info.width or cy < 0 or cy >= info.height:
                    out_of_bounds += 1
                    continue
                value = int(self.map_msg.data[cy * info.width + cx])
                if value >= 0:
                    known += 1

        known_fraction = known / max(total, 1)
        if known_fraction < self.known_area_min_known_fraction:
            return (
                False,
                'robot is near unknown/out-of-map area: '
                f'known_fraction={known_fraction:.2f}, '
                f'required={self.known_area_min_known_fraction:.2f}, '
                f'out_of_bounds_cells={out_of_bounds}',
            )
        return True, f'known_fraction={known_fraction:.2f}'

    def _point_transform(self, x: float, y: float):
        out = identity_transform()
        out[0][3] = x
        out[1][3] = y
        return out

    def _trigger_done(self, future, reason: str):
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'Relocalization trigger failed ({reason}): {exc}')
            return
        if response.success:
            self.get_logger().info(f'Relocalization triggered ({reason}): {response.message}')
        else:
            self.get_logger().warn(f'Relocalization trigger rejected ({reason}): {response.message}')

    def _pose_callback(self, msg: PoseStamped):
        if (
            self.output_mode == 'tf'
            and self.manage_map_to_odom_tf
            and not self.managed_tf_active
        ):
            if self.bootstrap_failed:
                self.get_logger().warn(
                    'Ignore relocalization result because AMCL bootstrap handoff failed; '
                    'AMCL remains the map->odom owner.'
                )
            else:
                self.get_logger().info(
                    'Ignore relocalization result until AMCL bootstrap has frozen map->odom.',
                    throttle_duration_sec=5.0,
                )
            return

        if not self._known_area_ok_for_relocalization('apply_result'):
            self.get_logger().warn('Reject relocalization result because robot is outside known map gate')
            return

        relocalized_map_odom = pose_to_transform(msg)
        if self.output_mode == 'tf' and self.managed_map_odom is not None:
            current_map_odom = self.managed_map_odom
        else:
            current_map_odom = self._lookup_transform(self.map_frame, self.odom_frame)
        if current_map_odom is None:
            self.get_logger().warn('Cannot compare relocalization result: map->odom TF is unavailable')
            return

        correction_m, correction_yaw = transform_delta(current_map_odom, relocalized_map_odom)
        if correction_m > self.max_correction_translation or correction_yaw > self.max_correction_yaw:
            self.get_logger().warn(
                'Reject relocalization correction outside gate: '
                f'd={correction_m:.3f} yaw={correction_yaw:.3f}'
            )
            return

        should_publish = False
        reason = ''
        if self.publish_on_fault and self.fault_active:
            should_publish = True
            reason = 'fault'
        elif self.publish_on_correction and (
            correction_m >= self.correction_translation_threshold
            or correction_yaw >= self.correction_yaw_threshold
        ):
            should_publish = True
            reason = 'correction'

        if not should_publish:
            self.get_logger().info(
                'Relocalization accepted but correction is below apply threshold: '
                f'd={correction_m:.3f} yaw={correction_yaw:.3f}',
                throttle_duration_sec=5.0,
            )
            return

        now = time.monotonic()
        if now - self.last_publish_time < self.min_publish_interval_sec:
            self.get_logger().info('Skip relocalization apply because cooldown is active')
            return

        if self.output_mode == 'tf':
            self.managed_map_odom = relocalized_map_odom
            self.managed_tf_active = True
            self._publish_managed_tf()
            self.last_publish_time = now
            self.get_logger().warn(
                'Updated frozen map->odom from relocalization '
                f'({reason}): correction={correction_m:.3f}m/{correction_yaw:.3f}rad'
            )
            return

        odom_base = self._lookup_transform(self.odom_frame, self.base_frame)
        if odom_base is None:
            self.get_logger().warn('Cannot publish AMCL reset: odom->base TF is unavailable')
            return

        map_base = multiply(relocalized_map_odom, odom_base)
        self._publish_initialpose(map_base, reason, correction_m, correction_yaw)
        self.last_publish_time = now

    def _lookup_transform(self, target: str, source: str, log_wait: bool = True) -> Optional[list]:
        try:
            msg = self.tf_buffer.lookup_transform(target, source, Time())
            return tf_to_transform(msg)
        except Exception as exc:  # noqa: BLE001
            if log_wait:
                self.get_logger().info(
                    f'Waiting for TF {target}->{source}: {exc}',
                    throttle_duration_sec=3.0,
                )
            return None

    def _publish_initialpose(self, map_base, reason: str, correction_m: float, correction_yaw: float):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.map_frame
        # 使用 0 时间戳，让 AMCL 使用最新可用 TF，减少仿真时间轻微滞后导致的外推问题。
        msg.header.stamp = Time().to_msg()
        msg.pose.pose.position.x = map_base[0][3]
        msg.pose.pose.position.y = map_base[1][3]
        msg.pose.pose.position.z = 0.0
        qx, qy, qz, qw = matrix_to_quaternion(map_base)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        xy_cov = self.xy_stddev * self.xy_stddev
        yaw_cov = self.yaw_stddev * self.yaw_stddev
        msg.pose.covariance[0] = xy_cov
        msg.pose.covariance[7] = xy_cov
        msg.pose.covariance[35] = yaw_cov
        self.initialpose_pub.publish(msg)
        self.get_logger().warn(
            'Published AMCL reset from relocalization '
            f'({reason}): x={msg.pose.pose.position.x:.3f}, '
            f'y={msg.pose.pose.position.y:.3f}, yaw={yaw_from_matrix(map_base):.3f}, '
            f'correction={correction_m:.3f}m/{correction_yaw:.3f}rad'
        )


def main(args=None):
    rclpy.init(args=args)
    node = RelocalizationAmclBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
