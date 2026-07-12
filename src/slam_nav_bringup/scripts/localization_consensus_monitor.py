#!/usr/bin/env python3

import json
import math
import time

import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import Bool, String

from slam_nav_bringup.localization_consensus import (
    Pose2D,
    compose_pose,
    evaluate_consensus,
)


def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def pose2d_from_pose(pose) -> Pose2D:
    return Pose2D(
        x=float(pose.position.x),
        y=float(pose.position.y),
        yaw=yaw_from_quaternion(pose.orientation),
    )


def pose2d_from_transform(transform) -> Pose2D:
    return Pose2D(
        x=float(transform.translation.x),
        y=float(transform.translation.y),
        yaw=yaw_from_quaternion(transform.rotation),
    )


class LocalizationConsensusMonitor(Node):
    """Compare localization candidates and publish advice without changing localization state."""

    def __init__(self):
        super().__init__('localization_consensus_monitor')

        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.odom_topic = self.declare_parameter('odom_topic', '/Odometry').value
        self.amcl_pose_topic = self.declare_parameter('amcl_pose_topic', '/amcl_pose').value
        self.amcl_status_topic = self.declare_parameter(
            'amcl_status_topic', '/amcl_convergence_status'
        ).value
        self.gicp_pose_topic = self.declare_parameter(
            'gicp_pose_topic', '/relocalization/pose'
        ).value
        self.gicp_quality_topic = self.declare_parameter(
            'gicp_quality_topic', '/relocalization/quality'
        ).value
        self.fault_topic = self.declare_parameter(
            'fault_topic', '/localization_fault'
        ).value
        self.output_topic = self.declare_parameter(
            'output_topic', '/localization/candidate_comparison'
        ).value

        self.check_rate_hz = float(self.declare_parameter('check_rate_hz', 2.0).value)
        self.odom_max_age_sec = float(self.declare_parameter('odom_max_age_sec', 1.0).value)
        self.amcl_max_age_sec = float(self.declare_parameter('amcl_max_age_sec', 3.0).value)
        self.gicp_max_age_sec = float(self.declare_parameter('gicp_max_age_sec', 45.0).value)
        self.quality_max_age_sec = float(
            self.declare_parameter('quality_max_age_sec', 3.0).value
        )
        self.agreement_translation = float(
            self.declare_parameter('agreement_translation', 0.25).value
        )
        self.agreement_yaw = float(self.declare_parameter('agreement_yaw', 0.15).value)
        self.correction_translation = float(
            self.declare_parameter('correction_translation', 0.50).value
        )
        self.correction_yaw = float(self.declare_parameter('correction_yaw', 0.25).value)
        self.max_auto_translation = float(
            self.declare_parameter('max_auto_translation', 2.0).value
        )
        self.max_auto_yaw = float(self.declare_parameter('max_auto_yaw', 0.8).value)

        self.last_odom = None
        self.last_odom_time = None
        self.last_amcl_pose = None
        self.last_amcl_pose_time = None
        self.last_amcl_status = None
        self.last_amcl_status_time = None
        self.last_gicp_pose = None
        self.last_gicp_pose_time = None
        self.last_gicp_quality = None
        self.last_gicp_quality_time = None
        self.fault_active = False
        self.last_decision = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.output_pub = self.create_publisher(String, self.output_topic, 10)

        self.create_subscription(
            Odometry, self.odom_topic, self._on_odom, qos_profile_sensor_data
        )
        self.create_subscription(
            PoseWithCovarianceStamped, self.amcl_pose_topic, self._on_amcl_pose, 10
        )
        self.create_subscription(String, self.amcl_status_topic, self._on_amcl_status, 10)
        self.create_subscription(PoseStamped, self.gicp_pose_topic, self._on_gicp_pose, 10)
        self.create_subscription(String, self.gicp_quality_topic, self._on_gicp_quality, 10)
        self.create_subscription(Bool, self.fault_topic, self._on_fault, 10)
        self.create_timer(1.0 / max(self.check_rate_hz, 0.2), self._on_timer)

        self.get_logger().info(
            'Read-only localization consensus monitor started: '
            f'output={self.output_topic}, agreement={self.agreement_translation:.2f}m/'
            f'{self.agreement_yaw:.2f}rad'
        )

    def _on_odom(self, msg):
        self.last_odom = msg
        self.last_odom_time = time.monotonic()

    def _on_amcl_pose(self, msg):
        self.last_amcl_pose = msg
        self.last_amcl_pose_time = time.monotonic()

    def _on_amcl_status(self, msg):
        parsed = self._parse_json_status(msg, 'AMCL')
        if parsed is not None:
            self.last_amcl_status = parsed
            self.last_amcl_status_time = time.monotonic()

    def _on_gicp_pose(self, msg):
        self.last_gicp_pose = msg
        self.last_gicp_pose_time = time.monotonic()

    def _on_gicp_quality(self, msg):
        parsed = self._parse_json_status(msg, 'GICP')
        if parsed is not None:
            self.last_gicp_quality = parsed
            self.last_gicp_quality_time = time.monotonic()

    def _on_fault(self, msg):
        self.fault_active = bool(msg.data)

    def _parse_json_status(self, msg, label):
        try:
            value = json.loads(msg.data)
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warn(
                f'Ignore invalid {label} quality JSON: {exc}',
                throttle_duration_sec=5.0,
            )
            return None
        if not isinstance(value, dict):
            return None
        return value

    def _on_timer(self):
        now = time.monotonic()
        odom_fresh = self._fresh(now, self.last_odom_time, self.odom_max_age_sec)
        amcl_pose_fresh = self._fresh(now, self.last_amcl_pose_time, self.amcl_max_age_sec)
        amcl_status_fresh = self._fresh(
            now, self.last_amcl_status_time, self.quality_max_age_sec
        )
        gicp_pose_fresh = self._fresh(now, self.last_gicp_pose_time, self.gicp_max_age_sec)
        gicp_quality_fresh = self._fresh(
            now, self.last_gicp_quality_time, self.gicp_max_age_sec
        )

        current_map_odom = self._lookup_map_odom()
        odom_base = (
            pose2d_from_pose(self.last_odom.pose.pose)
            if odom_fresh and self.last_odom is not None
            else None
        )
        fast_lio = (
            compose_pose(current_map_odom, odom_base)
            if current_map_odom is not None and odom_base is not None
            else None
        )
        amcl = (
            pose2d_from_pose(self.last_amcl_pose.pose.pose)
            if amcl_pose_fresh and self.last_amcl_pose is not None
            else None
        )
        gicp_map_odom = (
            pose2d_from_pose(self.last_gicp_pose.pose)
            if gicp_pose_fresh and self.last_gicp_pose is not None
            else None
        )
        gicp = (
            compose_pose(gicp_map_odom, odom_base)
            if gicp_map_odom is not None and odom_base is not None
            else None
        )

        amcl_healthy = bool(
            amcl_status_fresh
            and self.last_amcl_status is not None
            and self.last_amcl_status.get('converged')
        )
        gicp_quality_matches = self._gicp_quality_matches_pose()
        gicp_healthy = bool(
            gicp_quality_fresh
            and gicp_quality_matches
            and self.last_gicp_quality is not None
            and self.last_gicp_quality.get('accepted')
        )
        fast_lio_healthy = bool(odom_fresh and current_map_odom is not None and not self.fault_active)

        decision = evaluate_consensus(
            fast_lio,
            amcl,
            gicp,
            fast_lio_healthy=fast_lio_healthy,
            amcl_healthy=amcl_healthy,
            gicp_healthy=gicp_healthy,
            agreement_translation=self.agreement_translation,
            agreement_yaw=self.agreement_yaw,
            correction_translation=self.correction_translation,
            correction_yaw=self.correction_yaw,
            max_auto_translation=self.max_auto_translation,
            max_auto_yaw=self.max_auto_yaw,
        )

        payload = {
            'observation_only': True,
            'automatic_action_allowed': False,
            'decision': decision.decision,
            'reason': decision.reason,
            'reference': decision.reference,
            'candidates': {
                'fast_lio': self._candidate_payload(
                    fast_lio, fast_lio_healthy, self._age(now, self.last_odom_time)
                ),
                'amcl': self._candidate_payload(
                    amcl, amcl_healthy, self._age(now, self.last_amcl_pose_time)
                ),
                'gicp': self._candidate_payload(
                    gicp, gicp_healthy, self._age(now, self.last_gicp_pose_time)
                ),
            },
            'pairwise': {
                key: {
                    'translation_m': round(value.translation, 4),
                    'yaw_rad': round(value.yaw, 4),
                }
                for key, value in decision.pairwise.items()
            },
            'quality': {
                'localization_fault': self.fault_active,
                'amcl': self.last_amcl_status,
                'gicp': self.last_gicp_quality,
                'gicp_quality_matches_pose': gicp_quality_matches,
            },
            'thresholds': {
                'agreement_translation_m': self.agreement_translation,
                'agreement_yaw_rad': self.agreement_yaw,
                'correction_translation_m': self.correction_translation,
                'correction_yaw_rad': self.correction_yaw,
                'max_auto_translation_m': self.max_auto_translation,
                'max_auto_yaw_rad': self.max_auto_yaw,
            },
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False, separators=(',', ':'))
        self.output_pub.publish(msg)

        if decision.decision != self.last_decision:
            logger = self.get_logger().info
            if decision.decision not in ('NORMAL', 'NORMAL_DEGRADED', 'INSUFFICIENT_DATA'):
                logger = self.get_logger().warn
            logger(f'Localization consensus={decision.decision}: {decision.reason}')
            self.last_decision = decision.decision

    def _lookup_map_odom(self):
        try:
            msg = self.tf_buffer.lookup_transform(self.map_frame, self.odom_frame, Time())
        except Exception as exc:  # noqa: BLE001
            self.get_logger().info(
                f'Waiting for TF {self.map_frame}->{self.odom_frame}: {exc}',
                throttle_duration_sec=5.0,
            )
            return None
        return pose2d_from_transform(msg.transform)

    def _gicp_quality_matches_pose(self) -> bool:
        if self.last_gicp_pose is None or self.last_gicp_quality is None:
            return False
        try:
            quality_stamp = (
                int(self.last_gicp_quality['stamp_sec']),
                int(self.last_gicp_quality['stamp_nanosec']),
            )
        except (KeyError, TypeError, ValueError):
            return False
        pose_stamp = (
            int(self.last_gicp_pose.header.stamp.sec),
            int(self.last_gicp_pose.header.stamp.nanosec),
        )
        return quality_stamp == pose_stamp

    @staticmethod
    def _fresh(now, timestamp, max_age):
        return timestamp is not None and now - timestamp <= max_age

    @staticmethod
    def _age(now, timestamp):
        return None if timestamp is None else round(now - timestamp, 3)

    @staticmethod
    def _candidate_payload(pose, healthy, age):
        if pose is None:
            return {'available': False, 'healthy': False, 'age_sec': age}
        return {
            'available': True,
            'healthy': healthy,
            'age_sec': age,
            'x': round(pose.x, 4),
            'y': round(pose.y, 4),
            'yaw': round(pose.yaw, 4),
        }


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationConsensusMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
