#!/usr/bin/env python3

import json
import math
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Bool, String


class LocalizationGuard(Node):
    """定位健康监控节点：检测里程计、点云和扫描断流，以及明显跳变。"""

    def __init__(self):
        super().__init__('localization_guard_node')

        self.odom_topic = self.declare_parameter('odom_topic', '/Odometry').value
        self.cloud_topic = self.declare_parameter('cloud_topic', '/cloud_registered').value
        self.scan_topic = self.declare_parameter('scan_topic', '/scan').value
        self.health_topic = self.declare_parameter('health_topic', '/localization_health').value
        self.fault_topic = self.declare_parameter('fault_topic', '/localization_fault').value
        self.diagnostic_topic = self.declare_parameter('diagnostic_topic', '/diagnostics').value
        self.cmd_vel_topic = self.declare_parameter('cmd_vel_topic', '/cmd_vel').value

        self.check_rate_hz = float(self.declare_parameter('check_rate_hz', 5.0).value)
        self.odom_timeout_sec = float(self.declare_parameter('odom_timeout_sec', 1.0).value)
        self.cloud_timeout_sec = float(self.declare_parameter('cloud_timeout_sec', 1.0).value)
        self.scan_timeout_sec = float(self.declare_parameter('scan_timeout_sec', 1.5).value)

        self.max_linear_speed = float(self.declare_parameter('max_linear_speed', 2.0).value)
        self.max_angular_speed = float(self.declare_parameter('max_angular_speed', 3.5).value)
        self.max_pose_jump_m = float(self.declare_parameter('max_pose_jump_m', 1.0).value)
        self.max_yaw_jump_rad = float(self.declare_parameter('max_yaw_jump_rad', 1.2).value)
        self.fault_hold_sec = float(self.declare_parameter('fault_hold_sec', 2.0).value)

        self.publish_zero_on_fault = bool(
            self.declare_parameter('publish_zero_on_fault', False).value
        )
        self.zero_publish_count = int(self.declare_parameter('zero_publish_count', 5).value)

        self.last_odom_time = None
        self.last_cloud_time = None
        self.last_scan_time = None
        self.last_pose = None
        self.last_yaw = None
        self.fault_hold_until = 0.0
        self.latched_reasons = []

        self.health_pub = self.create_publisher(String, self.health_topic, 10)
        self.fault_pub = self.create_publisher(Bool, self.fault_topic, 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, self.diagnostic_topic, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.create_subscription(Odometry, self.odom_topic, self._odom_callback, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, self.cloud_topic, self._cloud_callback, qos_profile_sensor_data)
        self.create_subscription(LaserScan, self.scan_topic, self._scan_callback, qos_profile_sensor_data)

        timer_period = 1.0 / max(self.check_rate_hz, 0.5)
        self.create_timer(timer_period, self._on_timer)

        self.get_logger().info(
            'Localization guard started: '
            f'odom={self.odom_topic}, cloud={self.cloud_topic}, scan={self.scan_topic}, '
            f'zero_on_fault={self.publish_zero_on_fault}'
        )

    def _odom_callback(self, msg):
        now = time.monotonic()
        self.last_odom_time = now

        linear = msg.twist.twist.linear
        angular = msg.twist.twist.angular
        linear_speed = math.sqrt(linear.x * linear.x + linear.y * linear.y + linear.z * linear.z)
        angular_speed = abs(angular.z)

        reasons = []
        if linear_speed > self.max_linear_speed:
            reasons.append(f'linear_speed_high:{linear_speed:.2f}')
        if angular_speed > self.max_angular_speed:
            reasons.append(f'angular_speed_high:{angular_speed:.2f}')

        pose = msg.pose.pose.position
        yaw = self._yaw_from_quaternion(msg.pose.pose.orientation)
        if self.last_pose is not None and self.last_yaw is not None:
            jump = math.hypot(pose.x - self.last_pose[0], pose.y - self.last_pose[1])
            yaw_jump = abs(self._normalize_angle(yaw - self.last_yaw))
            if jump > self.max_pose_jump_m:
                reasons.append(f'pose_jump:{jump:.2f}')
            if yaw_jump > self.max_yaw_jump_rad:
                reasons.append(f'yaw_jump:{yaw_jump:.2f}')

        self.last_pose = (pose.x, pose.y)
        self.last_yaw = yaw

        if reasons:
            self._latch_fault(reasons)

    def _cloud_callback(self, _msg):
        self.last_cloud_time = time.monotonic()

    def _scan_callback(self, _msg):
        self.last_scan_time = time.monotonic()

    def _on_timer(self):
        now = time.monotonic()
        reasons = []
        reasons.extend(self._timeout_reason(now, 'odom_timeout', self.last_odom_time, self.odom_timeout_sec))
        reasons.extend(self._timeout_reason(now, 'cloud_timeout', self.last_cloud_time, self.cloud_timeout_sec))
        reasons.extend(self._timeout_reason(now, 'scan_timeout', self.last_scan_time, self.scan_timeout_sec))

        if reasons:
            self._latch_fault(reasons)

        fault_active = now < self.fault_hold_until
        active_reasons = self.latched_reasons if fault_active else []
        if not fault_active:
            self.latched_reasons = []

        self._publish_status(fault_active, active_reasons, now)
        if fault_active and self.publish_zero_on_fault:
            self._publish_zero_velocity()

    def _timeout_reason(self, now, name, last_time, timeout_sec):
        if last_time is None:
            return [f'{name}:never_seen']
        age = now - last_time
        if age > timeout_sec:
            return [f'{name}:{age:.2f}s']
        return []

    def _latch_fault(self, reasons):
        now = time.monotonic()
        self.fault_hold_until = max(self.fault_hold_until, now + self.fault_hold_sec)
        for reason in reasons:
            if reason not in self.latched_reasons:
                self.latched_reasons.append(reason)
        self.get_logger().warn(
            'Localization guard fault: ' + ', '.join(reasons),
            throttle_duration_sec=2.0,
        )

    def _publish_status(self, fault_active, reasons, now):
        payload = {
            'ok': not fault_active,
            'level': 'ERROR' if fault_active else 'OK',
            'reasons': reasons,
            'topic_age_sec': {
                'odom': self._age(now, self.last_odom_time),
                'cloud': self._age(now, self.last_cloud_time),
                'scan': self._age(now, self.last_scan_time),
            },
        }

        health_msg = String()
        health_msg.data = json.dumps(payload, ensure_ascii=False)
        self.health_pub.publish(health_msg)

        fault_msg = Bool()
        fault_msg.data = fault_active
        self.fault_pub.publish(fault_msg)

        diag = DiagnosticArray()
        diag.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = 'localization_guard'
        status.hardware_id = 'slam_nav_ws'
        status.level = DiagnosticStatus.ERROR if fault_active else DiagnosticStatus.OK
        status.message = ', '.join(reasons) if fault_active else 'localization inputs healthy'
        status.values = [
            KeyValue(key='odom_age_sec', value=str(payload['topic_age_sec']['odom'])),
            KeyValue(key='cloud_age_sec', value=str(payload['topic_age_sec']['cloud'])),
            KeyValue(key='scan_age_sec', value=str(payload['topic_age_sec']['scan'])),
            KeyValue(key='publish_zero_on_fault', value=str(self.publish_zero_on_fault)),
        ]
        diag.status.append(status)
        self.diag_pub.publish(diag)

    def _publish_zero_velocity(self):
        zero = Twist()
        for _ in range(max(self.zero_publish_count, 1)):
            self.cmd_vel_pub.publish(zero)

    @staticmethod
    def _age(now, last_time):
        if last_time is None:
            return None
        return round(now - last_time, 3)

    @staticmethod
    def _yaw_from_quaternion(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _normalize_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationGuard()
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
