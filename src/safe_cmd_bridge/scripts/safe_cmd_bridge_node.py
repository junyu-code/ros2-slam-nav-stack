#!/usr/bin/env python3

import json
import math
import socket
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, String


class SafeCmdBridge(Node):
    def __init__(self):
        super().__init__('safe_cmd_bridge_node')

        self.input_topic = self.declare_parameter('input_topic', '/cmd_vel').value
        self.output_topic = self.declare_parameter('output_topic', '/cmd_vel_safe').value
        self.enable_topic_output = self.declare_parameter('enable_topic_output', True).value
        self.enable_udp_output = self.declare_parameter('enable_udp_output', False).value
        self.enable_fault_stop = self.declare_parameter('enable_fault_stop', True).value
        self.fault_topic = self.declare_parameter('fault_topic', '/localization_fault').value
        self.enable_feedback_watchdog = self.declare_parameter(
            'enable_feedback_watchdog', False
        ).value
        self.feedback_topic = self.declare_parameter('feedback_topic', '/base/odom').value
        self.feedback_fault_topic = self.declare_parameter(
            'feedback_fault_topic', '/base_feedback_fault'
        ).value
        self.feedback_health_topic = self.declare_parameter(
            'feedback_health_topic', '/base_feedback_health'
        ).value
        self.udp_host = self.declare_parameter('udp_host', '192.168.123.22').value
        self.udp_port = int(self.declare_parameter('udp_port', 15000).value)

        self.max_vx = float(self.declare_parameter('max_vx', 0.5).value)
        self.min_vx = float(self.declare_parameter('min_vx', -0.2).value)
        self.max_vy = float(self.declare_parameter('max_vy', 0.3).value)
        self.min_vy = float(self.declare_parameter('min_vy', -0.3).value)
        self.max_wz = float(self.declare_parameter('max_wz', 0.8).value)
        self.min_wz = float(self.declare_parameter('min_wz', -0.8).value)

        self.max_linear_accel = float(self.declare_parameter('max_linear_accel', 0.8).value)
        self.max_linear_decel = float(self.declare_parameter('max_linear_decel', 1.0).value)
        self.max_angular_accel = float(self.declare_parameter('max_angular_accel', 1.0).value)
        self.max_angular_decel = float(self.declare_parameter('max_angular_decel', 1.2).value)

        self.deadband_v = float(self.declare_parameter('deadband_v', 0.01).value)
        self.deadband_w = float(self.declare_parameter('deadband_w', 0.02).value)
        self.command_timeout_sec = float(self.declare_parameter('command_timeout_sec', 0.5).value)
        self.publish_rate_hz = float(self.declare_parameter('publish_rate_hz', 30.0).value)
        self.send_zero_on_shutdown = self.declare_parameter('send_zero_on_shutdown', True).value
        self.feedback_timeout_sec = float(
            self.declare_parameter('feedback_timeout_sec', 0.8).value
        )
        self.feedback_startup_grace_sec = float(
            self.declare_parameter('feedback_startup_grace_sec', 3.0).value
        )
        self.feedback_stall_timeout_sec = float(
            self.declare_parameter('feedback_stall_timeout_sec', 1.0).value
        )
        self.min_command_speed_for_feedback = float(
            self.declare_parameter('min_command_speed_for_feedback', 0.05).value
        )
        self.min_feedback_speed = float(
            self.declare_parameter('min_feedback_speed', 0.02).value
        )

        self.invert_vx = self.declare_parameter('invert_vx', False).value
        self.invert_vy = self.declare_parameter('invert_vy', False).value
        self.invert_wz = self.declare_parameter('invert_wz', False).value

        self._target_vx = 0.0
        self._target_vy = 0.0
        self._target_wz = 0.0
        self._current_vx = 0.0
        self._current_vy = 0.0
        self._current_wz = 0.0
        self._last_cmd_time = None
        self._last_update_time = time.monotonic()
        self._last_timeout_warn = 0.0
        self._fault_active = False
        self._last_fault_warn = 0.0
        self._start_time = time.monotonic()
        self._last_feedback_time = None
        self._feedback_speed = 0.0
        self._last_nonzero_cmd_time = None
        self._feedback_fault_active = False
        self._feedback_fault_reasons = []
        self._last_feedback_warn = 0.0

        self._udp_socket = None
        if self.enable_udp_output:
            self._udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self._publisher = None
        if self.enable_topic_output:
            self._publisher = self.create_publisher(Twist, self.output_topic, 10)

        self._subscription = self.create_subscription(
            Twist,
            self.input_topic,
            self._on_cmd_vel,
            10,
        )
        self._fault_subscription = None
        if self.enable_fault_stop:
            self._fault_subscription = self.create_subscription(
                Bool,
                self.fault_topic,
                self._on_fault,
                10,
            )

        self._feedback_fault_pub = None
        self._feedback_health_pub = None
        self._feedback_subscription = None
        if self.enable_feedback_watchdog:
            self._feedback_fault_pub = self.create_publisher(
                Bool,
                self.feedback_fault_topic,
                10,
            )
            self._feedback_health_pub = self.create_publisher(
                String,
                self.feedback_health_topic,
                10,
            )
            self._feedback_subscription = self.create_subscription(
                Odometry,
                self.feedback_topic,
                self._on_feedback,
                10,
            )

        timer_period = 1.0 / max(self.publish_rate_hz, 1.0)
        self._timer = self.create_timer(timer_period, self._on_timer)

        self.get_logger().info(
            'Safe cmd bridge started: '
            f'{self.input_topic} -> {self.output_topic}, '
            f'topic_output={self.enable_topic_output}, '
            f'udp_output={self.enable_udp_output} udp={self.udp_host}:{self.udp_port}, '
            f'fault_stop={self.enable_fault_stop} fault_topic={self.fault_topic}, '
            f'feedback_watchdog={self.enable_feedback_watchdog} feedback={self.feedback_topic}'
        )

    def _on_cmd_vel(self, msg):
        vx = -msg.linear.x if self.invert_vx else msg.linear.x
        vy = -msg.linear.y if self.invert_vy else msg.linear.y
        wz = -msg.angular.z if self.invert_wz else msg.angular.z

        self._target_vx = self._sanitize(vx, self.min_vx, self.max_vx, self.deadband_v)
        self._target_vy = self._sanitize(vy, self.min_vy, self.max_vy, self.deadband_v)
        self._target_wz = self._sanitize(wz, self.min_wz, self.max_wz, self.deadband_w)
        self._last_cmd_time = time.monotonic()

    def _on_fault(self, msg):
        self._fault_active = bool(msg.data)
        if self._fault_active:
            self._target_vx = 0.0
            self._target_vy = 0.0
            self._target_wz = 0.0

    def _on_feedback(self, msg):
        linear = msg.twist.twist.linear
        self._feedback_speed = self._linear_command_norm(linear.x, linear.y)
        self._last_feedback_time = time.monotonic()

    def _on_timer(self):
        now = time.monotonic()
        dt = max(now - self._last_update_time, 0.0)
        self._last_update_time = now

        target_vx, target_vy, target_wz = self._get_active_target(now)
        self._update_feedback_watchdog(now, target_vx, target_vy)

        self._current_vx = self._limit_step(
            self._current_vx,
            target_vx,
            self.max_linear_accel,
            self.max_linear_decel,
            dt,
        )
        self._current_vy = self._limit_step(
            self._current_vy,
            target_vy,
            self.max_linear_accel,
            self.max_linear_decel,
            dt,
        )
        self._current_wz = self._limit_step(
            self._current_wz,
            target_wz,
            self.max_angular_accel,
            self.max_angular_decel,
            dt,
        )

        safe_msg = Twist()
        safe_msg.linear.x = self._current_vx
        safe_msg.linear.y = self._current_vy
        safe_msg.angular.z = self._current_wz
        self._emit(safe_msg)

    def _get_active_target(self, now):
        if self._fault_active:
            if now - self._last_fault_warn > 2.0:
                self.get_logger().warn('localization fault active, forcing safe velocity to zero')
                self._last_fault_warn = now
            return 0.0, 0.0, 0.0

        if self._feedback_fault_active:
            if now - self._last_feedback_warn > 2.0:
                self.get_logger().warn(
                    'base feedback fault active, forcing safe velocity to zero: '
                    + ', '.join(self._feedback_fault_reasons)
                )
                self._last_feedback_warn = now
            return 0.0, 0.0, 0.0

        if self._last_cmd_time is None:
            return 0.0, 0.0, 0.0

        age = now - self._last_cmd_time
        if age <= self.command_timeout_sec:
            return self._target_vx, self._target_vy, self._target_wz

        if now - self._last_timeout_warn > 2.0:
            self.get_logger().warn(
                f'cmd_vel timeout: no command for {age:.2f}s, ramping to zero'
            )
            self._last_timeout_warn = now
        return 0.0, 0.0, 0.0

    def _update_feedback_watchdog(self, now, target_vx, target_vy):
        if not self.enable_feedback_watchdog:
            return

        reasons = []
        command_speed = self._linear_command_norm(target_vx, target_vy)
        in_startup_grace = now - self._start_time < self.feedback_startup_grace_sec

        if not in_startup_grace and self._last_feedback_time is None:
            reasons.append('feedback_never_seen')

        if self._last_feedback_time is not None:
            feedback_age = now - self._last_feedback_time
            if feedback_age > self.feedback_timeout_sec:
                reasons.append(f'feedback_timeout:{feedback_age:.2f}s')

        if command_speed >= self.min_command_speed_for_feedback:
            if self._last_nonzero_cmd_time is None:
                self._last_nonzero_cmd_time = now
            command_age = now - self._last_nonzero_cmd_time
            if (
                command_age >= self.feedback_stall_timeout_sec
                and self._feedback_speed < self.min_feedback_speed
            ):
                reasons.append(
                    f'feedback_stall:cmd={command_speed:.2f},fb={self._feedback_speed:.2f}'
                )
        else:
            self._last_nonzero_cmd_time = None

        self._feedback_fault_active = bool(reasons)
        self._feedback_fault_reasons = reasons
        self._publish_feedback_status(now, command_speed)

    def _publish_feedback_status(self, now, command_speed):
        if self._feedback_fault_pub is None or self._feedback_health_pub is None:
            return

        fault_msg = Bool()
        fault_msg.data = self._feedback_fault_active
        self._feedback_fault_pub.publish(fault_msg)

        payload = {
            'ok': not self._feedback_fault_active,
            'reasons': self._feedback_fault_reasons,
            'command_speed': round(command_speed, 3),
            'feedback_speed': round(self._feedback_speed, 3),
            'feedback_age_sec': None
            if self._last_feedback_time is None
            else round(now - self._last_feedback_time, 3),
        }
        health_msg = String()
        health_msg.data = json.dumps(payload, ensure_ascii=False)
        self._feedback_health_pub.publish(health_msg)

    def _emit(self, msg):
        if self._publisher is not None:
            self._publisher.publish(msg)

        if self._udp_socket is not None:
            payload = f'{msg.linear.x:.4f},{msg.linear.y:.4f},{msg.angular.z:.4f}\n'
            try:
                self._udp_socket.sendto(payload.encode('ascii'), (self.udp_host, self.udp_port))
            except OSError as exc:
                self.get_logger().warn(f'UDP velocity send failed: {exc}')

    @staticmethod
    def _linear_command_norm(vx, vy):
        return math.sqrt(vx * vx + vy * vy)

    def publish_zero(self):
        zero = Twist()
        self._current_vx = 0.0
        self._current_vy = 0.0
        self._current_wz = 0.0
        for _ in range(5):
            self._emit(zero)
            time.sleep(0.02)

    @staticmethod
    def _sanitize(value, lower, upper, deadband):
        if not math.isfinite(value):
            return 0.0
        value = min(max(value, lower), upper)
        if abs(value) < deadband:
            return 0.0
        return value

    @staticmethod
    def _limit_step(current, target, accel_limit, decel_limit, dt):
        if dt <= 0.0:
            return current

        delta = target - current
        if abs(delta) < 1e-6:
            return target

        # 速度绝对值下降或反向穿零时，使用更保守的减速度限制。
        same_direction = (current == 0.0) or (target == 0.0) or (current * target > 0.0)
        slowing_down = abs(target) < abs(current)
        limit = decel_limit if slowing_down or not same_direction else accel_limit
        max_step = max(limit, 0.0) * dt

        if abs(delta) <= max_step:
            return target
        return current + math.copysign(max_step, delta)


def main(args=None):
    rclpy.init(args=args)
    node = SafeCmdBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.send_zero_on_shutdown:
            node.publish_zero()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
