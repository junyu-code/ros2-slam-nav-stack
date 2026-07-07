#!/usr/bin/env python3

import math
import time

import rclpy
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped
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
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class InitialPosePublisher(Node):
    """等待地图、扫描和里程计就绪后，为 AMCL 发布初始位姿。"""

    def __init__(self):
        super().__init__('initial_pose_publisher')

        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('base_frame_id', 'base_footprint')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/Odometry')
        self.declare_parameter('initialpose_topic', '/initialpose')
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('xy_stddev', 0.25)
        self.declare_parameter('yaw_stddev', 0.35)
        self.declare_parameter('publish_count', 1)
        self.declare_parameter('publish_period', 0.5)
        self.declare_parameter('settle_time', 2.0)
        self.declare_parameter('require_map_to_base_tf', True)
        self.declare_parameter('post_publish_timeout', 12.0)
        self.declare_parameter('require_amcl_convergence', False)
        self.declare_parameter('amcl_converged_topic', '/amcl_converged')
        self.declare_parameter('amcl_convergence_timeout', 30.0)
        self.declare_parameter('continue_on_amcl_convergence_timeout', True)

        self.frame_id = self.get_parameter('frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.x = float(self.get_parameter('x').value)
        self.y = float(self.get_parameter('y').value)
        self.yaw = float(self.get_parameter('yaw').value)
        self.xy_stddev = float(self.get_parameter('xy_stddev').value)
        self.yaw_stddev = float(self.get_parameter('yaw_stddev').value)
        self.publish_count = int(self.get_parameter('publish_count').value)
        self.settle_time = float(self.get_parameter('settle_time').value)
        self.require_map_to_base_tf = bool(self.get_parameter('require_map_to_base_tf').value)
        self.post_publish_timeout = float(self.get_parameter('post_publish_timeout').value)
        self.require_amcl_convergence = bool(self.get_parameter('require_amcl_convergence').value)
        self.amcl_convergence_timeout = float(self.get_parameter('amcl_convergence_timeout').value)
        self.continue_on_amcl_convergence_timeout = bool(
            self.get_parameter('continue_on_amcl_convergence_timeout').value
        )

        self.map_ready = False
        self.scan_ready = False
        self.odom_ready = False
        self.ready_since = None
        self.sent_count = 0
        self.waiting_for_map_tf = False
        self.waiting_for_convergence = False
        self.map_tf_wait_started = None
        self.convergence_wait_started = None
        self.amcl_converged = False
        self.finished = False
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        qos_depth = 10
        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            OccupancyGrid,
            self.get_parameter('map_topic').value,
            self._on_map,
            map_qos,
        )
        self.create_subscription(
            LaserScan,
            self.get_parameter('scan_topic').value,
            self._on_scan,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            self.get_parameter('odom_topic').value,
            self._on_odom,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Bool,
            self.get_parameter('amcl_converged_topic').value,
            self._on_amcl_converged,
            10,
        )
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            self.get_parameter('initialpose_topic').value,
            qos_depth,
        )

        timer_period = float(self.get_parameter('publish_period').value)
        self.timer = self.create_timer(timer_period, self._try_publish)
        self.get_logger().info(
            'Waiting for /map, /scan and /Odometry before publishing AMCL initial pose.'
        )

    def _on_map(self, _msg):
        self.map_ready = True

    def _on_scan(self, _msg):
        self.scan_ready = True

    def _on_odom(self, _msg):
        self.odom_ready = True

    def _on_amcl_converged(self, msg):
        self.amcl_converged = bool(msg.data)

    def _try_publish(self):
        if self.waiting_for_map_tf:
            self._wait_for_map_tf()
            return
        if self.waiting_for_convergence:
            self._wait_for_amcl_convergence()
            return

        missing = []
        if not self.map_ready:
            missing.append('/map')
        if not self.scan_ready:
            missing.append('/scan')
        if not self.odom_ready:
            missing.append('/Odometry')

        if missing:
            self.get_logger().info(
                'Waiting for topics: ' + ', '.join(missing),
                throttle_duration_sec=5.0,
            )
            return

        if self.ready_since is None:
            self.ready_since = time.monotonic()
            self.get_logger().info(
                f'Navigation inputs are ready. Waiting {self.settle_time:.1f}s for TF buffer to settle.'
            )
            return

        elapsed = time.monotonic() - self.ready_since
        if elapsed < self.settle_time:
            return

        if self.publisher.get_subscription_count() == 0:
            self.get_logger().info(
                'Waiting for AMCL subscription on /initialpose.',
                throttle_duration_sec=5.0,
            )
            return

        msg = PoseWithCovarianceStamped()
        # 初始位姿使用 0 时间戳，让 TF 按最新可用变换处理，避免启动阶段轻微时间滞后导致外推警告。
        msg.header.frame_id = self.frame_id
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0

        half_yaw = self.yaw * 0.5
        msg.pose.pose.orientation.z = math.sin(half_yaw)
        msg.pose.pose.orientation.w = math.cos(half_yaw)

        xy_cov = self.xy_stddev * self.xy_stddev
        yaw_cov = self.yaw_stddev * self.yaw_stddev
        msg.pose.covariance[0] = xy_cov
        msg.pose.covariance[7] = xy_cov
        msg.pose.covariance[35] = yaw_cov

        self.publisher.publish(msg)
        self.sent_count += 1
        self.get_logger().info(
            f'Published AMCL initial pose {self.sent_count}/{self.publish_count}: '
            f'x={self.x:.3f}, y={self.y:.3f}, yaw={self.yaw:.3f}, frame={self.frame_id}'
        )

        if self.sent_count >= self.publish_count:
            if self.require_map_to_base_tf:
                self.waiting_for_map_tf = True
                self.map_tf_wait_started = time.monotonic()
                self.get_logger().info(
                    f'Waiting for AMCL transform {self.frame_id} -> {self.base_frame_id}.'
                )
            else:
                self.get_logger().info('Initial pose publishing complete.')
                self.finished = True

    def _wait_for_map_tf(self):
        # 等到 AMCL 真正发布 map -> base_footprint 后，再让后续 Nav2 规划/控制节点启动。
        if self.tf_buffer.can_transform(self.frame_id, self.base_frame_id, Time()):
            self.get_logger().info(
                f'AMCL transform {self.frame_id} -> {self.base_frame_id} is ready.'
            )
            if self.require_amcl_convergence:
                self.waiting_for_map_tf = False
                self.waiting_for_convergence = True
                self.convergence_wait_started = time.monotonic()
                self.get_logger().info(
                    'Waiting for strict AMCL convergence score before starting navigation.'
                )
                return
            self.get_logger().info('Initial pose publishing complete.')
            self.finished = True
            return

        elapsed = time.monotonic() - self.map_tf_wait_started
        if elapsed >= self.post_publish_timeout:
            self.get_logger().warn(
                f'Timed out waiting for {self.frame_id} -> {self.base_frame_id}; continuing launch.'
            )
            self.finished = True
            return

        self.get_logger().info(
            f'Waiting for {self.frame_id} -> {self.base_frame_id} TF after initial pose.',
            throttle_duration_sec=2.0,
        )

    def _wait_for_amcl_convergence(self):
        # 严格启动模式下，先等 AMCL 多指标收敛，再放行后续 Nav2 节点。
        if self.amcl_converged:
            self.get_logger().info('AMCL convergence monitor reports converged. Initial pose stage complete.')
            self.finished = True
            return

        elapsed = time.monotonic() - self.convergence_wait_started
        if self.amcl_convergence_timeout > 0.0 and elapsed >= self.amcl_convergence_timeout:
            if self.continue_on_amcl_convergence_timeout:
                self.get_logger().warn(
                    'Timed out waiting for strict AMCL convergence; continuing launch so the operator can inspect RViz.'
                )
                self.finished = True
                return
            self.get_logger().warn(
                'Timed out waiting for strict AMCL convergence; keeping Nav2 gated because convergence is required.',
                throttle_duration_sec=5.0,
            )
            return

        self.get_logger().info(
            'Waiting for AMCL convergence monitor before starting navigation.',
            throttle_duration_sec=2.0,
        )


def main():
    rclpy.init()
    node = InitialPosePublisher()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
