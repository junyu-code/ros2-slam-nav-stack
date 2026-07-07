#!/usr/bin/env python3

import json
import math
import time
from collections import deque
from typing import Optional

import rclpy
import tf2_ros
from geometry_msgs.msg import PoseArray, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
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
from std_msgs.msg import Bool, Float32, String


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_to_matrix(q):
    norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if norm < 1e-9:
        return ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    x, y, z, w = q.x / norm, q.y / norm, q.z / norm, q.w / norm
    return (
        (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)),
        (2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)),
        (2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)),
    )


def score_from_limit(value: Optional[float], good_limit: float, bad_limit: float) -> float:
    if value is None or not math.isfinite(value):
        return 0.0
    if value <= good_limit:
        return 100.0
    if value >= bad_limit:
        return 0.0
    return 100.0 * (bad_limit - value) / max(bad_limit - good_limit, 1e-9)


class AmclConvergenceMonitor(Node):
    """AMCL ????????? covariance?TF ???????? scan-map ???"""

    def __init__(self):
        super().__init__('amcl_convergence_monitor')

        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.amcl_pose_topic = self.declare_parameter('amcl_pose_topic', '/amcl_pose').value
        self.particle_cloud_topic = self.declare_parameter('particle_cloud_topic', '/particle_cloud').value
        self.scan_topic = self.declare_parameter('scan_topic', '/scan').value
        self.map_topic = self.declare_parameter('map_topic', '/map').value
        self.converged_topic = self.declare_parameter('converged_topic', '/amcl_converged').value
        self.status_topic = self.declare_parameter('status_topic', '/amcl_convergence_status').value
        self.score_topic = self.declare_parameter('score_topic', '/amcl_convergence_score').value

        self.check_rate_hz = float(self.declare_parameter('check_rate_hz', 2.0).value)
        self.max_msg_age_sec = float(self.declare_parameter('max_msg_age_sec', 2.5).value)
        self.max_pose_age_sec = float(self.declare_parameter('max_pose_age_sec', self.max_msg_age_sec).value)
        self.max_particle_age_sec = float(self.declare_parameter('max_particle_age_sec', self.max_msg_age_sec).value)
        self.stability_window_sec = float(self.declare_parameter('stability_window_sec', 4.0).value)
        self.stable_required_sec = float(self.declare_parameter('stable_required_sec', 2.0).value)
        self.score_threshold = float(self.declare_parameter('score_threshold', 85.0).value)
        self.require_particle_cloud = bool(self.declare_parameter('require_particle_cloud', True).value)

        # covariance ???????m^2 / rad^2??????????????????????????
        self.covariance_xy_threshold = float(self.declare_parameter('covariance_xy_threshold', 0.025).value)
        self.covariance_yaw_threshold = float(self.declare_parameter('covariance_yaw_threshold', 0.030).value)
        self.tf_translation_threshold = float(self.declare_parameter('tf_translation_threshold', 0.08).value)
        self.tf_yaw_threshold = float(self.declare_parameter('tf_yaw_threshold', 0.05).value)
        self.particle_rms_threshold = float(self.declare_parameter('particle_rms_threshold', 0.35).value)
        self.particle_max_radius_threshold = float(self.declare_parameter('particle_max_radius_threshold', 0.90).value)
        self.scan_mean_residual_threshold = float(self.declare_parameter('scan_mean_residual_threshold', 0.20).value)
        self.scan_p90_residual_threshold = float(self.declare_parameter('scan_p90_residual_threshold', 0.45).value)
        self.scan_sample_step = int(self.declare_parameter('scan_sample_step', 4).value)
        self.min_scan_samples = int(self.declare_parameter('min_scan_samples', 30).value)
        self.occupied_threshold = int(self.declare_parameter('occupied_threshold', 65).value)

        self.last_amcl_pose = None
        self.last_amcl_pose_time = None
        self.last_particle_cloud = None
        self.last_particle_time = None
        self.last_scan = None
        self.last_scan_time = None
        self.map_msg = None
        self.distance_cells = None
        self.tf_samples = deque()
        self.candidate_since = None
        self.last_converged = False

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, self.map_topic, self._on_map, map_qos)
        self.create_subscription(PoseWithCovarianceStamped, self.amcl_pose_topic, self._on_amcl_pose, 10)
        self.create_subscription(PoseArray, self.particle_cloud_topic, self._on_particles, 10)
        self.create_subscription(LaserScan, self.scan_topic, self._on_scan, qos_profile_sensor_data)

        self.converged_pub = self.create_publisher(Bool, self.converged_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.score_pub = self.create_publisher(Float32, self.score_topic, 10)

        self.create_timer(1.0 / max(self.check_rate_hz, 0.2), self._on_timer)
        self.get_logger().info(
            'AMCL convergence monitor started: '
            f'cov_xy<={self.covariance_xy_threshold:.3f}, '
            f'cov_yaw<={self.covariance_yaw_threshold:.3f}, '
            f'score>={self.score_threshold:.1f}'
        )

    def _on_map(self, msg: OccupancyGrid):
        self.map_msg = msg
        started = time.monotonic()
        self.distance_cells = self._build_distance_field(msg)
        elapsed = time.monotonic() - started
        occupied_count = sum(1 for value in msg.data if value >= self.occupied_threshold)
        self.get_logger().info(
            f'Built map distance field: size={msg.info.width}x{msg.info.height}, '
            f'occupied={occupied_count}, elapsed={elapsed:.2f}s'
        )

    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped):
        self.last_amcl_pose = msg
        self.last_amcl_pose_time = time.monotonic()

    def _on_particles(self, msg: PoseArray):
        self.last_particle_cloud = msg
        self.last_particle_time = time.monotonic()

    def _on_scan(self, msg: LaserScan):
        self.last_scan = msg
        self.last_scan_time = time.monotonic()

    def _on_timer(self):
        now = time.monotonic()
        tf_metrics = self._update_tf_metrics(now)
        cov_metrics = self._covariance_metrics(now)
        particle_metrics = self._particle_metrics(now)
        scan_metrics = self._scan_map_metrics(now)

        covariance_ok = bool(cov_metrics.get('ok'))
        tf_ok = bool(tf_metrics.get('ok'))
        particle_ok = True if not self.require_particle_cloud else bool(particle_metrics.get('ok'))
        scan_ok = bool(scan_metrics.get('ok'))

        covariance_score = min(
            score_from_limit(cov_metrics.get('xy_cov'), self.covariance_xy_threshold, self.covariance_xy_threshold * 3.0),
            score_from_limit(cov_metrics.get('yaw_cov'), self.covariance_yaw_threshold, self.covariance_yaw_threshold * 3.0),
        )
        tf_score = min(
            score_from_limit(tf_metrics.get('translation_span'), self.tf_translation_threshold, self.tf_translation_threshold * 3.0),
            score_from_limit(tf_metrics.get('yaw_span'), self.tf_yaw_threshold, self.tf_yaw_threshold * 3.0),
        )
        if self.require_particle_cloud:
            particle_score = min(
                score_from_limit(particle_metrics.get('rms_radius'), self.particle_rms_threshold, self.particle_rms_threshold * 2.5),
                score_from_limit(particle_metrics.get('max_radius'), self.particle_max_radius_threshold, self.particle_max_radius_threshold * 2.0),
            )
        else:
            # 有些 Nav2 AMCL 启动方式不会稳定发布 /particle_cloud，不能让可视化辅助话题卡死启动门控。
            particle_score = 100.0
        scan_score = min(
            score_from_limit(scan_metrics.get('mean_residual'), self.scan_mean_residual_threshold, self.scan_mean_residual_threshold * 3.0),
            score_from_limit(scan_metrics.get('p90_residual'), self.scan_p90_residual_threshold, self.scan_p90_residual_threshold * 2.5),
        )

        # covariance ???????????? AMCL ??????????????
        total_score = 0.40 * covariance_score + 0.25 * tf_score + 0.20 * particle_score + 0.15 * scan_score
        candidate = covariance_ok and tf_ok and particle_ok and scan_ok and total_score >= self.score_threshold
        if candidate:
            if self.candidate_since is None:
                self.candidate_since = now
            converged = (now - self.candidate_since) >= self.stable_required_sec
        else:
            self.candidate_since = None
            converged = False

        self._publish_status(
            now, converged, candidate, total_score, covariance_score, tf_score, particle_score, scan_score,
            cov_metrics, tf_metrics, particle_metrics, scan_metrics,
        )

    def _covariance_metrics(self, now):
        if self.last_amcl_pose is None or self.last_amcl_pose_time is None:
            return {'ok': False, 'reason': 'amcl_pose_never_seen'}
        age = now - self.last_amcl_pose_time
        if self.max_pose_age_sec > 0.0 and age > self.max_pose_age_sec:
            return {'ok': False, 'reason': f'amcl_pose_stale:{age:.2f}s'}
        cov = self.last_amcl_pose.pose.covariance
        xy_cov = max(float(cov[0]), float(cov[7]))
        yaw_cov = float(cov[35])
        return {
            'ok': xy_cov <= self.covariance_xy_threshold and yaw_cov <= self.covariance_yaw_threshold,
            'xy_cov': xy_cov,
            'yaw_cov': yaw_cov,
            'xy_stddev': math.sqrt(max(xy_cov, 0.0)),
            'yaw_stddev': math.sqrt(max(yaw_cov, 0.0)),
            'age_sec': round(age, 3),
        }

    def _update_tf_metrics(self, now):
        try:
            transform = self.tf_buffer.lookup_transform(self.map_frame, self.odom_frame, Time())
        except Exception as exc:  # noqa: BLE001
            return {'ok': False, 'reason': f'tf_unavailable:{exc}'}

        t = transform.transform.translation
        yaw = yaw_from_quaternion(transform.transform.rotation)
        self.tf_samples.append((now, float(t.x), float(t.y), yaw))
        while self.tf_samples and now - self.tf_samples[0][0] > self.stability_window_sec:
            self.tf_samples.popleft()

        if len(self.tf_samples) < 2:
            return {'ok': False, 'reason': 'tf_window_not_ready'}
        covered = self.tf_samples[-1][0] - self.tf_samples[0][0]
        if covered < self.stability_window_sec * 0.75:
            return {'ok': False, 'reason': f'tf_window_short:{covered:.2f}s'}

        base = self.tf_samples[0]
        translation_span = max(math.hypot(sample[1] - base[1], sample[2] - base[2]) for sample in self.tf_samples)
        yaw_span = max(abs(normalize_angle(sample[3] - base[3])) for sample in self.tf_samples)
        return {
            'ok': translation_span <= self.tf_translation_threshold and yaw_span <= self.tf_yaw_threshold,
            'translation_span': translation_span,
            'yaw_span': yaw_span,
            'window_sec': round(covered, 3),
        }

    def _particle_metrics(self, now):
        if self.last_particle_cloud is None or self.last_particle_time is None:
            if not self.require_particle_cloud:
                return {'ok': True, 'optional': True, 'reason': 'particle_cloud_optional_never_seen'}
            return {'ok': False, 'reason': 'particle_cloud_never_seen'}
        age = now - self.last_particle_time
        if self.max_particle_age_sec > 0.0 and age > self.max_particle_age_sec:
            if not self.require_particle_cloud:
                return {'ok': True, 'optional': True, 'reason': f'particle_cloud_optional_stale:{age:.2f}s'}
            return {'ok': False, 'reason': f'particle_cloud_stale:{age:.2f}s'}
        poses = self.last_particle_cloud.poses
        if not poses:
            if not self.require_particle_cloud:
                return {'ok': True, 'optional': True, 'reason': 'particle_cloud_optional_empty'}
            return {'ok': False, 'reason': 'particle_cloud_empty'}
        mean_x = sum(p.position.x for p in poses) / len(poses)
        mean_y = sum(p.position.y for p in poses) / len(poses)
        distances = [math.hypot(p.position.x - mean_x, p.position.y - mean_y) for p in poses]
        rms_radius = math.sqrt(sum(d * d for d in distances) / len(distances))
        max_radius = max(distances)
        return {
            'ok': rms_radius <= self.particle_rms_threshold and max_radius <= self.particle_max_radius_threshold,
            'count': len(poses),
            'rms_radius': rms_radius,
            'max_radius': max_radius,
            'age_sec': round(age, 3),
        }

    def _scan_map_metrics(self, now):
        if self.map_msg is None or self.distance_cells is None:
            return {'ok': False, 'reason': 'map_distance_field_not_ready'}
        if self.last_scan is None or self.last_scan_time is None:
            return {'ok': False, 'reason': 'scan_never_seen'}
        age = now - self.last_scan_time
        if age > self.max_msg_age_sec:
            return {'ok': False, 'reason': f'scan_stale:{age:.2f}s'}

        try:
            transform = self.tf_buffer.lookup_transform(self.map_frame, self.last_scan.header.frame_id, Time())
        except Exception as exc:  # noqa: BLE001
            return {'ok': False, 'reason': f'scan_tf_unavailable:{exc}'}

        residuals = []
        rot = quaternion_to_matrix(transform.transform.rotation)
        trans = transform.transform.translation
        step = max(self.scan_sample_step, 1)
        for i in range(0, len(self.last_scan.ranges), step):
            distance = float(self.last_scan.ranges[i])
            if not math.isfinite(distance):
                continue
            if distance < self.last_scan.range_min or distance > self.last_scan.range_max:
                continue
            angle = self.last_scan.angle_min + i * self.last_scan.angle_increment
            lx = distance * math.cos(angle)
            ly = distance * math.sin(angle)
            mx = rot[0][0] * lx + rot[0][1] * ly + trans.x
            my = rot[1][0] * lx + rot[1][1] * ly + trans.y
            cell_distance = self._distance_to_nearest_occupied(mx, my)
            if cell_distance is not None:
                residuals.append(cell_distance)

        if len(residuals) < self.min_scan_samples:
            return {'ok': False, 'reason': f'scan_samples_low:{len(residuals)}', 'samples': len(residuals)}
        residuals.sort()
        mean_residual = sum(residuals) / len(residuals)
        p90_residual = residuals[int(0.90 * (len(residuals) - 1))]
        return {
            'ok': mean_residual <= self.scan_mean_residual_threshold and p90_residual <= self.scan_p90_residual_threshold,
            'samples': len(residuals),
            'mean_residual': mean_residual,
            'p90_residual': p90_residual,
            'age_sec': round(age, 3),
        }

    def _build_distance_field(self, msg: OccupancyGrid):
        width = int(msg.info.width)
        height = int(msg.info.height)
        inf = 1_000_000
        distances = [inf] * (width * height)
        queue = deque()
        for index, value in enumerate(msg.data):
            if value >= self.occupied_threshold:
                distances[index] = 0
                queue.append(index)
        neighbors = ((-1, -1), (0, -1), (1, -1), (-1, 0), (1, 0), (-1, 1), (0, 1), (1, 1))
        while queue:
            index = queue.popleft()
            x = index % width
            y = index // width
            next_distance = distances[index] + 1
            for dx, dy in neighbors:
                nx = x + dx
                ny = y + dy
                if nx < 0 or nx >= width or ny < 0 or ny >= height:
                    continue
                nindex = ny * width + nx
                if next_distance < distances[nindex]:
                    distances[nindex] = next_distance
                    queue.append(nindex)
        return distances

    def _distance_to_nearest_occupied(self, x: float, y: float) -> Optional[float]:
        info = self.map_msg.info
        resolution = float(info.resolution)
        origin = info.origin
        yaw = yaw_from_quaternion(origin.orientation)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        dx = x - origin.position.x
        dy = y - origin.position.y
        # world -> map grid????? origin yaw?
        gx = cos_yaw * dx + sin_yaw * dy
        gy = -sin_yaw * dx + cos_yaw * dy
        ix = int(math.floor(gx / resolution))
        iy = int(math.floor(gy / resolution))
        if ix < 0 or ix >= info.width or iy < 0 or iy >= info.height:
            return None
        cell_distance = self.distance_cells[iy * info.width + ix]
        if cell_distance >= 1_000_000:
            return None
        return float(cell_distance) * resolution

    def _publish_status(self, now, converged, candidate, total_score, covariance_score, tf_score, particle_score, scan_score, cov_metrics, tf_metrics, particle_metrics, scan_metrics):
        payload = {
            'converged': converged,
            'candidate': candidate,
            'score': round(total_score, 2),
            'score_threshold': self.score_threshold,
            'candidate_duration_sec': None if self.candidate_since is None else round(now - self.candidate_since, 3),
            'gates': {
                'covariance': bool(cov_metrics.get('ok')),
                'tf_stable': bool(tf_metrics.get('ok')),
                'particle_cloud': bool(particle_metrics.get('ok')),
                'scan_map_residual': bool(scan_metrics.get('ok')),
            },
            'subscores': {
                'covariance': round(covariance_score, 2),
                'tf_stable': round(tf_score, 2),
                'particle_cloud': round(particle_score, 2),
                'scan_map_residual': round(scan_score, 2),
            },
            'metrics': {
                'covariance': self._round_metrics(cov_metrics),
                'tf_stable': self._round_metrics(tf_metrics),
                'particle_cloud': self._round_metrics(particle_metrics),
                'scan_map_residual': self._round_metrics(scan_metrics),
            },
        }
        bool_msg = Bool()
        bool_msg.data = converged
        self.converged_pub.publish(bool_msg)

        score_msg = Float32()
        score_msg.data = float(total_score)
        self.score_pub.publish(score_msg)

        status_msg = String()
        status_msg.data = json.dumps(payload, ensure_ascii=False)
        self.status_pub.publish(status_msg)

        if converged != self.last_converged:
            level = self.get_logger().info if converged else self.get_logger().warn
            level(f'AMCL converged={converged}: score={total_score:.1f}, gates={payload["gates"]}')
            self.last_converged = converged

    @staticmethod
    def _round_metrics(metrics):
        rounded = {}
        for key, value in metrics.items():
            if isinstance(value, float):
                rounded[key] = round(value, 4)
            else:
                rounded[key] = value
        return rounded


def main(args=None):
    rclpy.init(args=args)
    node = AmclConvergenceMonitor()
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
