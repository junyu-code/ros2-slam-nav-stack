#!/usr/bin/env python3

import argparse
import math
import time
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, Imu, LaserScan, PointCloud2
from tf2_msgs.msg import TFMessage

try:
    from livox_ros_driver2.msg import CustomMsg as LivoxCustomMsg
except Exception:  # pragma: no cover - 只在未安装 Livox 消息包时触发
    LivoxCustomMsg = None


def stamp_to_sec(stamp) -> Optional[float]:
    if stamp is None:
        return None
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def message_stamp_sec(msg) -> Optional[float]:
    header = getattr(msg, 'header', None)
    if header is None:
        return None
    return stamp_to_sec(header.stamp)


@dataclass
class TopicStats:
    name: str
    required: bool = True
    count: int = 0
    first_wall: Optional[float] = None
    last_wall: Optional[float] = None
    last_stamp: Optional[float] = None

    def observe(self, msg) -> None:
        now = time.monotonic()
        if self.first_wall is None:
            self.first_wall = now
        self.last_wall = now
        self.count += 1
        stamp = message_stamp_sec(msg)
        if stamp is not None:
            self.last_stamp = stamp

    def hz(self) -> Optional[float]:
        if self.count < 2 or self.first_wall is None or self.last_wall is None:
            return None
        elapsed = self.last_wall - self.first_wall
        if elapsed <= 1.0e-6:
            return None
        return float(self.count - 1) / elapsed


class RuntimeDoctor(Node):
    """采样 ROS 图，检查导航链路中的时间、话题和 TF 状态。"""

    def __init__(self, args):
        super().__init__('slam_nav_runtime_doctor')
        self.args = args
        self.real_mode = args.real or args.time_mode == 'real'
        self.clock_sec: Optional[float] = None
        self.topic_stats: Dict[str, TopicStats] = {}
        self.tf_edges: Dict[str, str] = {}
        self.tf_edge_time: Dict[Tuple[str, str], Optional[float]] = {}

        self._subscribe('/clock', Clock, self._on_clock, required=not self.real_mode)
        if LivoxCustomMsg is not None:
            self._subscribe('/livox/lidar', LivoxCustomMsg, None, required=self.real_mode)
        else:
            self.get_logger().warn('livox_ros_driver2 CustomMsg not importable, skip Livox sampling.')
        self._subscribe('/livox/imu', Imu, None, required=self.real_mode)
        self._subscribe('/imu/data', Imu, None, required=self.real_mode)
        self._subscribe('/cloud_registered', PointCloud2, None, required=True)
        self._subscribe('/cloud_nav_filtered', PointCloud2, None, required=False)
        self._subscribe('/nav_obstacle_cloud', PointCloud2, None, required=False)
        self._subscribe('/nav_ground_cloud', PointCloud2, None, required=False)
        self._subscribe('/terrain_map', PointCloud2, None, required=False)
        self._subscribe('/terrain_map_ext', PointCloud2, None, required=False)
        self._subscribe('/visual_obstacles', PointCloud2, None, required=False)
        self._subscribe('/nav_camera/depth/image_raw', Image, None, required=False)
        self._subscribe('/nav_camera/depth/camera_info', CameraInfo, None, required=False)
        self._subscribe('/nav_camera/d435i/depth/image_rect_raw', Image, None, required=False)
        self._subscribe('/nav_camera/d435i/depth/camera_info', CameraInfo, None, required=False)
        self._subscribe('/scan', LaserScan, None, required=True)
        self._subscribe('/Odometry', Odometry, None, required=True)
        self._subscribe('/map', OccupancyGrid, None, required=args.require_map, transient_local=True)
        self._subscribe('/cmd_vel', Twist, None, required=False)
        self._subscribe('/cmd_vel_safe', Twist, None, required=False)

        tf_qos = QoSProfile(depth=100)
        self.create_subscription(TFMessage, '/tf', self._on_tf, tf_qos)
        tf_static_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(TFMessage, '/tf_static', self._on_tf, tf_static_qos)

    def _subscribe(self, topic, msg_type, callback, required=True, transient_local=False) -> None:
        self.topic_stats[topic] = TopicStats(topic, required=required)
        qos = qos_profile_sensor_data
        if transient_local:
            qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )

        def _callback(msg):
            self.topic_stats[topic].observe(msg)
            if callback is not None:
                callback(msg)

        self.create_subscription(msg_type, topic, _callback, qos)

    def _on_clock(self, msg: Clock) -> None:
        self.clock_sec = stamp_to_sec(msg.clock)

    def _on_tf(self, msg: TFMessage) -> None:
        for transform in msg.transforms:
            parent = transform.header.frame_id.lstrip('/')
            child = transform.child_frame_id.lstrip('/')
            if not parent or not child:
                continue
            self.tf_edges[child] = parent
            self.tf_edge_time[(parent, child)] = stamp_to_sec(transform.header.stamp)

    def topic_pub_sub_counts(self, topic: str) -> Tuple[int, int, str, str]:
        publishers = self.get_publishers_info_by_topic(topic)
        subscriptions = self.get_subscriptions_info_by_topic(topic)
        publisher_names = sorted({info.node_name for info in publishers})
        subscription_names = sorted({info.node_name for info in subscriptions})
        return len(publishers), len(subscriptions), ', '.join(publisher_names), ', '.join(subscription_names)

    def costmap_subscribers_for(self, topic: str) -> List[str]:
        subscriptions = self.get_subscriptions_info_by_topic(topic)
        return sorted(
            {
                info.node_name
                for info in subscriptions
                if info.node_name in ('local_costmap', 'global_costmap')
            }
        )

    def can_reach(self, target: str, source: str) -> Tuple[bool, List[str]]:
        target = target.lstrip('/')
        source = source.lstrip('/')
        chain = [source]
        current = source
        visited = set()
        while current in self.tf_edges and current not in visited:
            visited.add(current)
            parent = self.tf_edges[current]
            chain.append(parent)
            if parent == target:
                return True, chain
            current = parent
        return source == target, chain

    def summarize(self) -> Tuple[List[str], List[str], List[str]]:
        lines: List[str] = []
        warnings: List[str] = []
        errors: List[str] = []

        lines.append('== 运行时导航链路诊断 ==')
        lines.append(f'采样时长: {self.args.duration:.1f}s')
        lines.append(f'时间模式: {"real" if self.real_mode else "sim"}')
        if self.clock_sec is None:
            if self.real_mode:
                lines.append('/clock: 未收到（实机 use_sim_time=false 时正常）')
            else:
                errors.append('没有收到 /clock；仿真 use_sim_time 链路可能没有启动。')
                lines.append('/clock: 未收到')
        else:
            lines.append(f'/clock: {self.clock_sec:.3f}s')

        lines.append('')
        lines.append('== 关键话题 ==')
        for topic, stats in self.topic_stats.items():
            pub_count, sub_count, publisher_names, subscription_names = self.topic_pub_sub_counts(topic)
            hz = stats.hz()
            hz_text = 'n/a' if hz is None else f'{hz:.2f} Hz'
            stamp_text = 'no header stamp'
            if stats.last_stamp is not None:
                stamp_text = f'stamp={stats.last_stamp:.3f}s'
                if self.clock_sec is not None:
                    age = self.clock_sec - stats.last_stamp
                    stamp_text += f', clock-stamp={age:+.3f}s'
                    timed_topics = (
                        '/livox/lidar',
                        '/cloud_registered',
                        '/cloud_nav_filtered',
                        '/nav_obstacle_cloud',
                        '/nav_ground_cloud',
                        '/terrain_map',
                        '/terrain_map_ext',
                        '/visual_obstacles',
                        '/nav_camera/depth/image_raw',
                        '/nav_camera/depth/camera_info',
                        '/nav_camera/d435i/depth/image_rect_raw',
                        '/nav_camera/d435i/depth/camera_info',
                        '/scan',
                        '/Odometry',
                    )
                    if topic in timed_topics and abs(age) > self.args.max_stamp_offset:
                        warnings.append(
                            f'{topic} 消息时间与 /clock 差 {age:+.3f}s，可能导致 TF/message_filter 丢帧。'
                        )
            line = (
                f'{topic}: count={stats.count}, hz={hz_text}, '
                f'publishers={pub_count}, subscribers={sub_count}, {stamp_text}'
            )
            if publisher_names:
                line += f', pubs=[{publisher_names}]'
            if subscription_names and topic in (
                '/cloud_nav_filtered',
                '/nav_obstacle_cloud',
                '/nav_ground_cloud',
                '/terrain_map',
                '/terrain_map_ext',
                '/visual_obstacles',
                '/scan',
            ):
                line += f', subs=[{subscription_names}]'
            lines.append(line)

            if stats.required and stats.count == 0:
                errors.append(f'{topic} 未收到消息。')
            elif stats.required and pub_count == 0:
                warnings.append(f'{topic} 当前没有发布者。')

        lines.append('')
        lines.append('== Costmap 观测源 ==')
        costmap_topics = (
            '/scan',
            '/terrain_map',
            '/terrain_map_ext',
            '/nav_obstacle_cloud',
            '/cloud_nav_filtered',
            '/nav_ground_cloud',
            '/visual_obstacles',
        )
        for topic in costmap_topics:
            subscribers = self.costmap_subscribers_for(topic)
            if subscribers:
                lines.append(f'{topic}: subscribed by {", ".join(subscribers)}')
            else:
                lines.append(f'{topic}: no costmap subscriber')

        if not self.args.skip_costmap_checks:
            cloud_nav_costmaps = self.costmap_subscribers_for('/cloud_nav_filtered')
            if cloud_nav_costmaps:
                warnings.append(
                    '/cloud_nav_filtered 仍被 costmap 订阅；默认 3D 导航应改用 /terrain_map 或 /terrain_map_ext，'
                    '请 clean 后重启导航，或检查是否加载了旧参数文件。'
                )
            terrain_costmaps = self.costmap_subscribers_for('/terrain_map')
            if not terrain_costmaps:
                warnings.append('/terrain_map 当前没有被 costmap 订阅；一阶段地形分析没有进入局部代价地图。')
            terrain_ext_costmaps = self.costmap_subscribers_for('/terrain_map_ext')
            if not terrain_ext_costmaps:
                warnings.append('/terrain_map_ext 当前没有被 costmap 订阅；二阶段扩展地形图没有进入全局代价地图。')

            visual_costmaps = self.costmap_subscribers_for('/visual_obstacles')
            visual_publishers = self.get_publishers_info_by_topic('/visual_obstacles')
            if visual_costmaps and not visual_publishers:
                warnings.append(
                    '/visual_obstacles 被 costmap 订阅但没有发布者；若未测试 RGB-D，建议确认当前参数是否为保守默认配置。'
                )

        lines.append('')
        lines.append('== TF 链路 ==')
        tf_pub_count, _, tf_publishers, _ = self.topic_pub_sub_counts('/tf')
        static_pub_count, _, static_publishers, _ = self.topic_pub_sub_counts('/tf_static')
        lines.append(f'/tf publishers={tf_pub_count}: {tf_publishers or "none"}')
        lines.append(f'/tf_static publishers={static_pub_count}: {static_publishers or "none"}')
        if tf_pub_count > self.args.max_tf_publishers:
            warnings.append(
                f'/tf 发布者数量为 {tf_pub_count}，请确认没有旧工作区或重复定位节点同时发布 TF。'
            )

        expected_checks = [
            ('map', 'odom'),
            ('odom', self.args.base_frame),
            ('map', self.args.base_frame),
        ]
        if self.args.check_lidar_frame and self.args.lidar_frame:
            expected_checks.append(('map', self.args.lidar_frame))
        for target, source in expected_checks:
            ok, chain = self.can_reach(target, source)
            arrow_chain = ' <- '.join(chain)
            if ok:
                lines.append(f'{target} -> {source}: OK ({arrow_chain})')
            else:
                lines.append(f'{target} -> {source}: MISSING ({arrow_chain})')
                if target == 'map' and source == 'odom':
                    errors.append('缺少 map -> odom 链路；AMCL/slam_toolbox/重定位节点可能未就绪。')
                elif source == self.args.base_frame:
                    errors.append(f'缺少 {target} -> {source} 链路；Nav2 costmap 可能无法工作。')
                else:
                    warnings.append(f'缺少 {target} -> {source} 链路。')

        if self.tf_edges:
            lines.append('')
            lines.append('已观察到的 TF 边:')
            for child, parent in sorted(self.tf_edges.items()):
                lines.append(f'  {parent} -> {child}')
        else:
            errors.append('没有收到 /tf 或 /tf_static。')

        return lines, warnings, errors


def parse_args():
    parser = argparse.ArgumentParser(
        description='检查 SLAM/Nav2 运行时的话题、时间戳和 TF 链路。'
    )
    parser.add_argument('--duration', type=float, default=5.0, help='采样秒数，默认 5。')
    parser.add_argument('--real', action='store_true', help='按实机 use_sim_time=false 链路检查，不要求 /clock。')
    parser.add_argument(
        '--time-mode',
        choices=('sim', 'real'),
        default='sim',
        help='时间模式，默认 sim。实机可用 --real 或 --time-mode real。',
    )
    parser.add_argument('--base-frame', default='base_footprint', help='导航基坐标，默认 base_footprint。')
    parser.add_argument('--lidar-frame', default='livox_frame', help='雷达坐标，默认 livox_frame。')
    parser.add_argument('--check-lidar-frame', action='store_true', help='额外检查 map 到雷达 frame 的 TF 连通性。')
    parser.add_argument('--require-map', action='store_true', help='要求 /map 在采样窗口内有消息。')
    parser.add_argument('--skip-costmap-checks', action='store_true', help='跳过 Nav2 costmap 订阅检查，适合只查建图。')
    parser.add_argument(
        '--max-stamp-offset',
        type=float,
        default=0.5,
        help='消息时间与 /clock 的最大建议差值，默认 0.5 秒。',
    )
    parser.add_argument(
        '--max-tf-publishers',
        type=int,
        default=8,
        help='超过该 /tf 发布者数量时提示可能存在重复节点，默认 8。',
    )
    parser.add_argument('--strict', action='store_true', help='发现错误时返回非零退出码。')
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = RuntimeDoctor(args)
    end_time = time.monotonic() + max(args.duration, 0.5)
    try:
        while rclpy.ok() and time.monotonic() < end_time:
            rclpy.spin_once(node, timeout_sec=0.1)
        lines, warnings, errors = node.summarize()
        for line in lines:
            print(line)
        if warnings:
            print('')
            print('== 警告 ==')
            for item in warnings:
                print(f'- {item}')
        if errors:
            print('')
            print('== 需要处理 ==')
            for item in errors:
                print(f'- {item}')
        if args.strict and errors:
            raise SystemExit(2)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
