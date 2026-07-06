#!/usr/bin/env python3

import math
import struct

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection2DArray, Detection3DArray

try:
    import tf2_geometry_msgs  # noqa: F401
except Exception:  # pragma: no cover - 仅在运行环境缺少插件时触发
    tf2_geometry_msgs = None


class TargetPoseEstimatorNode(Node):
    """从独立腕部深度相机估计目标中心位姿，后续可替换为真实识别网络。"""

    def __init__(self):
        super().__init__('target_pose_estimator_node')
        self.declare_parameter('color_image_topic', '/piper/arm_camera/color/image_raw')
        self.declare_parameter('depth_image_topic', '/piper/arm_camera/depth/image_raw')
        self.declare_parameter('depth_camera_info_topic', '/piper/arm_camera/depth/camera_info')
        self.declare_parameter('target_pose_topic', '/piper/perception/target_pose')
        self.declare_parameter('detections_2d_topic', '/piper/perception/detections_2d')
        self.declare_parameter('detections_3d_topic', '/piper/perception/detections_3d')
        self.declare_parameter('target_frame', 'piper_base_link')
        self.declare_parameter('sample_window_px', 12)
        self.declare_parameter('min_depth_m', 0.15)
        self.declare_parameter('max_depth_m', 2.5)
        self.declare_parameter('publish_empty_detections', True)

        self.target_frame = str(self.get_parameter('target_frame').value)
        self.sample_window_px = max(int(self.get_parameter('sample_window_px').value), 1)
        self.min_depth_m = float(self.get_parameter('min_depth_m').value)
        self.max_depth_m = float(self.get_parameter('max_depth_m').value)
        self.publish_empty_detections = bool(self.get_parameter('publish_empty_detections').value)

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.target_pub = self.create_publisher(
            PoseStamped,
            str(self.get_parameter('target_pose_topic').value),
            10,
        )
        self.detections_2d_pub = self.create_publisher(
            Detection2DArray,
            str(self.get_parameter('detections_2d_topic').value),
            10,
        )
        self.detections_3d_pub = self.create_publisher(
            Detection3DArray,
            str(self.get_parameter('detections_3d_topic').value),
            10,
        )

        self.create_subscription(
            CameraInfo,
            str(self.get_parameter('depth_camera_info_topic').value),
            self.camera_info_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter('depth_image_topic').value),
            self.depth_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter('color_image_topic').value),
            self.color_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            'Piper 目标位姿估计节点已启动：输入 /piper/arm_camera/*，输出 /piper/perception/*。'
        )

    def color_callback(self, _msg):
        # 当前占位实现不处理 RGB，后续可在这里接入目标检测网络。
        return

    def camera_info_callback(self, msg):
        fx = float(msg.k[0])
        fy = float(msg.k[4])
        if fx <= 0.0 or fy <= 0.0:
            self.get_logger().warn('Piper 相机内参无效，等待有效 CameraInfo。', throttle_duration_sec=3.0)
            return
        self.fx = fx
        self.fy = fy
        self.cx = float(msg.k[2])
        self.cy = float(msg.k[5])

    def depth_callback(self, msg):
        self.publish_empty_detection_arrays(msg.header)

        if self.fx is None or self.fy is None:
            self.get_logger().warn('尚未收到 Piper 深度相机内参，暂不发布目标位姿。', throttle_duration_sec=3.0)
            return
        if msg.encoding not in ('16UC1', 'mono16', '32FC1'):
            self.get_logger().warn(f'不支持的 Piper 深度图编码: {msg.encoding}', throttle_duration_sec=3.0)
            return

        depth_m = self.center_depth(msg)
        if depth_m is None:
            self.get_logger().warn('Piper 深度图中心窗口没有可用深度。', throttle_duration_sec=3.0)
            return

        u = float(msg.width) * 0.5
        v = float(msg.height) * 0.5
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.position.x = (u - self.cx) * depth_m / self.fx
        pose.pose.position.y = (v - self.cy) * depth_m / self.fy
        pose.pose.position.z = depth_m
        pose.pose.orientation.w = 1.0

        if self.target_frame and pose.header.frame_id != self.target_frame:
            if tf2_geometry_msgs is None:
                self.get_logger().warn(
                    '缺少 tf2_geometry_msgs，无法把目标位姿转换到 piper_base_link。',
                    throttle_duration_sec=3.0,
                )
                return
            try:
                # 假相机与 joint_state_publisher 分别取当前时间，时间戳可能相差几毫秒。
                # 这里使用最新可用 TF，避免低速目标估计被未来外推误差卡住。
                original_stamp = pose.header.stamp
                pose.header.stamp = Time().to_msg()
                pose = self.tf_buffer.transform(
                    pose,
                    self.target_frame,
                    timeout=Duration(seconds=0.20),
                )
                pose.header.stamp = original_stamp
            except TransformException as exc:
                self.get_logger().warn(
                    f'等待 Piper 相机到 {self.target_frame} 的 TF: {exc}',
                    throttle_duration_sec=3.0,
                )
                return

        self.target_pub.publish(pose)

    def center_depth(self, msg):
        bytes_per_pixel = 4 if msg.encoding == '32FC1' else 2
        if msg.step < msg.width * bytes_per_pixel:
            return None
        half = self.sample_window_px // 2
        center_u = int(msg.width // 2)
        center_v = int(msg.height // 2)
        values = []
        for v in range(max(0, center_v - half), min(int(msg.height), center_v + half + 1)):
            row = v * msg.step
            for u in range(max(0, center_u - half), min(int(msg.width), center_u + half + 1)):
                offset = row + u * bytes_per_pixel
                if offset + bytes_per_pixel > len(msg.data):
                    continue
                if msg.encoding == '32FC1':
                    depth = struct.unpack_from('<f', msg.data, offset)[0]
                else:
                    raw = struct.unpack_from('<H', msg.data, offset)[0]
                    depth = float(raw) * 0.001 if raw > 0 else math.nan
                if math.isfinite(depth) and self.min_depth_m <= depth <= self.max_depth_m:
                    values.append(depth)
        if not values:
            return None
        values.sort()
        return values[len(values) // 2]

    def publish_empty_detection_arrays(self, header):
        if not self.publish_empty_detections:
            return
        detections_2d = Detection2DArray()
        detections_2d.header = header
        detections_3d = Detection3DArray()
        detections_3d.header = header
        self.detections_2d_pub.publish(detections_2d)
        self.detections_3d_pub.publish(detections_3d)


def main():
    rclpy.init()
    node = TargetPoseEstimatorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
