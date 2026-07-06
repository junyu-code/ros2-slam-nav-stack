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
from vision_msgs.msg import Detection2D, Detection2DArray, Detection3D, Detection3DArray, ObjectHypothesisWithPose

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
        self.declare_parameter('debug_image_topic', '/piper/perception/debug_image')
        self.declare_parameter('target_frame', 'piper_base_link')
        self.declare_parameter('sample_window_px', 12)
        self.declare_parameter('min_depth_m', 0.15)
        self.declare_parameter('max_depth_m', 2.5)
        self.declare_parameter('publish_empty_detections', True)
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('synthetic_detection_class_id', 'center_depth_target')
        self.declare_parameter('synthetic_detection_score', 0.50)
        self.declare_parameter('synthetic_bbox_size_px', 48.0)
        self.declare_parameter('synthetic_bbox_size_m', 0.06)

        self.target_frame = str(self.get_parameter('target_frame').value)
        self.sample_window_px = max(int(self.get_parameter('sample_window_px').value), 1)
        self.min_depth_m = float(self.get_parameter('min_depth_m').value)
        self.max_depth_m = float(self.get_parameter('max_depth_m').value)
        self.publish_empty_detections = bool(self.get_parameter('publish_empty_detections').value)
        self.publish_debug_image = bool(self.get_parameter('publish_debug_image').value)
        self.synthetic_detection_class_id = str(self.get_parameter('synthetic_detection_class_id').value)
        self.synthetic_detection_score = float(self.get_parameter('synthetic_detection_score').value)
        self.synthetic_bbox_size_px = float(self.get_parameter('synthetic_bbox_size_px').value)
        self.synthetic_bbox_size_m = float(self.get_parameter('synthetic_bbox_size_m').value)
        self.latest_color_msg = None

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
        self.debug_image_pub = None
        if self.publish_debug_image:
            self.debug_image_pub = self.create_publisher(
                Image,
                str(self.get_parameter('debug_image_topic').value),
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

    def color_callback(self, msg):
        # 当前假检测只保存最近彩色图用于调试框；正式网络接入后可在这里做 RGB 推理。
        if msg.encoding not in ('rgb8', 'bgr8'):
            self.get_logger().warn(
                f'Piper 调试图只支持 rgb8/bgr8，当前为 {msg.encoding}。',
                throttle_duration_sec=5.0,
            )
            return
        if msg.step < msg.width * 3 or len(msg.data) < msg.step * msg.height:
            self.get_logger().warn('Piper 彩色图尺寸/step 无效，暂不生成调试图。', throttle_duration_sec=5.0)
            return
        self.latest_color_msg = msg

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
        if self.fx is None or self.fy is None:
            self.publish_empty_detection_arrays(msg.header)
            self.get_logger().warn('尚未收到 Piper 深度相机内参，暂不发布目标位姿。', throttle_duration_sec=3.0)
            return
        if msg.encoding not in ('16UC1', 'mono16', '32FC1'):
            self.publish_empty_detection_arrays(msg.header)
            self.get_logger().warn(f'不支持的 Piper 深度图编码: {msg.encoding}', throttle_duration_sec=3.0)
            return

        depth_m = self.center_depth(msg)
        if depth_m is None:
            self.publish_empty_detection_arrays(msg.header)
            self.get_logger().warn('Piper 深度图中心窗口没有可用深度。', throttle_duration_sec=3.0)
            return

        u = float(msg.width) * 0.5
        v = float(msg.height) * 0.5
        camera_pose = PoseStamped()
        camera_pose.header = msg.header
        camera_pose.pose.position.x = (u - self.cx) * depth_m / self.fx
        camera_pose.pose.position.y = (v - self.cy) * depth_m / self.fy
        camera_pose.pose.position.z = depth_m
        camera_pose.pose.orientation.w = 1.0
        target_pose = camera_pose

        if self.target_frame and camera_pose.header.frame_id != self.target_frame:
            if tf2_geometry_msgs is None:
                self.publish_empty_detection_arrays(msg.header)
                self.get_logger().warn(
                    '缺少 tf2_geometry_msgs，无法把目标位姿转换到 piper_base_link。',
                    throttle_duration_sec=3.0,
                )
                return
            try:
                # 假相机与 joint_state_publisher 分别取当前时间，时间戳可能相差几毫秒。
                # 这里使用最新可用 TF，避免低速目标估计被未来外推误差卡住。
                original_stamp = camera_pose.header.stamp
                target_pose.header.stamp = Time().to_msg()
                target_pose = self.tf_buffer.transform(
                    target_pose,
                    self.target_frame,
                    timeout=Duration(seconds=0.20),
                )
                target_pose.header.stamp = original_stamp
            except TransformException as exc:
                self.publish_empty_detection_arrays(msg.header)
                self.get_logger().warn(
                    f'等待 Piper 相机到 {self.target_frame} 的 TF: {exc}',
                    throttle_duration_sec=3.0,
                )
                return

        self.target_pub.publish(target_pose)
        self.publish_synthetic_detections(msg.header, camera_pose, target_pose, u, v)
        self.publish_debug_detection_image(msg.header, u, v)

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

    def publish_debug_detection_image(self, header, u, v):
        if self.debug_image_pub is None or self.latest_color_msg is None:
            return
        color = self.latest_color_msg
        if color.encoding not in ('rgb8', 'bgr8'):
            return

        debug = Image()
        debug.header = header
        debug.height = color.height
        debug.width = color.width
        debug.encoding = color.encoding
        debug.is_bigendian = color.is_bigendian
        debug.step = color.step
        data = bytearray(color.data)

        half = max(int(self.synthetic_bbox_size_px * 0.5), 1)
        center_u = int(round(u))
        center_v = int(round(v))
        min_u = max(center_u - half, 0)
        max_u = min(center_u + half, int(color.width) - 1)
        min_v = max(center_v - half, 0)
        max_v = min(center_v + half, int(color.height) - 1)
        line_color = (255, 80, 0) if color.encoding == 'rgb8' else (0, 80, 255)

        # 直接在字节数组上画 2px 检测框，避免给轻量假感知引入 OpenCV 依赖。
        for thickness in range(2):
            top = min(min_v + thickness, max_v)
            bottom = max(max_v - thickness, min_v)
            left = min(min_u + thickness, max_u)
            right = max(max_u - thickness, min_u)
            for pixel_u in range(left, right + 1):
                self.set_rgb_pixel(data, color, pixel_u, top, line_color)
                self.set_rgb_pixel(data, color, pixel_u, bottom, line_color)
            for pixel_v in range(top, bottom + 1):
                self.set_rgb_pixel(data, color, left, pixel_v, line_color)
                self.set_rgb_pixel(data, color, right, pixel_v, line_color)

        debug.data = bytes(data)
        self.debug_image_pub.publish(debug)

    def set_rgb_pixel(self, data, image, pixel_u, pixel_v, color):
        offset = int(pixel_v) * int(image.step) + int(pixel_u) * 3
        if offset + 3 > len(data):
            return
        data[offset:offset + 3] = bytes(color)

    def make_hypothesis(self, pose):
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = self.synthetic_detection_class_id
        hypothesis.hypothesis.score = self.synthetic_detection_score
        hypothesis.pose.pose = pose
        return hypothesis

    def publish_synthetic_detections(self, camera_header, camera_pose, target_pose, u, v):
        # 假检测让 detection topic 具备真实数据形态；正式视觉模型接入后替换此函数即可。
        detection_2d = Detection2D()
        detection_2d.header = camera_header
        detection_2d.id = self.synthetic_detection_class_id
        detection_2d.bbox.center.position.x = u
        detection_2d.bbox.center.position.y = v
        detection_2d.bbox.center.theta = 0.0
        detection_2d.bbox.size_x = self.synthetic_bbox_size_px
        detection_2d.bbox.size_y = self.synthetic_bbox_size_px
        detection_2d.results.append(self.make_hypothesis(camera_pose.pose))

        detections_2d = Detection2DArray()
        detections_2d.header = camera_header
        detections_2d.detections.append(detection_2d)

        detection_3d = Detection3D()
        detection_3d.header = target_pose.header
        detection_3d.id = self.synthetic_detection_class_id
        detection_3d.bbox.center = target_pose.pose
        detection_3d.bbox.size.x = self.synthetic_bbox_size_m
        detection_3d.bbox.size.y = self.synthetic_bbox_size_m
        detection_3d.bbox.size.z = self.synthetic_bbox_size_m
        detection_3d.results.append(self.make_hypothesis(target_pose.pose))

        detections_3d = Detection3DArray()
        detections_3d.header = target_pose.header
        detections_3d.detections.append(detection_3d)

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
