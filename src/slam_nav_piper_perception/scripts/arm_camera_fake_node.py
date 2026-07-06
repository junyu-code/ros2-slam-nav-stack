#!/usr/bin/env python3

from array import array

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class ArmCameraFakeNode(Node):
    """发布独立的 Piper 腕部 RGB-D 相机假数据，用于命名空间和链路冒烟测试。"""

    def __init__(self):
        super().__init__('arm_camera_fake_node')
        self.declare_parameter('frame_id', 'piper_arm_camera_optical_frame')
        self.declare_parameter('width', 320)
        self.declare_parameter('height', 240)
        self.declare_parameter('depth_mm', 900)
        self.declare_parameter('publish_hz', 10.0)
        self.declare_parameter('color_image_topic', '/piper/arm_camera/color/image_raw')
        self.declare_parameter('depth_image_topic', '/piper/arm_camera/depth/image_raw')
        self.declare_parameter('color_camera_info_topic', '/piper/arm_camera/color/camera_info')
        self.declare_parameter('depth_camera_info_topic', '/piper/arm_camera/depth/camera_info')

        self.frame_id = str(self.get_parameter('frame_id').value)
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.depth_mm = int(self.get_parameter('depth_mm').value)
        publish_hz = max(float(self.get_parameter('publish_hz').value), 1.0)

        self.color_pub = self.create_publisher(
            Image,
            str(self.get_parameter('color_image_topic').value),
            10,
        )
        self.depth_pub = self.create_publisher(
            Image,
            str(self.get_parameter('depth_image_topic').value),
            10,
        )
        self.color_info_pub = self.create_publisher(
            CameraInfo,
            str(self.get_parameter('color_camera_info_topic').value),
            10,
        )
        self.depth_info_pub = self.create_publisher(
            CameraInfo,
            str(self.get_parameter('depth_camera_info_topic').value),
            10,
        )
        self.timer = self.create_timer(1.0 / publish_hz, self.publish_camera)

        self.get_logger().info('Piper 腕部假 RGB-D 相机已启动，话题位于 /piper/arm_camera/*。')

    def make_camera_info(self):
        msg = CameraInfo()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.width = self.width
        msg.height = self.height
        fx = 300.0
        fy = 300.0
        cx = float(self.width) * 0.5
        cy = float(self.height) * 0.5
        msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return msg

    def publish_camera(self):
        stamp = self.get_clock().now().to_msg()

        color = Image()
        color.header.stamp = stamp
        color.header.frame_id = self.frame_id
        color.height = self.height
        color.width = self.width
        color.encoding = 'rgb8'
        color.is_bigendian = 0
        color.step = self.width * 3
        color.data = bytes([40, 120, 210]) * (self.width * self.height)

        depth = Image()
        depth.header.stamp = stamp
        depth.header.frame_id = self.frame_id
        depth.height = self.height
        depth.width = self.width
        depth.encoding = '16UC1'
        depth.is_bigendian = 0
        depth.step = self.width * 2
        depth_values = array('H', [self.depth_mm] * (self.width * self.height))
        depth.data = depth_values.tobytes()

        color_info = self.make_camera_info()
        depth_info = self.make_camera_info()
        color_info.header.stamp = stamp
        depth_info.header.stamp = stamp

        self.color_pub.publish(color)
        self.depth_pub.publish(depth)
        self.color_info_pub.publish(color_info)
        self.depth_info_pub.publish(depth_info)


def main():
    rclpy.init()
    node = ArmCameraFakeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
