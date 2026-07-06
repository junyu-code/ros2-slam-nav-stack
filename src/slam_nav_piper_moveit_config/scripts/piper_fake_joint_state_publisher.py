#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class PiperFakeJointStatePublisher(Node):
    """Piper MoveIt2 plan-only 假关节状态发布器，不连接真实机械臂。"""

    def __init__(self):
        super().__init__('piper_fake_joint_state_publisher')
        self.declare_parameter(
            'joint_names',
            [
                'piper_joint1',
                'piper_joint2',
                'piper_joint3',
                'piper_joint4',
                'piper_joint5',
                'piper_joint6',
                'piper_joint7',
                'piper_joint8',
            ],
        )
        self.declare_parameter('joint_positions', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.038, -0.038])
        self.declare_parameter('publish_rate_hz', 20.0)

        self.joint_names = [str(item) for item in self.get_parameter('joint_names').value]
        self.joint_positions = [float(item) for item in self.get_parameter('joint_positions').value]
        if len(self.joint_positions) != len(self.joint_names):
            self.get_logger().warn('joint_positions 长度不匹配，改用全零关节位置。')
            self.joint_positions = [0.0 for _ in self.joint_names]

        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        period = 1.0 / max(1.0, float(self.get_parameter('publish_rate_hz').value))
        self.timer = self.create_timer(period, self.publish_joint_state)
        self.get_logger().info('已启动 Piper plan-only 假关节状态发布器；不会控制真实硬件。')

    def publish_joint_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.joint_positions
        msg.velocity = [0.0 for _ in self.joint_names]
        msg.effort = [0.0 for _ in self.joint_names]
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = PiperFakeJointStatePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
