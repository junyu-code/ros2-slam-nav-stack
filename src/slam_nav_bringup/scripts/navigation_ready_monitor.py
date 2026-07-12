#!/usr/bin/env python3

import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


class NavigationReadyMonitor(Node):
    def __init__(self):
        super().__init__('navigation_ready_monitor')

        self.declare_parameter('localization_ready_topic', '/localization_ready')
        self.declare_parameter('navigation_ready_topic', '/navigation_ready')
        self.declare_parameter('localization_mode', 'amcl')
        self.declare_parameter('navigate_to_pose_action', '/navigate_to_pose')

        localization_topic = str(
            self.get_parameter('localization_ready_topic').value
        )
        navigation_topic = str(
            self.get_parameter('navigation_ready_topic').value
        )
        localization_mode = str(
            self.get_parameter('localization_mode').value
        ).strip().lower()
        action_name = str(
            self.get_parameter('navigate_to_pose_action').value
        )

        self.localization_ready = localization_mode == 'static'
        self.navigation_ready = False
        self.announced = False

        if localization_mode != 'static':
            self.create_subscription(
                Bool,
                localization_topic,
                self._localization_ready_callback,
                10,
            )

        ready_qos = QoSProfile(depth=1)
        ready_qos.reliability = ReliabilityPolicy.RELIABLE
        ready_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.ready_pub = self.create_publisher(Bool, navigation_topic, ready_qos)
        self.navigate_client = ActionClient(self, NavigateToPose, action_name)
        self.timer = self.create_timer(0.5, self._check_ready)

        self.get_logger().info(
            f'Waiting for localization and Nav2 action server {action_name}.'
        )

    def _localization_ready_callback(self, msg: Bool):
        self.localization_ready = bool(msg.data)

    def _check_ready(self):
        ready = bool(
            self.localization_ready and self.navigate_client.server_is_ready()
        )
        ready_msg = Bool()
        ready_msg.data = ready
        self.ready_pub.publish(ready_msg)

        if ready and not self.announced:
            self.announced = True
            banner = (
                '\n\033[1;32m'
                '================================================================\n'
                '                 NAVIGATION READY / 导航已就绪\n'
                '       定位和 Nav2 均已启动，可以在 RViz 发布 Nav2 Goal\n'
                '================================================================'
                '\033[0m\n'
            )
            print(banner, flush=True)
            self.get_logger().info(
                'Navigation is ready. /navigation_ready=true'
            )
        elif not ready and self.announced:
            self.announced = False
            self.get_logger().warn(
                'Navigation is no longer ready. /navigation_ready=false'
            )


def main():
    rclpy.init()
    node = NavigationReadyMonitor()
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
