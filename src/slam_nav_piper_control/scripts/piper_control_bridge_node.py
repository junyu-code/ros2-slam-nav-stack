#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger


class PiperControlBridgeNode(Node):
    """统一 Piper 控制边界，防止上层直接依赖 MoveIt2 或厂家 SDK 话题。"""

    VALID_OWNERS = {'moveit', 'sdk_test', 'disabled'}

    def __init__(self):
        super().__init__('piper_control_bridge_node')
        self.declare_parameter('backend', 'moveit')
        self.declare_parameter('initial_owner', 'disabled')
        self.declare_parameter('auto_enable', False)
        self.declare_parameter('state_topic', '/piper/control/state')
        self.declare_parameter('owner_request_topic', '/piper/control/owner_request')

        self.backend = str(self.get_parameter('backend').value)
        self.owner = self.normalize_owner(str(self.get_parameter('initial_owner').value))
        self.enabled = bool(self.get_parameter('auto_enable').value)
        self.estopped = False
        self.last_command_wall = time.monotonic()

        self.state_pub = self.create_publisher(String, str(self.get_parameter('state_topic').value), 10)
        self.create_subscription(
            String,
            str(self.get_parameter('owner_request_topic').value),
            self.owner_request_callback,
            10,
        )

        self.create_service(Trigger, '/piper/control/disable', self.disable_callback)
        self.create_service(Trigger, '/piper/control/enable', self.enable_callback)
        self.create_service(Trigger, '/piper/control/estop', self.estop_callback)
        self.create_service(Trigger, '/piper/control/clear_estop', self.clear_estop_callback)
        self.create_service(Trigger, '/piper/control/home', self.home_callback)
        self.create_service(SetBool, '/piper/control/set_auto_enable', self.set_auto_enable_callback)
        self.timer = self.create_timer(0.5, self.publish_state)

        self.get_logger().info(
            f'Piper 控制桥已启动: backend={self.backend}, owner={self.owner}, auto_enable={self.enabled}'
        )

    def normalize_owner(self, owner):
        if owner not in self.VALID_OWNERS:
            return 'disabled'
        return owner

    def owner_request_callback(self, msg):
        requested = self.normalize_owner(msg.data.strip())
        if self.estopped and requested != 'disabled':
            self.get_logger().warn('Piper 急停状态下拒绝切换控制 owner。')
            return
        self.owner = requested
        self.last_command_wall = time.monotonic()
        self.get_logger().info(f'Piper 控制 owner 切换为: {self.owner}')
        self.publish_state()

    def disable_callback(self, _request, response):
        self.enabled = False
        self.owner = 'disabled'
        response.success = True
        response.message = 'Piper 已失能，owner=disabled。'
        self.publish_state()
        return response

    def enable_callback(self, _request, response):
        if self.estopped:
            response.success = False
            response.message = 'Piper 处于急停状态，需先 clear_estop。'
            return response
        self.enabled = True
        response.success = True
        response.message = f'Piper 已允许控制，当前后端={self.backend}。'
        self.publish_state()
        return response

    def estop_callback(self, _request, response):
        self.estopped = True
        self.enabled = False
        self.owner = 'disabled'
        response.success = True
        response.message = 'Piper 急停已触发，控制已失能。'
        self.publish_state()
        return response

    def clear_estop_callback(self, _request, response):
        self.estopped = False
        response.success = True
        response.message = 'Piper 急停状态已清除，仍需显式 enable。'
        self.publish_state()
        return response

    def home_callback(self, _request, response):
        if not self.enabled or self.estopped:
            response.success = False
            response.message = 'Piper 未使能或处于急停，拒绝 home。'
            return response
        response.success = True
        response.message = 'Piper home 请求已由控制桥接收；当前为占位后端，不直接驱动硬件。'
        self.last_command_wall = time.monotonic()
        self.publish_state()
        return response

    def set_auto_enable_callback(self, request, response):
        self.enabled = bool(request.data) and not self.estopped
        response.success = self.enabled == bool(request.data)
        response.message = 'Piper auto_enable 已更新。'
        self.publish_state()
        return response

    def publish_state(self):
        msg = String()
        msg.data = (
            f'backend={self.backend};owner={self.owner};'
            f'enabled={str(self.enabled).lower()};estop={str(self.estopped).lower()}'
        )
        self.state_pub.publish(msg)


def main():
    rclpy.init()
    node = PiperControlBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
