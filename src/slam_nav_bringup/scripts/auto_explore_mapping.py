#!/usr/bin/env python3

import math
import time
from collections import deque

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class AutoExploreMapper(Node):
    """基于 LaserScan 的保守自动探索节点，用于建图阶段替代手动键盘巡航。"""

    def __init__(self):
        super().__init__('auto_explore_mapper')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/Odometry')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('linear_speed', 0.28)
        self.declare_parameter('turn_speed', 0.65)
        self.declare_parameter('backup_speed', 0.12)
        self.declare_parameter('forward_clearance', 1.15)
        self.declare_parameter('danger_clearance', 0.48)
        self.declare_parameter('side_clearance', 0.55)
        self.declare_parameter('front_sector_deg', 32.0)
        self.declare_parameter('side_sector_deg', 55.0)
        self.declare_parameter('initial_spin_sec', 7.0)
        self.declare_parameter('periodic_spin_interval_sec', 42.0)
        self.declare_parameter('periodic_spin_sec', 5.5)
        self.declare_parameter('backup_sec', 1.1)
        self.declare_parameter('turn_min_sec', 1.0)
        self.declare_parameter('turn_max_sec', 4.0)
        self.declare_parameter('stuck_window_sec', 4.0)
        self.declare_parameter('stuck_min_distance', 0.08)
        self.declare_parameter('max_runtime_sec', 0.0)

        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.turn_speed = float(self.get_parameter('turn_speed').value)
        self.backup_speed = float(self.get_parameter('backup_speed').value)
        self.forward_clearance = float(self.get_parameter('forward_clearance').value)
        self.danger_clearance = float(self.get_parameter('danger_clearance').value)
        self.side_clearance = float(self.get_parameter('side_clearance').value)
        self.front_sector = math.radians(float(self.get_parameter('front_sector_deg').value))
        self.side_sector = math.radians(float(self.get_parameter('side_sector_deg').value))
        self.initial_spin_sec = float(self.get_parameter('initial_spin_sec').value)
        self.periodic_spin_interval_sec = float(
            self.get_parameter('periodic_spin_interval_sec').value
        )
        self.periodic_spin_sec = float(self.get_parameter('periodic_spin_sec').value)
        self.backup_sec = float(self.get_parameter('backup_sec').value)
        self.turn_min_sec = float(self.get_parameter('turn_min_sec').value)
        self.turn_max_sec = float(self.get_parameter('turn_max_sec').value)
        self.stuck_window_sec = float(self.get_parameter('stuck_window_sec').value)
        self.stuck_min_distance = float(self.get_parameter('stuck_min_distance').value)
        self.max_runtime_sec = float(self.get_parameter('max_runtime_sec').value)

        self.scan = None
        self.odom_position = None
        self.odom_history = deque()
        self.start_time = time.monotonic()
        self.state = 'WAIT_SCAN'
        self.state_started = self.start_time
        self.state_deadline = None
        self.turn_direction = 1.0
        self.last_periodic_spin = self.start_time
        self.last_cmd_linear = 0.0
        self.finished = False

        self.cmd_pub = self.create_publisher(
            Twist,
            self.get_parameter('cmd_vel_topic').value,
            10,
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
        self.timer = self.create_timer(0.1, self._on_timer)

        self.get_logger().info(
            'Auto explore mapper ready. Start simulation and mapping first; this node publishes /cmd_vel.'
        )

    def _on_scan(self, msg):
        self.scan = msg

    def _on_odom(self, msg):
        pos = msg.pose.pose.position
        self.odom_position = (float(pos.x), float(pos.y))
        now = time.monotonic()
        self.odom_history.append((now, self.odom_position))
        while self.odom_history and now - self.odom_history[0][0] > self.stuck_window_sec:
            self.odom_history.popleft()

    def _on_timer(self):
        now = time.monotonic()
        if self.finished:
            self._publish_cmd(0.0, 0.0)
            return

        if self.max_runtime_sec > 0.0 and now - self.start_time >= self.max_runtime_sec:
            self.finished = True
            self.get_logger().info('Auto exploration reached max_runtime_sec; stopping robot.')
            self._publish_cmd(0.0, 0.0)
            return

        if self.scan is None:
            self.get_logger().info('Waiting for /scan before auto exploration.', throttle_duration_sec=3.0)
            self._publish_cmd(0.0, 0.0)
            return

        sectors = self._scan_sectors()

        if self.state == 'WAIT_SCAN':
            self._enter_state('SPIN_SCAN', self.initial_spin_sec)

        if self.state == 'SPIN_SCAN':
            self._spin_scan(now)
        elif self.state == 'BACKUP':
            self._backup(now)
        elif self.state == 'TURN':
            self._turn(now, sectors)
        else:
            self._cruise(now, sectors)

    def _scan_sectors(self):
        front = self._sector_min(-self.front_sector, self.front_sector)
        front_left = self._sector_min(0.20, self.side_sector)
        front_right = self._sector_min(-self.side_sector, -0.20)
        left = self._sector_min(0.85, 1.75)
        right = self._sector_min(-1.75, -0.85)
        return {
            'front': front,
            'front_left': front_left,
            'front_right': front_right,
            'left': left,
            'right': right,
        }

    def _sector_min(self, angle_min, angle_max):
        if self.scan is None or not self.scan.ranges:
            return float('inf')

        best = float('inf')
        angle = float(self.scan.angle_min)
        inc = float(self.scan.angle_increment)
        range_min = max(float(self.scan.range_min), 0.01)
        range_max = float(self.scan.range_max) if self.scan.range_max > 0.0 else float('inf')

        for value in self.scan.ranges:
            if angle_min <= angle <= angle_max and math.isfinite(value):
                distance = float(value)
                if range_min <= distance <= range_max:
                    best = min(best, distance)
            angle += inc
        return best

    def _cruise(self, now, sectors):
        if sectors['front'] < self.danger_clearance:
            self.turn_direction = self._choose_turn_direction(sectors)
            self._enter_state('BACKUP', self.backup_sec)
            return

        if self._is_stuck():
            self.turn_direction *= -1.0
            self.get_logger().warn('Robot appears stuck during mapping; backing up and changing direction.')
            self._enter_state('BACKUP', self.backup_sec)
            return

        if sectors['front'] < self.forward_clearance:
            self.turn_direction = self._choose_turn_direction(sectors)
            self._enter_state('TURN', self.turn_max_sec)
            return

        if self.periodic_spin_interval_sec > 0.0:
            if now - self.last_periodic_spin >= self.periodic_spin_interval_sec:
                self.turn_direction *= -1.0
                self._enter_state('SPIN_SCAN', self.periodic_spin_sec)
                self.last_periodic_spin = now
                return

        angular = 0.0
        if sectors['left'] < self.side_clearance:
            angular -= 0.22
        if sectors['right'] < self.side_clearance:
            angular += 0.22

        # 轻微摆动可以让 Livox 扫到更多边界结构，避免纯直线扫图过薄。
        angular += 0.06 * math.sin((now - self.start_time) / 5.0)
        self._publish_cmd(self.linear_speed, angular)

    def _backup(self, now):
        if self.state_deadline is not None and now >= self.state_deadline:
            self._enter_state('TURN', self.turn_max_sec)
            return
        self._publish_cmd(-self.backup_speed, -0.25 * self.turn_direction)

    def _turn(self, now, sectors):
        elapsed = now - self.state_started
        if elapsed >= self.turn_min_sec and sectors['front'] > self.forward_clearance:
            self._enter_state('CRUISE', None)
            return

        if self.state_deadline is not None and now >= self.state_deadline:
            self._enter_state('CRUISE', None)
            return

        self._publish_cmd(0.0, self.turn_direction * self.turn_speed)

    def _spin_scan(self, now):
        if self.state_deadline is not None and now >= self.state_deadline:
            self._enter_state('CRUISE', None)
            return
        self._publish_cmd(0.0, self.turn_direction * min(self.turn_speed, 0.55))

    def _choose_turn_direction(self, sectors):
        # 左侧更近时向右转，右侧更近时向左转。
        left_clearance = min(sectors['front_left'], sectors['left'])
        right_clearance = min(sectors['front_right'], sectors['right'])
        if left_clearance < right_clearance:
            return -1.0
        if right_clearance < left_clearance:
            return 1.0
        return -self.turn_direction

    def _is_stuck(self):
        if self.last_cmd_linear <= 0.05 or len(self.odom_history) < 2:
            return False
        oldest_time, oldest_pos = self.odom_history[0]
        newest_time, newest_pos = self.odom_history[-1]
        if newest_time - oldest_time < self.stuck_window_sec * 0.8:
            return False
        dx = newest_pos[0] - oldest_pos[0]
        dy = newest_pos[1] - oldest_pos[1]
        return math.hypot(dx, dy) < self.stuck_min_distance

    def _enter_state(self, state, duration):
        self.state = state
        self.state_started = time.monotonic()
        self.state_deadline = None if duration is None else self.state_started + duration
        self.get_logger().info(f'Auto exploration state -> {state}', throttle_duration_sec=1.0)

    def _publish_cmd(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_pub.publish(msg)
        self.last_cmd_linear = float(linear_x)


def main():
    rclpy.init()
    node = AutoExploreMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_cmd(0.0, 0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
