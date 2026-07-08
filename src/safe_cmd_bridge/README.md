# safe_cmd_bridge

`safe_cmd_bridge` 是一个通用速度安全桥，用在导航控制器和仿真/真实底盘之间。

它订阅原始速度指令，输出经过安全处理后的速度指令，也可以选择把速度通过 UDP 发给外部控制进程。

## 功能

- 限制 `vx`、`vy`、`wz` 的最大/最小速度。
- 限制线速度和角速度的加速度、减速度。
- 过滤很小的速度死区。
- 输入超时后自动平滑降到零速度。
- 可订阅 `/localization_fault`，定位健康异常时强制安全速度降到零。
- 退出节点时主动发送零速度。
- 支持方向反转参数，便于适配不同底盘坐标约定。
- 支持 ROS topic 输出、UDP 输出或两者同时输出。

## 仿真调试

默认只启用 ROS topic 输出：

```bash
ros2 launch safe_cmd_bridge safe_cmd_bridge.launch.py
```

输入 `/cmd_vel`，输出 `/cmd_vel_safe`。

如果同时启动了 `localization_guard`，安全桥会默认订阅 `/localization_fault`。当定位健康监控发布 `true` 时，安全桥会忽略原始速度目标，按减速度限制输出零速度。

手动测试限速：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 2.0, y: 1.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 3.0}}"
```

查看输出：

```bash
ros2 topic echo /cmd_vel_safe
```

## 实机部署思路

真实底盘部署时，建议先保持 `enable_udp_output:=false` 做 dry-run 观察；确认速度方向和幅值正确后，再打开 UDP 输出。

```bash
ros2 launch safe_cmd_bridge safe_cmd_bridge.launch.py \
  input_topic:=/cmd_vel \
  output_topic:=/cmd_vel_safe \
  enable_fault_stop:=true \
  enable_udp_output:=true \
  udp_host:=192.168.123.22 \
  udp_port:=15000
```

这层模块只负责速度安全整形和转发，不直接绑定某个底盘 SDK。具体底盘侧可以用一个独立进程接收 UDP，再调用对应运动接口。

## Unitree GO2

Unitree GO2 联调时可以直接使用本包的 UDP 输出，不需要额外 ROS2 sender：

```bash
ros2 launch safe_cmd_bridge safe_cmd_bridge.launch.py \
  input_topic:=/cmd_vel \
  output_topic:=/cmd_vel_safe \
  enable_fault_stop:=true \
  enable_udp_output:=true \
  udp_host:=192.168.123.22 \
  udp_port:=15000 \
  max_vx:=0.2 max_vy:=0.1 max_wz:=0.3
```

发送 payload 为 ASCII：

```text
vx,vy,wz\n
```

GO2 侧接收器应把这三个值对应到 `SportClient.Move(vx, vy, vyaw)`。打开 UDP 前先保持 `enable_udp_output:=false` 观察 `/cmd_vel_safe`，确认限速、超时停车、方向和急停条件都正确。
