# localization_guard

`localization_guard` 是定位健康监控包，用于在运行时检查 SLAM/导航链路是否出现明显异常。

它默认只发布状态，不接管控制；实机部署时可以打开 `publish_zero_on_fault`，在定位输入异常时向 `/cmd_vel` 发布零速度。

## 监控内容

- `/Odometry` 是否断流。
- `/cloud_registered` 是否断流。
- `/scan` 是否断流。
- 里程计线速度、角速度是否超过阈值。
- 里程计位姿或航向角是否发生明显跳变。
- AMCL 首次收敛后，`/scan` 与二维地图的残差是否持续超限。
- 地图一致性质量话题是否在启用监控后断流。

地图一致性检查订阅 `/amcl_convergence_status`。它只在首次收到
`converged=true` 后启用，避免把 AMCL 启动收敛过程误判为慢漂。

## 输出

```text
/localization_health  # JSON 文本状态
/localization_fault   # Bool，true 表示当前存在定位健康故障
/diagnostics          # diagnostic_msgs/DiagnosticArray
```

## 启动

```bash
cd ~/slam_nav_ws
./run.sh guard
```

默认不发零速度，只监控：

```bash
ros2 launch localization_guard localization_guard.launch.py
```

实机保守模式：

```bash
ros2 launch localization_guard localization_guard.launch.py publish_zero_on_fault:=true
```

## 常用检查

```bash
ros2 topic echo /localization_health
ros2 topic echo /localization_fault
ros2 topic echo /diagnostics
```

这个包不替代重定位算法。它的作用是尽早发现“输入断流、定位跳变、速度异常、
持续偏离二维地图”等问题，并为后续地图辅助重定位或安全停车提供统一状态入口。
