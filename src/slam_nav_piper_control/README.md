# slam_nav_piper_control

Piper 控制边界包。它的职责是把项目侧接口和 MoveIt2/厂家 SDK 解耦，避免上层任务直接依赖厂家话题名。

## MoveIt2 状态

当前系统已安装 MoveIt2 规划接口：

```text
ros-humble-moveit-ros-planning-interface 2.5.9
```

验证：

```bash
ros2 pkg prefix moveit_ros_planning_interface
ros2 pkg prefix moveit_ros_move_group
ros2 pkg prefix moveit_core
```

本包当前仍是控制边界占位实现，不直接调用 MoveIt2。后续应在本包内部增加 MoveIt2 后端适配，保持 `/piper/control/*` 和 `/piper/task/*` 项目侧接口稳定。

## 控制 owner

同一时刻只允许一个 owner：

```text
moveit
sdk_test
disabled
```

切换方式：

```bash
ros2 topic pub --once /piper/control/owner_request std_msgs/msg/String "{data: moveit}"
```

安全服务：

```text
/piper/control/enable
/piper/control/disable
/piper/control/estop
/piper/control/clear_estop
/piper/control/home
```

当前节点是后端边界占位实现，不直接控制真实机械臂。接入 AgileX `agx_arm_ros` 或 MoveIt2 后，应在本包内部适配，保持 `/piper/task/*` 上层接口不变。

## 已预留参数

`config/piper_control.yaml` 已预留：

```text
moveit_config_package
moveit_planning_group
moveit_tcp_frame
sdk_driver_package
sdk_driver_namespace
allow_real_motion
velocity_scaling
workspace_min_xyz / workspace_max_xyz
```

默认 `allow_real_motion=false`、`auto_enable=false`、`initial_owner=disabled`。后续即使接入 MoveIt2 或 SDK，也应先在控制桥内部完成限速、工作空间、急停和 owner 检查。
