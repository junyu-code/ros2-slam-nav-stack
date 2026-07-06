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

控制边界烟测：

```bash
./run.sh piper-control-smoke
```

该脚本会在独立 `ROS_DOMAIN_ID` 下启动控制桥，检查服务发现、enable、owner 切换、estop、clear_estop 和 disable。它不连接 SDK、不执行 MoveIt2 轨迹。

实机入口默认拒绝验收：

```bash
./run.sh piper-real-dry-run
```

该脚本从 `piper_real.launch.py` 启动完整边界，保持 `real_backend_connected=false`，确认 `home`、pick/place 不会绕过控制桥和真实后端声明。

## 已预留参数

`config/piper_control.yaml` 已预留：

```text
moveit_config_package: slam_nav_piper_moveit_config
moveit_planning_group: piper_arm
moveit_tcp_frame: piper_tcp
sdk_driver_package
sdk_driver_namespace
allow_real_motion
velocity_scaling
workspace_min_xyz / workspace_max_xyz
```

默认 `allow_real_motion=false`、`auto_enable=false`、`initial_owner=disabled`。当前 MoveIt2 只配置到项目侧 plan-only 包，控制桥还不会执行真实轨迹。后续即使接入 MoveIt2 或 SDK，也应先在控制桥内部完成限速、工作空间、急停和 owner 检查。
