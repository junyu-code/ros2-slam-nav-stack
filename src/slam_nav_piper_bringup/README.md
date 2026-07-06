# slam_nav_piper_bringup

Piper 移动操作扩展的独立启动入口。这里的 launch 不会被 `start_mapping.sh`、`start_navigation.sh` 或 `start_navigation_3d.sh` 默认调用。

## 仿真/冒烟

```bash
ros2 launch slam_nav_piper_bringup piper_sim.launch.py
```

该入口启动占位 TF、假腕部 RGB-D 相机、目标位姿估计、控制桥和 fake 抓取/放置 action。

注意：MoveIt2 规划接口已经安装，但当前 `piper_sim.launch.py` 仍默认使用 fake 执行后端。这样可以先验证 `/piper` 命名空间、TF、相机和 action 链路，不会误动真实机械臂。

## 与现有导航并行

先按原流程启动仿真和导航，再单独启动：

```bash
ros2 launch slam_nav_piper_bringup piper_mobile_manipulation.launch.py fake_camera:=true
```

Piper 相机保持在 `/piper/arm_camera/*`，不会 remap 到 `/nav_camera/*`。

## 实机边界

```bash
ros2 launch slam_nav_piper_bringup piper_real.launch.py backend:=moveit
```

实机默认 `auto_enable=false`，启动后需要先验证 CAN/SDK、急停、失能、home 和限速，再逐步执行 MoveIt2 plan-only、低速预抓取、完整 pick/place。

后续接真实 MoveIt2 时，需要补齐 Piper 的 URDF/SRDF、planning groups、controller 配置和 `move_group` 启动文件。`slam_nav_piper_bringup` 只负责组合这些入口，不应把 Piper 相机或抓取结果接入 Nav2 默认 costmap。
