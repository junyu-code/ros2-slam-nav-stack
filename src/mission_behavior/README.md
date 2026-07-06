# mission_behavior

`mission_behavior` 是任务层行为树包，放在当前 `slam_nav_ws` 中，作为导航、感知、语义和机械臂之间的上层调度入口。

当前版本先实现一个轻量可运行骨架：

```text
等待 Nav2 接口
-> 发送 NavigateToPose
-> 如果导航失败
   -> 读取 local costmap 和 Odometry
   -> 选择后方、左后、右后、左、右中更空的脱困方向
   -> 短距离发布 cmd_vel 脱离障碍膨胀区
   -> 若自由空间恢复不可用，则回退到 Nav2 BackUp
   -> 等待 0.5 s
   -> 再次发送 NavigateToPose
```

它不替代 `slam_nav_bringup` 中 Nav2 自带的行为树，而是作为未来“任务大脑”的独立包。后续可以继续接入语义任务、RGB-D 识别和机械臂动作。

## 运行方式

先启动仿真和导航：

```bash
cd ~/slam_nav_ws
./run.sh clean
./run.sh sim-static
./run.sh nav
```

确认 Nav2 active 后，另开终端运行任务行为树：

```bash
cd ~/slam_nav_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch mission_behavior mission_behavior.launch.py auto_start:=true goal_x:=8.0 goal_y:=4.0 goal_yaw:=0.0
```

默认恢复策略为 `free_space`：

```bash
ros2 launch mission_behavior mission_behavior.launch.py \
  auto_start:=true \
  goal_x:=8.0 goal_y:=4.0 goal_yaw:=0.0 \
  recovery_strategy:=free_space
```

如果只想使用 Nav2 标准后退恢复，可以切回固定后退：

```bash
ros2 launch mission_behavior mission_behavior.launch.py \
  auto_start:=true \
  goal_x:=8.0 goal_y:=4.0 goal_yaw:=0.0 \
  recovery_strategy:=backup
```

自由空间恢复依赖：

```text
/local_costmap/costmap
/Odometry
/cmd_vel
```

它只在任务层导航失败后短时间接管速度输出，正常跟踪路径时仍由 Nav2 控制。

## Piper 任务 demo

Piper 机械臂扩展保持独立 `/piper` 命名空间。`mission_behavior` 只通过项目侧 action 调用 Piper，不直接依赖 MoveIt2、SDK 或厂家话题：

```bash
ros2 launch mission_behavior piper_pick_place_demo.launch.py auto_start:=true
```

该 demo 会顺序调用 `/piper/task/pick_object` 和 `/piper/task/place_object`。它不启动 Nav2、不接真实机械臂，也不会修改 task1 默认入口；需要先单独启动 Piper fake runtime，例如：

```bash
./run.sh piper-sim
```

自动验收入口：

```bash
./run.sh piper-mission-demo
```

## 文件说明

```text
scripts/mission_behavior_node.py
  可运行的任务行为树节点。

scripts/mission_piper_pick_place_demo_node.py
  只调用 /piper/task/* action 的 Piper pick/place 调度示例。

behavior_tree/mission_navigation_recovery.xml
  行为树结构示意，后续可迁移为 BehaviorTree.CPP 插件版本。

config/mission_behavior.yaml
  默认参数，包含自由空间恢复的采样距离、走廊宽度、占用阈值和速度。

launch/mission_behavior.launch.py
  启动入口。

launch/piper_pick_place_demo.launch.py
  Piper action 边界 demo 启动入口。
```

## 项目命名建议

当前仓库名 `ros2-slam-nav-stack` 仍然合适，因为行为树是导航系统上的任务层扩展。等系统真正扩展到语义理解、视觉抓取和机械臂移动操作后，再考虑改成更宽的名字，例如 `ros2-mobile-robot-autonomy`。
