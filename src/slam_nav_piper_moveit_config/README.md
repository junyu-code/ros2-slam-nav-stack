# slam_nav_piper_moveit_config

Piper 项目侧 MoveIt2 配置包。它把 AgileX 官方 Piper 语义配置适配到本项目的
`piper_*` frame/joint/controller 命名，供后续 `slam_nav_piper_control` 接 MoveIt2
后端使用。

## 边界

这个包默认不被 task1、Nav2、FAST-LIO2 或仿真入口启动。它只提供显式 plan-only
验证入口：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch slam_nav_piper_moveit_config piper_project_moveit_plan.launch.py
```

默认 `allow_trajectory_execution=false`，不会尝试执行轨迹，也不会连接厂家 SDK。
如果要和现有 Gazebo 底盘模型共用 TF，请先启动仿真，再把本 launch 的
`publish_robot_state:=false`，避免重复发布 robot_state_publisher：

```bash
./run.sh sim enable_piper_arm:=true piper_arm_model:=official
ros2 launch slam_nav_piper_moveit_config piper_project_moveit_plan.launch.py publish_robot_state:=false
```

## 命名约定

官方原生命名：

```text
base_link -> link1 -> ... -> link6
joint1 ... joint8
arm / gripper
arm_controller / gripper_controller
```

项目侧命名：

```text
piper_base_link -> piper_link1 -> ... -> piper_link6 -> piper_tcp
piper_joint1 ... piper_joint8
piper_arm / piper_gripper
piper_arm_controller / piper_gripper_controller
```

`/piper/arm_camera/*` 仍只属于 Piper 机械臂相机，不进入 `/nav_camera/*`，也不作为
Nav2 costmap 默认观测源。

## 系统依赖

本 launch 不依赖 `moveit_configs_utils`，但需要 OMPL planner 插件。若要直接运行
项目侧 plan-only 和 AgileX 官方 `piper_moveit_config_v4/v5` demo，建议安装：

```bash
sudo apt-get install ros-humble-moveit-planners-ompl ros-humble-moveit-configs-utils ros-humble-moveit-ros-visualization
```

当前包的目标是先完成项目侧 plan-only 配置，不直接接入任务执行。
