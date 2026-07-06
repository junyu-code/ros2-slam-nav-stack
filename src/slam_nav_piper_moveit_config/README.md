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
./run.sh sim enable_piper_arm:=true
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

本 launch 不依赖 `moveit_configs_utils`，但需要 OMPL planner 和 simple controller
manager 插件。推荐系统安装：

```bash
sudo apt-get install ros-humble-moveit-planners-ompl ros-humble-moveit-simple-controller-manager
```

当前工作区也支持无需 sudo 的 Piper 专用本地 overlay：

```bash
./run.sh setup-piper-moveit
./run.sh piper-preflight
./run.sh piper-moveit-plan
```

## 静态审计

在启动 MoveIt2 前，可以先检查项目侧配置是否仍与 AgileX 官方 v5 配置保持映射一致：

```bash
./run.sh piper-moveit-config
```

该脚本会读取官方 `piper_description` 渲染项目侧 `piper_*` URDF，确认没有退回占位关节；随后检查 `piper.srdf`、`joint_limits.yaml`、`ros2_controllers.yaml`、`moveit_controllers.yaml`、`initial_positions.yaml` 和假关节状态发布器。默认会对照 `piper_moveit_config_v5`，确保官方 `arm/gripper`、`joint1...joint8`、`arm_controller/gripper_controller` 已正确映射为项目侧 `piper_arm/piper_gripper`、`piper_joint1...piper_joint8`、`piper_arm_controller/piper_gripper_controller`。

启动 `piper-moveit-plan` 后，可另开终端做一次规划服务冒烟测试：

```bash
./run.sh piper-plan-test
```

该测试调用 `/piper/plan_kinematic_path`，只验证 `piper_arm` 关节目标规划能返回非空轨迹；默认不执行轨迹，也不连接 SDK。

也可以一条命令完成启动、等待服务、发送规划请求和清理本次测试进程：

```bash
./run.sh piper-moveit-smoke
```

AgileX 官方 `piper_moveit_config_v4/v5` demo wrapper 还需要：

```bash
sudo apt-get install ros-humble-moveit-configs-utils ros-humble-moveit-ros-visualization
```

当前包的目标是先完成项目侧 plan-only 配置，不直接接入任务执行。
