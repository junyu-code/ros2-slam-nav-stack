# slam_nav_piper_bringup

Piper 移动操作扩展的独立启动入口。这里的 launch 不会被 `start_mapping.sh`、`start_navigation.sh` 或 `start_navigation_3d.sh` 默认调用。

## 仿真/冒烟

```bash
ros2 launch slam_nav_piper_bringup piper_sim.launch.py
```

该入口启动占位 TF、假腕部 RGB-D 相机、目标位姿估计、控制桥和 fake 抓取/放置 action。

安装并构建 AgileX 官方 `piper_description` 后，可以显式用官方 URDF 适配链替换占位 TF：

```bash
ros2 launch slam_nav_piper_bringup piper_sim.launch.py arm_model:=official
```

注意：MoveIt2 规划接口已经安装，但当前 `piper_sim.launch.py` 仍默认使用 fake 执行后端。这样可以先验证 `/piper` 命名空间、TF、相机和 action 链路，不会误动真实机械臂。

项目侧 MoveIt2 plan-only 单独启动：

```bash
./run.sh piper-moveit-plan
```

该入口来自 `slam_nav_piper_moveit_config`，使用 `piper_*` frame/joint/group/controller 命名和假关节状态发布器。默认 `allow_trajectory_execution=false`，不接 SDK、不执行轨迹。当前机器缺 `ros-humble-moveit-planners-ompl` 时，预检会失败，安装后再跑该入口：

```bash
sudo apt-get install ros-humble-moveit-planners-ompl
```

## 预检

```bash
ros2 run slam_nav_piper_bringup piper_preflight_check.py
```

要求官方 AgileX Piper 包也必须存在时：

```bash
ros2 run slam_nav_piper_bringup piper_preflight_check.py --require-official
```

官方包导入后，先审计 URDF/Xacro 的根 frame 和 TCP frame：

```bash
ros2 run slam_nav_piper_bringup piper_official_frame_audit.py
```

## 配置文件

```text
config/piper_mobile_manipulation.yaml  # 总体边界和默认开关
config/piper_external_dependencies.yaml # AgileX/MoveIt2 外部依赖记录
config/piper_moveit2_boundary.yaml      # MoveIt2 接入前的规划边界
config/piper_safety_limits.yaml         # 实机安全限制和验收项
config/piper_learning_boundary.yaml     # 后续强化学习旁路策略边界
```

这些配置只做预留，不会自动接入 task1，也不会启动真实 MoveIt2、SDK 或学习策略。

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

实机默认 `auto_enable=false` 且 `real_backend_connected=false`。启动后需要先验证 CAN/SDK、急停、失能、home 和限速，再把 `real_backend_connected:=true` 作为显式接入声明，逐步执行 MoveIt2 plan-only、低速预抓取、完整 pick/place。

后续接真实 MoveIt2 时，需要确认使用哪套 frame 名：官方 demo 原生使用 `base_link/link1.../joint1...`，移动底盘组合模型使用项目侧 `piper_base_link/piper_link1.../piper_joint1...`。当前已提供项目侧 `slam_nav_piper_moveit_config` 做 plan-only；真实执行后端仍需在 `slam_nav_piper_control` 内部适配。`slam_nav_piper_bringup` 只负责组合这些入口，不应把 Piper 相机或抓取结果接入 Nav2 默认 costmap。

## 外部依赖

根目录的 `piper_external.repos` 记录 AgileX 相关外部仓库。需要时手动拉取：

```bash
cd ~/slam_nav_ws
mkdir -p external
vcs import external < piper_external.repos
```

`external/` 不进 Git，避免把厂家仓库和后续仿真资产塞进主仓库。

已记录的 AgileX Piper 课程参考：

```text
https://github.com/agilexrobotics/agilex_open_class/tree/master/piper
```

该目录里的 `piper_description`、`piper_moveit_config_v4`、`piper_moveit_config_v5` 作为官方模型和 MoveIt2 示例来源；当前 bringup 不会默认启动它们。

导入并构建官方包后，可以单独跑官方 demo wrapper：

```bash
cd ~/slam_nav_ws
./run.sh setup-piper
source install/setup.bash
```

`./run.sh setup-piper` 默认只下载 `piper_description`、`piper_moveit_config_v4`、`piper_moveit_config_v5`，下载目标在 `external/`，支持中断后续跑。遇到 GitHub API rate limit 时可等待后重跑，或显式自动等待：

```bash
PIPER_OPEN_CLASS_WAIT_RATE_LIMIT=1 ./run.sh setup-piper
```

当前工作区已补齐官方 `piper_description` 的 `base_link.STL` 和 `link1.STL` 到 `link8.STL`；预检应显示 AgileX open class 下载目录 66 个文件。

```bash
ros2 launch slam_nav_piper_bringup piper_official_moveit_demo.launch.py
ros2 launch slam_nav_piper_bringup piper_official_gazebo_demo.launch.py
ros2 launch slam_nav_piper_bringup piper_official_gazebo_demo.launch.py start_moveit:=true
```

这些入口只用于对照官方模型、MoveIt2 配置和 Gazebo 控制器，不接入 task1，也不让 `mission_behavior` 直接依赖官方话题。

底盘组合仿真中使用官方 URDF 适配链：

```bash
./run.sh sim enable_piper_arm:=true piper_arm_model:=official
```

## 学习层预留

学习/强化学习先单独运行，不被本包默认 include：

```bash
ros2 launch slam_nav_piper_learning piper_learning.launch.py enable_learning:=true policy_backend:=heuristic
```

任务层默认不消费 `/piper/learning/grasp_candidates_ranked`。
