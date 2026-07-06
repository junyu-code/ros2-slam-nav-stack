# slam_nav_piper_bringup

Piper 移动操作扩展的独立启动入口。这里的 launch 不会被 `./run.sh mapping`、`./run.sh nav` 或 `./run.sh nav-3d` 默认调用。

## 仿真/冒烟

全链路烟测：

```bash
./run.sh piper-full-smoke
```

该入口会顺序跑 `piper-safety-check`、`piper-boundary-check`、`piper-size-check`、`piper-preflight --require-official`、官方 frame audit、`piper-moveit-config`、`piper-hand-eye-check`、`piper-hand-eye-gate`、`piper-tf-smoke`、`piper-namespace-smoke`、`piper-control-smoke`、`piper-real-dry-run`、`piper-gazebo-smoke`、`piper-task-smoke`、`piper-mobile-sequence`、`piper-mission-demo`、`piper-learning-smoke` 和 `piper-moveit-smoke`。它用于确认当前 Piper 扩展从安全默认值、task1 隔离边界、仓库体积边界、官方 URDF/MoveIt2 映射、手眼标定配置边界、真实 pick 标定门禁、运行时 TF、runtime 命名空间、控制安全、实机默认拒绝、模型、假感知、任务 action、移动操作组合入口、mission 行为层 action 边界、学习排序到 MoveIt2 plan-only 都是通的。

只检查安全默认值，不启动 ROS 节点：

```bash
./run.sh piper-safety-check
```

只检查边界，不启动 Gazebo/MoveIt2：

```bash
./run.sh piper-boundary-check
```

只检查运行时 TF 链和 task1 TF 隔离：

```bash
./run.sh piper-tf-smoke
```

该入口会查询 `base_link -> piper_base_link`、`piper_base_link -> piper_tcp`、`piper_tcp -> piper_arm_camera_optical_frame`，并确认独立 Piper TF 图没有发布 `map -> odom`、`odom -> base_footprint` 或 `nav_camera` frame。

只检查 runtime topic/action/node 命名空间：

```bash
./run.sh piper-namespace-smoke
```

该入口会启动 `piper_sim`，确认 Piper 相机、感知、抓取候选、控制状态和 task action 都在 `/piper` 下，并确认没有 `/nav_camera`、costmap、Nav2 action 或 AMCL/Nav2 节点。

```bash
ros2 launch slam_nav_piper_bringup piper_sim.launch.py
```

该入口默认使用 AgileX 官方 Piper URDF 适配链，并启动假腕部 RGB-D 相机、目标位姿估计、控制桥和 fake 抓取/放置 action。独立仿真默认发布 Piper 假关节状态，保证 `piper_base_link -> piper_arm_camera_optical_frame` TF 可用于目标位姿估计；实机入口不启用这个假关节状态发布器。

想直接“看到东西”，使用 RViz 可视化入口：

```bash
./run.sh piper-viz
```

该入口默认启动 `piper_sim.launch.py` 并打开 `config/piper_visualization.rviz`，可查看官方 Piper 适配链 RobotModel、TF、假腕部 RGB 图和 `/piper/perception/target_pose`。它不启动 Nav2、不接 SDK、不执行 MoveIt2 轨迹。需要同时看 MoveIt2 plan-only 服务时可显式加：

```bash
./run.sh piper-viz start_moveit_plan:=true
```

此时仍保持 `allow_trajectory_execution=false`，只用于观察规划服务和 RViz 模型。

缺少官方包、只想先检查 `/piper` 话题/action 边界时，可以显式退回占位 TF：

```bash
ros2 launch slam_nav_piper_bringup piper_sim.launch.py arm_model:=placeholder
```

注意：MoveIt2 规划接口已经安装，但当前 `piper_sim.launch.py` 仍默认使用 fake 执行后端。这样可以先验证 `/piper` 命名空间、TF、相机和 action 链路，不会误动真实机械臂。

一键 fake 任务链路烟测：

```bash
./run.sh piper-task-smoke
```

该入口会等待 Piper 假 RGB-D、目标位姿、抓取候选和 `/piper/task/*` action server，然后发送一次 fake pick/place goal。它不启动 Nav2、不执行真实轨迹、不连接 SDK。当前无 GUI 冒烟已验证通过。

移动操作组合入口烟测：

```bash
./run.sh piper-mobile-sequence
```

该入口启动 `piper_mobile_manipulation.launch.py` 的独立 fake runtime，显式打开 `publish_base_stop:=true`，确认假相机、目标位姿、抓取候选、停车零速度 `/cmd_vel`、pick/place action 和控制 owner 切换都能按顺序工作。它不启动 Nav2，也不连接真实 MoveIt2 执行器或 SDK。

mission_behavior 到 Piper action 边界烟测：

```bash
./run.sh piper-mission-demo
```

该入口会启动 Piper fake runtime，再启动 `mission_behavior` 的 `piper_pick_place_demo.launch.py`。它验证上层任务包只通过 `/piper/task/*` action 调用 Piper，不直接依赖 MoveIt2、SDK 或厂家话题。

控制桥安全边界烟测：

```bash
./run.sh piper-control-smoke
```

该入口验证 `/piper/control/*` 服务、`moveit/disabled` owner、急停后拒绝 owner 切换和急停中拒绝 enable。它只检查边界状态机，不接真实执行后端。

实机入口 dry-run：

```bash
./run.sh piper-real-dry-run
```

该入口启动 `piper_real.launch.py`，保持 `real_backend_connected=false`，确认默认禁用状态下 `home` 失败，pick/place action 返回安全拒绝。它不连接 CAN、SDK 或真实 MoveIt2 执行器。

项目侧 MoveIt2 plan-only 单独启动：

```bash
./run.sh piper-moveit-plan
```

该入口来自 `slam_nav_piper_moveit_config`，使用 `piper_*` frame/joint/group/controller 命名和假关节状态发布器。默认 `allow_trajectory_execution=false`，不接 SDK、不执行轨迹。

MoveIt2 配置映射审计：

```bash
./run.sh piper-moveit-config
```

该入口不启动 ROS 节点，只检查官方 Piper URDF 适配链、项目侧 SRDF/YAML 和 AgileX 官方 `piper_moveit_config_v5` 的映射一致性，防止后续维护时误回退到占位关节或官方原生 `base_link/link*/joint*` 名称。

手眼标定配置边界检查：

```bash
./run.sh piper-hand-eye-check
```

该入口不启动相机驱动，也不移动机械臂；只检查 eye-in-hand 标定默认配置是否仍使用 `/piper/arm_camera/*`、`piper_base_link`、`piper_tcp`、`piper_arm_camera_optical_frame`，并确认结果/样本进入 `datasets/piper_hand_eye/` 而不是 `src/`。

真实 pick 手眼标定门禁烟测：

```bash
./run.sh piper-hand-eye-gate
```

该入口只启动控制桥和任务层，不启动相机、Gazebo、MoveIt2 执行器或 SDK。它模拟“真实后端已声明接入，但手眼标定尚未验收”的危险状态，确认 pick action 必须安全拒绝。

推荐系统安装：

```bash
sudo apt-get install ros-humble-moveit-planners-ompl ros-humble-moveit-simple-controller-manager
```

没有 sudo 时可用 Piper 专用本地 overlay：

```bash
./run.sh setup-piper-moveit
./run.sh piper-preflight
```

一键启动 MoveIt2 plan-only 并发送一次规划请求：

```bash
./run.sh piper-moveit-smoke
```

该测试只验证 `/piper/plan_kinematic_path`、`piper_arm` planning group 和 OMPL 规划链路能返回非空轨迹，不执行轨迹、不连接 SDK。

学习层候选排序烟测：

```bash
./run.sh piper-learning-smoke
```

该入口只启动学习排序旁路，发布假抓取候选并检查 ranked 输出顺序。任务层默认仍不消费 `/piper/learning/grasp_candidates_ranked`。

仓库体积边界检查：

```bash
./run.sh piper-size-check
```

该入口确认 Piper 外部依赖、训练数据、模型权重、checkpoint、rosbag 和点云产物没有进入 Git 跟踪；已有非 Piper 历史大文件只提示 warning。

轻量静态配置验收：

```bash
./run.sh piper-static-check
```

该入口串联安全默认值、task1/Nav2 隔离、体积边界、依赖预检、官方 frame audit、MoveIt2 配置映射、手眼标定边界、实机接入前状态报告和 launch 参数展开检查。它不启动 Gazebo、Nav2、MoveIt2 `move_group` 或真实硬件，适合每次修改 Piper 配置后先跑。

实机接入前状态报告：

```bash
./run.sh piper-real-readiness
./run.sh piper-real-readiness --require-ready
```

默认模式会把当前未接入项列为 `WAIT` 并返回成功，适合查看还差什么；`--require-ready` 会把这些 `WAIT` 作为失败，用于真正准备打开真实后端前的门禁。

## 预检

```bash
ros2 run slam_nav_piper_bringup piper_preflight_check.py
./run.sh piper-preflight
```

`./run.sh piper-preflight` 会自动加载 `external/ros_humble_debs/overlay` 中的本地 MoveIt2 插件。

要求官方 AgileX Piper 包也必须存在时：

```bash
ros2 run slam_nav_piper_bringup piper_preflight_check.py --require-official
```

官方包导入后，先审计 URDF/Xacro 的根 frame 和 TCP frame：

```bash
./run.sh piper-official-frame-audit
ros2 run slam_nav_piper_bringup piper_official_frame_audit.py
```

`./run.sh piper-official-frame-audit` 默认追加 `--check-project-adapter`，会确认官方 Piper 关节链已经被适配为 `piper_base_link/piper_link*/piper_joint*`，并补齐 `piper_tcp` 与腕部相机 frame。

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

该组合入口默认 `start_description:=false`，假设整车仿真或真实机器人已经在发布 `robot_state_publisher`，避免重复 TF。若要脱离 Gazebo 单独冒烟，可显式打开：

```bash
ros2 launch slam_nav_piper_bringup piper_mobile_manipulation.launch.py start_description:=true publish_joint_states:=true fake_camera:=true
```

Piper 相机保持在 `/piper/arm_camera/*`，不会 remap 到 `/nav_camera/*`。

## 实机边界

```bash
ros2 launch slam_nav_piper_bringup piper_real.launch.py backend:=moveit
```

实机默认 `auto_enable=false` 且 `real_backend_connected=false`。启动后需要先验证 CAN/SDK、急停、失能、home 和限速，再把 `real_backend_connected:=true` 作为显式接入声明，逐步执行 MoveIt2 plan-only、低速预抓取、完整 pick/place。

默认拒绝路径可先自动验证：

```bash
./run.sh piper-real-dry-run
```

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

该目录里的 `piper_description`、`piper_moveit_config_v4`、`piper_moveit_config_v5` 作为官方模型和 MoveIt2 示例来源；Piper 项目侧入口默认读取官方 `piper_description`，但仍不会把官方 demo、MoveIt2 执行控制器或 Gazebo 控制器接入 task1。

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
./run.sh sim enable_piper_arm:=true
```

`enable_piper_arm` 默认仍是 `false`，因此 task1 默认仿真不变。只有显式打开 Piper 时，默认才使用官方适配链；需要占位模型时可传 `piper_arm_model:=placeholder`。

headless 自动烟测：

```bash
./run.sh piper-gazebo-smoke
```

该入口会在独立 `ROS_DOMAIN_ID` 和 `GAZEBO_MASTER_URI` 下启动静态场地、官方 Piper 适配链和 Gazebo server，确认 `/robot_description` 没有落回占位关节，并检查 `mobile_robot` 已经 spawn。它不启动 Nav2、MoveIt2 执行或厂家 SDK。

## 学习层预留

学习/强化学习先单独运行，不被本包默认 include：

```bash
ros2 launch slam_nav_piper_learning piper_learning.launch.py enable_learning:=true policy_backend:=heuristic
./run.sh piper-learning-smoke
```

任务层默认不消费 `/piper/learning/grasp_candidates_ranked`。

## 实机前状态报告

Piper 上实机前先使用无 GUI、无真实运动的静态检查：

```bash
./run.sh piper-static-check
./run.sh piper-real-readiness
```

`piper-static-check` 会顺序检查安全默认值、task1/Nav2 隔离边界、仓库体积边界、官方包依赖、URDF frame 映射、MoveIt2 配置、手眼标定边界、实机 readiness 报告和 launch 参数展开。`piper-real-readiness` 会单独输出 OK/WAIT/FAIL 三类状态；默认 WAIT 不算失败，用来确认当前仍处于“配置安全、真实执行未接入/未验收”的阶段。真正上机前再使用 `./run.sh piper-real-readiness --require-ready`，把 WAIT 也视为失败。
