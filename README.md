# Ubuntu 22.04 ROS2 SLAM Navigation System

这是一个面向 Ubuntu 22.04 + ROS2 Humble + Gazebo Classic 的通用移动机器人 SLAM 与自主导航工作区。当前主目标是稳定完成仿真建图、保存地图、加载地图导航、目标点到达和静态避障验证。

项目长期会继续扩展 RGB-D 深度相机、语义识别、行为树和机械臂，但这些内容不作为当前结课作业的主流程。

## 工作区结构

```text
slam_nav_ws/
  src/
    slam_nav_simulation/       # Gazebo 场地、机器人模型、仿真启动
    slam_nav_bringup/          # 建图、导航参数和启动入口
    FAST_LIO/                  # FAST-LIO2 风格 LiDAR-IMU 定位建图前端
    pointcloud_to_laserscan/   # 3D 点云转 2D LaserScan
    cloud_relocalization/      # PCD 地图辅助重定位，默认只观测不接管 TF
    ros2_livox_simulation/     # Livox Mid-360 仿真插件
    imu_complementary_filter/  # IMU 姿态滤波工具
    perception_adapter/        # 后续部署阶段的松耦合感知适配接口
    terrain_analysis/          # LiDAR 一阶段滚动地形分析，输出 /terrain_map
    terrain_analysis_ext/      # LiDAR 二阶段扩展地形分析，输出 /terrain_map_ext
    pb_nav2_plugins/           # Nav2 强度体素层与自由空间后退恢复行为
    pb_omni_pid_pursuit_controller/ # 全向底盘 PID 路径跟踪控制器
    mission_behavior/          # 任务层行为树入口，用于导航失败恢复和后续语义/机械臂调度
    slam_nav_piper_interfaces/  # Piper 移动操作项目侧 msg/action 接口
    slam_nav_piper_description/ # Piper 底盘挂载、腕部相机和占位 TF 描述
    slam_nav_piper_perception/  # Piper 独立 RGB-D 感知，使用 /piper/arm_camera/*
    slam_nav_piper_control/     # Piper MoveIt2/SDK 控制边界和安全 owner 管理
    slam_nav_piper_manipulation/# Piper pick/place 任务 action server
    slam_nav_piper_bringup/     # Piper 独立启动入口，不参与 task1 默认链路
    safe_cmd_bridge/           # 通用速度安全桥，用于限速、限加速度、超时停车、反馈看门狗和可选 UDP 转发
    localization_guard/        # 定位健康监控，用于检测断流、跳变和速度异常
  tasks/
    task0/                     # FAST-LIO2、Point-LIO 等学习笔记
    task1/                     # 当前课程作业材料和交付说明
    task2/                     # 后续长期扩展路线，如视觉、行为树、机械臂
```

## 构建

```bash
cd ~/slam_nav_ws
./build.sh
source install/setup.bash
```

## 主流程 1：建图

建图阶段只使用原始稳定链路，不接入 `perception_adapter`。

终端 1 启动仿真：

```bash
cd ~/slam_nav_ws
./start_simulation.sh
```

WSL 中 Gazebo Classic 有时启动较慢，看到下面日志后再继续启动建图或导航更稳：

```text
SpawnEntity: Successfully spawned entity [mobile_robot]
```

终端 2 启动 FAST-LIO2、点云转 `/scan`、slam_toolbox：

```bash
cd ~/slam_nav_ws
./start_mapping.sh
```

终端 3 键盘控制机器人探索场地：

```bash
cd ~/slam_nav_ws
./teleop.sh
```

保存地图：

```bash
cd ~/slam_nav_ws
./save_map.sh nav_test_map
```

保存结果应位于：

```text
src/slam_nav_bringup/map/nav_test_map.yaml
src/slam_nav_bringup/map/nav_test_map.pgm
```

## 主流程 2：加载地图导航

导航阶段不需要继续运行 `start_mapping.sh`。先清理旧进程，再启动仿真和导航。

终端 1：

```bash
cd ~/slam_nav_ws
./clean.sh
./start_simulation.sh
```

终端 2：

```bash
cd ~/slam_nav_ws
./start_navigation.sh
```

当前 `start_navigation.sh` 会启动：

```text
FAST-LIO2
pointcloud_to_laserscan -> /scan
Nav2 bringup
map_server 加载 nav_test_map.yaml
publish_initial_pose.py 初始化 AMCL
RViz
```


当前 Nav2 规划链路：

```text
全局规划：planner_server / GridBased / SmacPlanner2D
局部路径跟踪：controller_server / FollowPath / OmniPidPursuitController
局部障碍物输入：local_costmap / obstacle_layer / /scan
全局静态地图与障碍物输入：global_costmap / static_layer + obstacle_layer
速度平滑：velocity_smoother
恢复行为：behavior_server + 行为树 BackUp/ClearCostmap/Spin
```

由于仿真底盘使用 `libgazebo_ros_planar_move.so`，它支持平面全向速度。当前 3D 导航参数已从通用 DWB 轨迹采样切换为泛化后的 `pb_omni_pid_pursuit_controller::OmniPidPursuitController`：控制器根据全局路径前瞻点输出 x/y 平面速度，保留接近目标减速、曲率限速，并在“命令速度较大但里程计实际速度很小”时短时后退脱困。该控制器已经整理为通用全向底盘路径跟踪模块。
导航行为树已经加入拥挤场景恢复策略：

```text
规划/跟踪失败
  -> 清理局部与全局代价地图
  -> 后退 0.45 m 脱离障碍物膨胀区
  -> 短暂等待并重新规划
  -> 必要时旋转恢复
```

这是 Nav2 默认导航链路中的轻量恢复配置。任务层 `mission_behavior` 已进一步加入基于 local costmap 的自由空间脱困恢复：导航失败后优先选择更空的后退/侧移方向短距离脱困，若局部代价地图或里程计数据不可用，再回退到 Nav2 标准 `BackUp`。

当前仿真出生点固定，导航启动文件会运行 `publish_initial_pose.py`，等待 `/map`、`/scan`、`/Odometry` 和 AMCL 订阅者就绪后，向 AMCL 发布默认初始位姿 `(0, 0, 0)`。正常情况下直接使用 `2D Goal Pose` 指定目标点即可；如果 RViz 中机器人位置明显不准，再手动使用 `2D Pose Estimate` 校正。

启动最前几秒可能会出现等待 `map` 或 `odom` TF 的日志。只要随后能看到 `Published AMCL initial pose 1/1`，并且下面命令能持续输出位姿变换，就说明导航坐标链路已经恢复：

```bash
ros2 run tf2_ros tf2_echo map base_footprint
```

同时建议检查 Nav2 规划器来源：

```bash
ros2 pkg prefix nav2_planner
```

正常情况下应输出 `/opt/ros/humble`。如果输出 `/home/junyu/0glut2/...` 等旧工作区路径，说明当前终端环境混入了旧覆盖层。项目脚本已经在启动时清理 `AMENT_PREFIX_PATH`、`COLCON_PREFIX_PATH`、`CMAKE_PREFIX_PATH` 和 `ROS_PACKAGE_PATH`，遇到这种情况时执行 `./clean.sh` 后重新从 `~/slam_nav_ws` 运行启动脚本。

若 RViz 中发送目标点后出现 `Planning algorithm GridBased failed to generate a valid path`，优先确认机器人当前位置和目标点都在白色空旷区域内，并与障碍物黑边或灰色膨胀区保持一定距离。机器人贴近障碍物时，起点可能已经落入代价地图膨胀区，规划器会合理地拒绝生成路径。

Livox Mid-360 传感器已设置为 `always_on`，无 GUI 模式也可以发布 `/livox/lidar`。作业截图和交互调试仍建议使用 `./start_simulation.sh` 的默认 GUI 模式。


## 增强流程：3D 地形代价地图导航

默认导航链路主要依赖 `/scan`，它稳定、容易调试，适合当前作业验收。为了后续提升复杂障碍环境下的鲁棒性，项目新增了两级 LiDAR 地形分析和强度体素代价地图入口：

```bash
cd ~/slam_nav_ws
./start_navigation_3d.sh
```

该入口会在默认导航链路基础上额外启动：

```text
FAST-LIO2 /cloud_registered_body + /Odometry
  -> terrain_analysis
  -> /terrain_map
  -> terrain_analysis_ext
  -> /terrain_map_ext
  -> Nav2 IntensityVoxelLayer
  -> SmacPlanner2D + OmniPidPursuitController + BackUpFreeSpace
```

对应配置文件：

```text
src/slam_nav_bringup/config/nav2_params_3d.yaml
src/slam_nav_bringup/launch/navigation_3d.launch.py
src/terrain_analysis/
src/terrain_analysis_ext/
src/pb_nav2_plugins/
src/pb_omni_pid_pursuit_controller/
```

3D 增强链路保留原有 `/scan` 障碍层作为兜底，同时新增 `/terrain_map` 和 `/terrain_map_ext` 的 PointCloud2 观测源。地形分析节点会估计局部地面高度，并把点相对地面的高度差写入 `intensity`；Nav2 中的 `IntensityVoxelLayer` 再按高度和 intensity 范围筛选障碍点。这样比直接把完整点云或简单高度带塞进普通 VoxelLayer 更稳，能减少地面点、历史点云和无效高度点把 costmap 大面积染黑的问题。

`adaptive_cloud_filter` 仍保留为辅助感知适配与可视化输出，默认 3D costmap 不再直接订阅 `/cloud_nav_filtered`。初次使用时建议观察 `/terrain_map`、`/terrain_map_ext`、`/scan`、`/local_costmap/costmap` 和 `/global_costmap/costmap` 是否正常。

## 增强流程：鲁棒导航入口

`start_robust_navigation.sh` 是面向后续长期开发的统一增强入口，它会启动 3D 点云导航、定位健康监控和速度安全桥：

```bash
cd ~/slam_nav_ws
./start_robust_navigation.sh
```

默认组合：

```text
navigation_3d.launch.py
localization_guard.launch.py
safe_cmd_bridge.launch.py
```

默认情况下，速度安全桥只把 `/cmd_vel` 整形成 `/cmd_vel_safe`，不会改变 Gazebo 底盘仍然订阅 `/cmd_vel` 的事实；定位健康监控也只发布状态，不主动停车。这让增强入口适合先做观察和调试。进入真实底盘部署时，可以逐步打开：

```bash
./start_robust_navigation.sh \
  publish_zero_on_fault:=true \
  safe_enable_fault_stop:=true \
  safe_enable_udp_output:=true \
  safe_udp_host:=192.168.123.22
```

当前作业演示仍建议优先使用 `start_navigation.sh` 或 `start_navigation_3d.sh`，`start_robust_navigation.sh` 更适合后续实机级鲁棒性测试。


## 可选流程：任务层行为树

`src/mission_behavior/` 是当前工作区中的任务层行为树包，放在 `slam_nav_ws` 内部维护，不另开工作空间。它不替代 FAST-LIO2、slam_toolbox 或 Nav2，而是在导航系统已经启动后，作为上层任务入口调用 Nav2 的动作和恢复服务。

当前行为树骨架：

```text
等待 Nav2 动作与服务就绪
  -> 发送 NavigateToPose
  -> 若导航失败
     -> 基于 local costmap 选择更空的后退/侧移方向
     -> 发布短距离脱困 cmd_vel
     -> 若自由空间恢复不可用，则清理 costmap 并执行 Nav2 BackUp
     -> 等待 0.5 s
     -> 重新发送 NavigateToPose
```

运行前先按“加载地图导航”启动仿真和导航，确认 Nav2 已 active。然后另开终端：

```bash
cd ~/slam_nav_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch mission_behavior mission_behavior.launch.py auto_start:=true goal_x:=8.0 goal_y:=4.0 goal_yaw:=0.0
```

常用可调参数：

```bash
ros2 launch mission_behavior mission_behavior.launch.py \
  auto_start:=true \
  goal_x:=8.0 goal_y:=4.0 goal_yaw:=0.0 \
  max_navigation_retries:=2 \
  recovery_strategy:=free_space \
  backup_distance:=0.5 backup_speed:=0.12
```

它更适合放在后续扩展中继续发展：接收语义任务、选择目标点、触发视觉识别、调度机械臂动作、处理失败恢复。当前结课作业仍以稳定建图和 Nav2 目标点导航为主。

## 可选流程：速度安全桥

`src/safe_cmd_bridge/` 是面向实机部署阶段新增的通用速度安全层。它订阅原始速度指令，输出经过限速、限加速度、死区过滤和超时停车处理后的速度指令。

默认测试方式不会影响当前仿真底盘：

```bash
cd ~/slam_nav_ws
./start_safe_cmd_bridge.sh
```

默认输入 `/cmd_vel`，输出 `/cmd_vel_safe`。可以用下面命令观察安全层是否正常限幅：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 2.0, y: 1.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 3.0}}"
```

```bash
ros2 topic echo /cmd_vel_safe
```

后续接真实底盘时，可以把 Nav2 的速度输出重映射到安全桥输入，再由安全桥输出给底盘控制接口；如果底盘侧需要独立控制进程，也可以打开 UDP 输出。当前作业主流程不强制启用这一层，以免影响已经验证过的仿真导航链路。

如果底盘能提供实际里程计反馈，可以打开反馈看门狗，检查“持续发送速度但底盘没有响应”或“底盘反馈断流”的情况：

```bash
./start_robust_navigation.sh \
  safe_enable_feedback_watchdog:=true \
  safe_feedback_topic:=/base/odom
```

反馈异常时安全桥会发布：

```text
/base_feedback_fault
/base_feedback_health
```

## 可选流程：PCD 地图辅助重定位

`src/cloud_relocalization/` 提供一个可触发的 ICP 点云地图重定位入口，用于部署阶段处理初始位姿不准、局部漂移或需要从已保存 PCD 地图恢复的情况。它默认只发布状态、估计位姿和对齐点云，不直接接管 `map -> odom`。

```bash
cd ~/slam_nav_ws
./start_relocalization.sh \
  map_pcd_path:=/home/junyu/slam_nav_ws/src/FAST_LIO/PCD/scan.pcd \
  input_cloud_topic:=/cloud_registered \
  publish_tf:=false
```

触发一次匹配：

```bash
ros2 service call /relocalization/trigger std_srvs/srv/Trigger {}
```

建议先在 RViz 中观察 `/relocalization/aligned_cloud` 和 `/relocalization/pose`，确认匹配方向正确后，再显式使用 `publish_tf:=true`。当前实现会围绕初始位姿裁剪局部 PCD 子图，并同时检查 ICP fitness、局部地图点数和位姿跳变门限，避免相似场景下错误匹配把坐标系突然拉偏。

## 可选流程：定位健康监控

`src/localization_guard/` 是面向实机部署和长期运行新增的定位健康监控层。它订阅 `/Odometry`、`/cloud_registered` 和 `/scan`，检测断流、速度异常和位姿跳变，并发布统一健康状态。

```bash
cd ~/slam_nav_ws
./start_localization_guard.sh
```

默认只监控，不接管控制。输出话题：

```text
/localization_health
/localization_fault
/diagnostics
```

实机保守模式可以打开故障零速度输出：

```bash
./start_localization_guard.sh publish_zero_on_fault:=true
```

这个模块不替代地图辅助重定位算法。它的作用是先把“定位是否可信”变成明确状态，后续可以接入任务层、速度安全桥或重定位触发逻辑。

## 常用检查

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic hz /Odometry
ros2 topic hz /cloud_registered
ros2 topic hz /cloud_nav_filtered
ros2 topic echo /perception_mode
ros2 topic hz /scan
ros2 topic echo /localization_health
ros2 topic echo /localization_fault
ros2 topic echo /map --once --qos-durability transient_local
ros2 run tf2_ros tf2_echo map base_footprint
ros2 topic info /cmd_vel
ros2 lifecycle nodes
```

启动仿真和导航后，也可以先跑一键运行时诊断：

```bash
cd ~/slam_nav_ws
./diagnose_runtime.sh --duration 5
```

它会采样 `/clock`、`/livox/lidar`、`/cloud_registered`、`/cloud_nav_filtered`、`/nav_obstacle_cloud`、`/nav_ground_cloud`、`/visual_obstacles`、`/scan`、`/Odometry`、`/map`、`/cmd_vel`、`/cmd_vel_safe` 和关键 TF 链路，并检查消息时间戳与仿真时间是否明显错位。诊断输出还会列出 local/global costmap 正在订阅哪些观测源，便于发现 costmap 误订阅完整点云或未验证视觉点云的问题。遇到 `timestamp earlier than transform cache`、`map frame does not exist`、`GridBased failed`、话题断流或 RViz 中 TF 断链时，建议先跑这个命令确认底层输入是否正常。

3D 增强链路还可以单独观察感知适配层输出：

```bash
ros2 topic hz /cloud_nav_filtered
ros2 topic hz /nav_obstacle_cloud
ros2 topic hz /nav_ground_cloud
ros2 topic echo /perception_mode
```

清理残留进程：

```bash
cd ~/slam_nav_ws
./clean.sh
```

## 当前创新定位

当前不把动态障碍物或行为树作为主线创新。更稳的定位是：

```text
基于 FAST-LIO2 与 Nav2 的可扩展移动机器人 SLAM 导航底座，
并预留面向部署阶段的松耦合感知适配接口。
```

`src/perception_adapter/` 是这个接口的雏形。它不参与首次建图，后续可在部署导航阶段接入，用于速度感知点云处理、RGB-D 深度相机、语义识别和行为树决策扩展。

`src/mission_behavior/` 是任务层行为树骨架，当前提供导航失败后的清图、后退、重试流程，后续可作为语义任务和机械臂调度入口。

## 可选流程：Piper 移动操作扩展

`src/slam_nav_piper_*` 是 task2 长期扩展中的 Piper 机械臂方向。它保持独立 `/piper` 命名空间，不修改 task1 的 FAST-LIO2 + Nav2 主链路，也不把机械臂 RGB-D 相机 remap 到 `/nav_camera/*`。

当前已安装 MoveIt2 规划接口：

```text
ros-humble-moveit-ros-planning-interface 2.5.9
```

可用下面命令确认：

```bash
ros2 pkg prefix moveit_ros_planning_interface
ros2 pkg prefix moveit_ros_move_group
ros2 pkg prefix moveit_core
```

Piper 冒烟启动：

```bash
cd ~/slam_nav_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch slam_nav_piper_bringup piper_sim.launch.py
```

该入口只启动占位 TF、假腕部 RGB-D 相机、目标位姿估计、控制桥和 fake pick/place action。真实机械臂阶段仍需要接入 AgileX 官方 Piper URDF/MoveIt2 配置或厂家 SDK，项目侧上层接口保持 `/piper/task/pick_object` 和 `/piper/task/place_object`。


## 常见问题：FastDDS 共享内存端口锁

如果终端出现类似日志：

```text
[RTPS_TRANSPORT_SHM Error] Failed init_port fastrtps_port7421: open_and_lock_file failed
```

通常不是 FAST-LIO2 或 Nav2 算法错误，而是 FastDDS/FastRTPS 在 WSL 或异常退出后留下了共享内存端口锁。处理方式是先停止当前流程，再清理残留：

```bash
cd ~/slam_nav_ws
./clean.sh
```

不要在 ROS/Gazebo 正在运行时手动删除 `/dev/shm/fastrtps_*` 或 `sem.fastrtps_*`，否则可能打断正在通信的节点。`clean.sh` 会先终止本工作区相关进程，再清理 FastDDS/FastRTPS 的共享内存残留。

## 后续长期扩展

长期规划放在：

```text
tasks/task2/FUTURE_ROADMAP.md
tasks/task2/ROBUST_NAVIGATION_UPGRADE_PLAN.md
tasks/task2/PIPER_MOBILE_MANIPULATION.md
```

包括：

- 鲁棒导航升级路线；
- RGB-D 深度相机近场感知；
- 本地语义任务理解；
- 行为树任务决策；
- 机械臂抓取与放置；
- 是否评估 Fast-LIVO2 等紧耦合视觉-激光-惯性算法。
