# SLAM 导航系统实现过程记录

本文档记录当前工程的真实状态、主流程、创新定位和后续维护要点。它服务于 `tasks/task1` 的结课作业材料整理，也避免后续把建图、导航、长期扩展混在一起。

## 1. 工程定位

当前工程是一个 ROS2 Humble 移动机器人 SLAM 导航底座，运行环境为 Ubuntu 22.04 + Gazebo Classic。

当前结课作业目标：

- 成功构建仿真环境地图。
- 使用已保存地图实现指定目标点自主导航。
- 在包含不少于 5 个障碍物的场地中完成静态避障。
- 通过截图和实验记录证明静态避障成功率不低于 80%。

长期扩展目标放在 `tasks/task2`，包括 RGB-D 相机、行为树、语义理解和机械臂。

## 2. 保留与抽离原则

保留内容：

- Gazebo 仿真场地、机器人模型、LiDAR/IMU 传感器。
- FAST-LIO2 风格 LiDAR-IMU 定位建图前端。
- `pointcloud_to_laserscan`，用于把 3D 点云转换为 2D `/scan`。
- `slam_toolbox`，用于首次在线建图。
- Nav2，用于加载地图后的目标点导航和静态避障。

不作为当前主线的内容：

- 特定场景专用模块。
- 动态障碍物作为主创新。
- 行为树决策不参与当前建图主流程；已新增 `mission_behavior` 作为后续任务层初版。
- Fast-LIVO/Fast-LIVO2。
- 深度相机和机械臂。

## 3. 主链路

### 3.1 建图链路

建图阶段追求地图完整和稳定，不接入 `perception_adapter`。

```text
Gazebo
  -> /livox/lidar + /livox/imu
  -> FAST-LIO2
  -> /Odometry + /cloud_registered
  -> pointcloud_to_laserscan
  -> /scan
  -> slam_toolbox
  -> /map
  -> ./run.sh save-map nav_test_map
```

为减少手动键盘探索成本，新增 `auto_explore_mapper` 作为建图阶段的可选自动巡航节点：

```text
/scan + /Odometry
  -> auto_explore_mapper
  -> /cmd_vel
  -> Gazebo robot
```

它不依赖 Nav2，也不要求已有地图；逻辑是基于 `/scan` 的保守反应式覆盖巡航：安全时慢速前进，前方过近时后退并向更空方向转向，周期性原地旋转补全 Livox 点云视角。该入口适合自动扫静态仿真地图和累计 FAST-LIO PCD 地图，但它不是完整 frontier planner；如果某些角落覆盖不足，仍可用 `./run.sh teleop` 手动补扫。

对应入口：

```bash
cd ~/slam_nav_ws
./run.sh auto-mapping
./run.sh save-pcd nav_test_static
```

`save_pcd_map.sh` 调用 FAST-LIO 的 `/map_save` 服务，默认更新 `src/FAST_LIO/PCD/scan.pcd`，传入名称时会额外复制一份命名 PCD，方便后续重定位或报告截图使用。

### 3.2 导航链路

导航阶段加载已保存地图，不再运行 slam_toolbox 建图。

```text
Gazebo
  -> FAST-LIO2
  -> pointcloud_to_laserscan -> /scan
  -> Nav2 + map_server(nav_test_map.yaml) + AMCL initial pose publisher
  -> /cmd_vel
  -> Gazebo robot
```

当前 `./run.sh nav` 会调用 `scripts/start_navigation.sh`，启动 FAST-LIO2、`pointcloud_to_laserscan`、Nav2 bringup 和初始位姿发布器，并默认加载 `nav_test_map.yaml`。

导航鲁棒性排查后，当前默认链路先做“底层稳定性优先”的修复，而不是立即替换整套控制器：

- Livox 仿真点云时间戳统一使用 Gazebo `world->SimTime()`，避免 `/clock`、TF 缓存和点云消息时间不一致。
- FAST-LIO2 的近距离盲区从 2.0 m 调整为 0.4 m，保留更多近距离障碍点，避免机器人面前障碍物被前端直接滤掉。
- `pointcloud_to_laserscan` 的 TF 容忍从 0.05 s 调整为 0.20 s，降低 FAST-LIO 和 TF 轻微延迟时 `/scan` 丢帧的概率。
- Nav2 障碍观测增加短时保留，并提高 DWB 中障碍物 critic 权重，先缓解贴障、局部卡住和障碍闪烁问题。

这一步不改变算法主线，只是先修复会导致“容易飘、局部避障弱、时间戳 drop”的底层问题。后续再逐步迁移更强的三维障碍层、自由空间脱困行为和全向路径跟踪控制器。


### 3.3 3D 地形代价地图增强链路

默认导航链路只把 `/cloud_registered` 投影为 `/scan` 后送入 Nav2，优点是稳定、容易验收；缺点是会损失高度、密度和局部三维结构信息。为后续复杂障碍场景和实机部署预留鲁棒性提升空间，当前 3D 增强链路已从“简单高度带点云”升级为“两级地形分析 + intensity 体素代价地图”：

```text
Gazebo / real LiDAR
  -> FAST-LIO2
  -> /cloud_registered_body + /Odometry
  -> terrain_analysis
  -> /terrain_map
  -> terrain_analysis_ext
  -> /terrain_map_ext
  -> Nav2 IntensityVoxelLayer
  -> local/global costmap
```

对应入口：

```bash
cd ~/slam_nav_ws
./run.sh nav-3d
```

对应文件：

```text
src/slam_nav_bringup/config/nav2_params_3d.yaml
src/slam_nav_bringup/launch/navigation_3d.launch.py
src/terrain_analysis/
src/terrain_analysis_ext/
src/pb_nav2_plugins/
```

这个链路保留原有 `/scan` 障碍层作为稳定兜底，并额外加入地形分析后的 PointCloud2 观测源。一阶段 `terrain_analysis` 维护近场滚动地形图，估计局部地面高度，把点相对地面的高度差写入 `intensity` 并发布 `/terrain_map`；二阶段 `terrain_analysis_ext` 融合近场地形，维护更大范围滚动地形图并发布 `/terrain_map_ext`。Nav2 中的 `IntensityVoxelLayer` 按高度和 intensity 范围筛选障碍点，避免把地面点、历史点云或完整注册点云整片标成障碍。

`adaptive_cloud_filter` 仍保留为松耦合感知适配和可视化诊断入口，但默认 3D costmap 不再直接订阅 `/cloud_nav_filtered`。当前结构的目标是让系统从“2D 投影避障”逐步升级为“2D 稳定兜底 + 3D 地形代价地图”的感知结构。该链路主要面向导航部署阶段，不参与首次建图主流程。


2026-07-06 验证记录：

- `navigation_3d.launch.py` 默认使用非组合模式启动 Nav2。原因是当前 ROS2 Humble 官方组合启动路径下，costmap 子节点参数容易回落到默认插件；非组合模式可以稳定加载 `nav2_params_3d.yaml` 中的 `IntensityVoxelLayer`。
- `params_file` 仍保留兼容；新增的 `nav2_params_file` / `navigation_params_file` 默认继承 `params_file`，避免后续传自定义参数文件时被嵌套 launch 忽略。
- 已验证 local costmap 插件为 `["intensity_voxel_layer", "obstacle_layer", "inflation_layer"]`，global costmap 插件为 `["static_layer", "intensity_voxel_layer", "obstacle_layer", "inflation_layer"]`。
- 启用 `enable_nav_rgbd_camera:=true` 和 `start_navigation_rgbd.sh` 时，`/visual_obstacles` 会作为 local costmap `ObstacleLayer` 的 PointCloud2 观察源。该链路是松耦合近场补盲，只影响局部代价地图，不写入 global costmap。
- 若终端自动 source 了其他 ROS2 工作区，可能会通过 `LD_LIBRARY_PATH` 误加载旧插件库，表现为 local costmap 配置失败、`local_costmap/clear_entirely_local_costmap` 服务缺失、RViz goal 被拒。已新增 `scripts/setup_workspace_env.sh`，各启动脚本会先清理旧 overlay 再 source 当前工作区。

### 3.4 部署阶段速度安全桥链路

速度安全桥不参与首次建图，也不强制参与当前作业的默认仿真导航。它是面向后续实机部署的安全层，用于把导航控制器输出的速度指令整理成更适合真实底盘执行的速度。

```text
Nav2 / teleop / mission_behavior
  -> /cmd_vel
  -> safe_cmd_bridge
  -> 限速、限加速度、死区过滤、超时停车
  -> /cmd_vel_safe 或 UDP
  -> 仿真底盘 / 真实底盘控制进程
```

当前实现位于：

```text
src/safe_cmd_bridge/
start_safe_cmd_bridge.sh
```

它的价值不是提升建图精度，而是提高部署阶段的可控性：防止异常速度直接打到底盘，避免通信中断后继续执行旧速度，并为后续真实底盘接口预留清晰边界。


### 3.5 定位健康监控链路

长期运行和实机部署时，系统需要知道定位输入是否仍然可信。新增 `localization_guard` 作为运行时健康监控层：

```text
/Odometry
/cloud_registered
/scan
  -> localization_guard
  -> /localization_health
  -> /localization_fault
  -> /diagnostics
```

它会检测里程计、点云和 LaserScan 是否断流，也会检测明显速度异常和位姿跳变。默认只发布状态，不抢控制；需要实机保守策略时，可以设置 `publish_zero_on_fault:=true`，在故障保持时间内向 `/cmd_vel` 发布零速度。

对应文件：

```text
src/localization_guard/
start_localization_guard.sh
```

这个模块不等同于 ICP/GICP 重定位。它是后续重定位触发、安全停车、任务层恢复的状态入口，先解决“系统什么时候应该怀疑定位不可信”的问题。

### 3.5.1 PCD 地图辅助重定位链路

`cloud_relocalization` 是面向部署阶段的先验点云地图辅助重定位模块，不参与首次建图，也不默认接管主导航 TF。它订阅当前注册点云，加载离线 PCD 地图，并在手动触发或自动触发时执行点云配准：

```text
/cloud_registered + PCD map
  -> cloud_relocalization
  -> ICP/GICP/NDT
  -> /relocalization/status
  -> /relocalization/pose
  -> /relocalization/aligned_cloud
  -> 可选 map -> odom
```

当前实现支持三种后端：

```text
ICP：速度快、参数少，适合初值较准时快速校验。
GICP：考虑局部几何协方差，配准稳定性通常优于普通 ICP，但计算更重。
NDT：基于体素正态分布，适合较粗粒度地图匹配，需要调体素分辨率。
```

为了避免错误匹配直接拉偏导航坐标系，默认 `publish_tf:=false`，并加入局部 PCD 子图裁剪、fitness 门限、局部地图点数检查和位姿跳变门限。推荐调试顺序是：先观察 `/relocalization/aligned_cloud` 与 `/relocalization/pose`，确认多次触发结果稳定后，再考虑是否允许它发布 `map -> odom`。

这个模块和 `localization_guard` 的关系是：`localization_guard` 判断定位是否可疑，`cloud_relocalization` 提供地图辅助校准手段。当前版本暂不把二者自动闭环，避免在仿真或实机中因为一次误匹配造成更大跳变。


### 3.6 统一鲁棒导航入口

随着增强模块增多，单独启动 3D 导航、定位健康监控和速度安全桥会让实验流程变散。新增 `robust_navigation.launch.py` 作为统一增强入口：

```text
robust_navigation.launch.py
  -> navigation_3d.launch.py
  -> localization_guard.launch.py
  -> safe_cmd_bridge.launch.py
```

对应脚本：

```bash
cd ~/slam_nav_ws
./run.sh robust-nav
```

默认配置保持保守：

- 3D costmap 参与 Nav2 感知输入。
- localization_guard 只发布 `/localization_health`、`/localization_fault` 和 `/diagnostics`。
- safe_cmd_bridge 订阅 `/localization_fault`，并只输出 `/cmd_vel_safe`，不改变默认 `/cmd_vel -> Gazebo robot` 控制链。

进入真实底盘部署阶段后，可以逐步开启故障停车和 UDP 输出：

```bash
./run.sh robust-nav \
  publish_zero_on_fault:=true \
  safe_enable_fault_stop:=true \
  safe_enable_udp_output:=true \
  safe_udp_host:=192.168.123.22
```

这个入口用于后续长期增强测试，不强制替代当前结课作业的默认导航流程。


### 3.7 任务层行为树链路

行为树不参与首次建图，当前作为导航启动后的上层任务入口。它适合后续接入语义理解、视觉识别和机械臂控制。

```text
mission_behavior
  -> NavigateToPose action
  -> 若失败，读取 local costmap 和 Odometry
  -> 选择更空的后退/侧移方向并短距离发布 cmd_vel 脱困
  -> 若自由空间恢复不可用，则回退到 clear costmap + BackUp action
  -> 等待并重新发送 NavigateToPose
```

当前版本是轻量 Python 节点，配套 XML 用于描述树结构。后续如果任务复杂度提高，可以迁移为 BehaviorTree.CPP 插件形式。
## 4. 启动步骤

构建：

```bash
cd ~/slam_nav_ws
./run.sh build
source install/setup.bash
```

建图：

```bash
cd ~/slam_nav_ws
./run.sh clean
./run.sh sim-static
```

另开终端：

```bash
cd ~/slam_nav_ws
./run.sh mapping
```

另开终端控制探索：

```bash
cd ~/slam_nav_ws
./run.sh teleop
```

保存地图：

```bash
cd ~/slam_nav_ws
./run.sh save-map nav_test_map
```

导航：

```bash
cd ~/slam_nav_ws
./run.sh clean
./run.sh sim-static
```

另开终端：

```bash
cd ~/slam_nav_ws
./run.sh nav
```

导航启动后建议先做一次统一诊断：

```bash
cd ~/slam_nav_ws
./run.sh diagnose --duration 5
```

该诊断用于确认仿真时间、传感器话题、Nav2 输入输出和 TF 链路是否正常，重点关注 `/clock`、`/livox/lidar`、`/cloud_registered`、`/scan`、`/Odometry`、`/map`、`/cmd_vel`、`map -> odom` 和 `map -> base_footprint`。它主要服务于实验复现和故障定位，不改变建图或导航算法流程。

3D 点云代价地图增强导航可选运行：

```bash
cd ~/slam_nav_ws
./run.sh nav-3d
```

第一次测试 3D 增强链路时，建议重点观察：

```bash
ros2 topic hz /cloud_nav_filtered
ros2 topic echo /perception_mode
ros2 topic hz /local_costmap/voxel_grid
ros2 topic hz /global_costmap/voxel_grid
```

统一鲁棒导航入口可选运行：

```bash
cd ~/slam_nav_ws
./run.sh robust-nav
```

它会同时启动 3D 点云代价地图、定位健康监控和速度安全桥。默认适合观察，不会主动改变 Gazebo 底盘控制链。


任务层行为树可选运行：

```bash
cd ~/slam_nav_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch mission_behavior mission_behavior.launch.py auto_start:=true goal_x:=8.0 goal_y:=4.0 goal_yaw:=0.0
```

速度安全桥可选运行：

```bash
cd ~/slam_nav_ws
./run.sh safe-bridge
```

默认输入 `/cmd_vel`，输出 `/cmd_vel_safe`，用于单独验证限速和超时停车。当前作业演示仍可保持默认 `/cmd_vel -> Gazebo robot` 链路；等进入实机部署阶段，再把 Nav2 输出重映射到安全桥输入。

定位健康监控可选运行：

```bash
cd ~/slam_nav_ws
./run.sh guard
```

常用检查：

```bash
ros2 topic echo /localization_health
ros2 topic echo /localization_fault
ros2 topic echo /diagnostics
```

默认只监控不干预；实机保守模式可使用：

```bash
./run.sh guard publish_zero_on_fault:=true
```

当前仿真出生点固定，导航启动文件会运行 `publish_initial_pose.py`，等待 `/map`、`/scan`、`/Odometry` 和 AMCL 订阅者就绪后，向 AMCL 发布默认初始位姿 `(0, 0, 0)`。一般直接使用 `2D Goal Pose` 即可；如果 RViz 中机器人位姿明显不准，再使用 `2D Pose Estimate` 手动校正。

Gazebo Classic 在 WSL 中启动较慢时，`/spawn_entity` 服务可能需要二三十秒才出现。当前仿真启动文件已将机器人生成超时提高到 120 秒，并显式加载 `gazebo_ros_init`、`gazebo_ros_factory` 和 `gazebo_ros_force_system`。因此启动仿真后应先等到终端出现 `SpawnEntity: Successfully spawned entity [mobile_robot]`，再判断传感器和导航是否正常。

Livox Mid-360 传感器已设置为 `always_on`，无 GUI 模式也可以发布 `/livox/lidar`。作业演示和截图仍建议使用默认 GUI 模式启动仿真。

## 5. 验收指标对应关系

成功构建环境地图：

- RViz 中显示 `/map`。
- 成功保存 `nav_test_map.yaml` 和 `nav_test_map.pgm`。

实现指定目标点自主导航：

- Nav2 成功加载地图。
- RViz 中全局路径出现。
- 机器人发布 `/cmd_vel` 并向目标点移动。
- 最终到达目标点附近。

静态避障成功率不低于 80%：

- 建议至少测试 10 次目标点导航。
- 记录成功、失败、碰撞或卡住情况。
- 成功次数不少于 8 次即可满足 80%。

地图障碍物不少于 5 个：

- 静态场地中包含外围墙、走廊、窄门、柱状障碍、低矮障碍、斜坡平台等。
- 报告截图中需要标注至少 5 个障碍物。

## 6. 当前创新点定位

当前不把动态障碍物作为主创新点。更稳的创新定位是：

```text
设计可扩展的 ROS2 移动机器人 SLAM 导航底座，
并预留部署阶段的松耦合感知适配接口与 3D 点云避障增强链路。
```

`src/perception_adapter/` 是这个接口的初步实现：

- 订阅 `/cloud_registered` 和 `/Odometry`。
- 发布 `/cloud_nav_filtered` 和 `/perception_mode`。
- 当前不参与首次建图。
- 后续可接入导航部署阶段，用于速度自适应点云处理、RGB-D 深度相机、语义识别和行为树决策。

报告中应诚实表述为“架构扩展设计与接口预留”，不要写成已经显著提升建图质量。

当前 3D 点云增强链路可表述为“在 2D LaserScan 导航输入基础上，增加 PointCloud2 体素代价地图观测源”。它提升的是避障感知输入的丰富度，不改变 FAST-LIO2 建图算法本身。报告中可以把它放在系统改进或后续扩展章节，并用 RViz 中 voxel grid/costmap 的截图证明链路接入成功。

当前导航部分还加入了“拥挤障碍恢复策略”作为可验证的小创新点。Nav2 默认行为树中保留标准 `BackUp` 作为稳定兜底；任务层 `mission_behavior` 进一步实现了基于 local costmap 的自由空间脱困恢复：

```text
规划或跟踪失败
  -> 读取 local costmap
  -> 对后方、左后、右后、左、右方向进行走廊采样
  -> 选择占用率更低的方向短距离脱困
  -> 若采样数据不可用，则回退到清理 costmap + BackUp
  -> 等待 0.5 s
  -> 重新规划
```

对应文件为：

```text
src/mission_behavior/scripts/mission_behavior_node.py
src/mission_behavior/config/mission_behavior.yaml
src/mission_behavior/behavior_tree/mission_navigation_recovery.xml
src/slam_nav_bringup/behavior_tree/navigate_to_pose_with_backup_recovery.xml
src/slam_nav_bringup/behavior_tree/navigate_through_poses_with_backup_recovery.xml
```

报告中可以表述为“面向拥挤障碍场景的局部自由空间脱困恢复策略”，不要夸大成全新局部规划算法。它属于任务层恢复行为增强：正常路径跟踪仍由 Nav2 局部控制器完成，只有导航失败后才短时间介入。

部署增强部分新增了“速度安全桥”作为长期扩展创新链路的一部分：

```text
规划器输出速度
  -> 安全桥限幅和限加速度
  -> 指令超时自动降零
  -> 可选 ROS topic 或 UDP 输出
```

报告中如果需要写到这一点，应表述为“面向真实底盘部署的速度安全接口设计”。它是工程鲁棒性增强，不是 SLAM 算法本身创新；现阶段更适合放在系统设计或后续扩展章节中。

定位健康监控可以作为另一个工程鲁棒性增强点：

```text
定位输入断流/跳变/速度异常
  -> 发布 localization_fault
  -> safe_cmd_bridge 将安全速度降为零
  -> 任务层可进一步触发重试或重定位
```

报告中可表述为“面向长期运行的定位健康监测与安全状态输出”。它不宣称解决重定位问题，而是为后续地图辅助重定位提供触发条件。

## 7. 截图清单

结课报告建议至少收集：

- Gazebo 静态场地总览，标注 5 个以上障碍物。
- RViz 建图过程，显示 `/map`、TF、点云或 `/scan`。
- 保存后的地图文件截图或 `.pgm` 地图图像。
- Nav2 加载地图后的 RViz。
- 初始位姿发布器日志或 `map -> base_footprint` TF 检查结果。
- `2D Goal Pose` 后的全局路径。
- 机器人绕障碍运动过程。
- 到达目标点后的截图。
- 静态避障测试记录表。

## 8. 维护记录

- 2026-07-03：创建 `~/slam_nav_ws`，抽取通用 SLAM/导航能力，去除特定场景专用模块。
- 2026-07-03：完成基础仿真、建图、导航包整理。
- 2026-07-03：修正 slam_toolbox 最小激光量程参数，与 `/scan` 的 `range_min` 对齐。
- 2026-07-04：曾添加动态障碍物场地作为可选扩展，但当前不再作为主创新点。
- 2026-07-05：整理 FAST-LIO2 与 Point-LIO 学习笔记到 `tasks/task0`。
- 2026-07-05：创建 `tasks/task1/PROJECT_DELIVERY_GUIDE.md`，用于当前作业交付整理。
- 2026-07-05：创建 `tasks/task2/FUTURE_ROADMAP.md`，用于长期扩展规划。
- 2026-07-05：新增 `perception_adapter` 作为部署阶段感知适配接口雏形。
- 2026-07-05：删除“adapter 参与建图”的入口，明确建图阶段使用原始稳定链路。
- 2026-07-05：修正 `navigation.launch.py`，使导航流程加载保存地图，并启动 FAST-LIO2 与 `/scan` 输入。
- 2026-07-05：补全 AMCL 参数，统一 Nav2 机器人基坐标为 `base_footprint`，导航启动时延后 Nav2 以等待 FAST-LIO 初始化。
- 2026-07-05：为 Livox Mid-360 仿真传感器补充 `always_on`，修复无 GUI 模式下 `/livox/lidar` 可能不发布的问题。
- 2026-07-05：新增 `publish_initial_pose.py`，在 `/map`、`/scan`、`/Odometry` 和 AMCL 订阅者就绪后发布 `/initialpose`，修复导航启动时 `map` 坐标系无法稳定建立的问题。
- 2026-07-05：将导航启动拆分为 localization 和 navigation 两阶段，先启动 `map_server`/AMCL 并等待初始位姿发布器确认 `map -> base_footprint` 可用，再启动 planner/controller，避免 `planner_server` 抢先激活导致持续报 `map` frame 不存在。
- 2026-07-05：将 AMCL 的 `transform_tolerance` 从 1.0 调整为 0.2，减少 `map -> odom` 变换发布时间过于靠未来造成的 costmap 外推等待。
- 2026-07-05：加固 `slam_nav_simulation` 启动流程，显式启用 Gazebo ROS init/factory 插件，并把 `spawn_entity.py` 超时提高到 120 秒，适配 WSL/Gazebo Classic 慢启动。
- 2026-07-05：扩展 `clean.sh` 的清理范围，覆盖 Nav2、AMCL、map_server、planner/controller、lifecycle_manager 等残留进程，避免多次测试后的旧节点互相干扰。
- 2026-07-06：为 `build.sh`、`start_simulation.sh`、`start_mapping.sh`、`start_navigation.sh`、`teleop.sh` 和 `save_map.sh` 增加 ROS2 覆盖层清理，启动前清除当前终端中可能遗留的旧工作区路径，避免 `nav2_planner` 等包从 `/home/junyu/0glut2` 混入当前工程。
- 2026-07-06：记录 `GridBased failed to generate a valid path` 的排查经验：若目标点在原始地图中为可通行区域，仍需检查机器人起点是否贴近障碍物膨胀区，以及 `ros2 pkg prefix nav2_planner` 是否指向 `/opt/ros/humble`。
- 2026-07-06：新增显式 Nav2 行为树，调整恢复动作顺序为清理代价地图、后退脱困、等待重规划、旋转恢复，用于改善前方障碍密集时机器人卡在局部膨胀区的问题。
- 2026-07-06：新增 `mission_behavior` 任务层行为树包，提供导航动作调用、代价地图清理、后退脱困和重试流程，作为后续语义理解、视觉识别和机械臂调度的上层入口。
- 2026-07-06：明确 Nav2 的全局/局部规划链路。全局规划器由 `NavfnPlanner` 调整为 `SmacPlanner2D`，让路径搜索考虑代价地图代价；局部控制仍使用官方 `DWBLocalPlanner`，但开启 y 方向速度采样与速度平滑器 y 向输出，使仿真中的全向底盘能够横向避障，而不是按差速底盘方式硬转。
- 2026-07-06：根据鲁棒性复盘完成底层稳定性修复：Livox 仿真点云改用 Gazebo 仿真时间戳，FAST-LIO2 近距离盲区从 2.0 m 降至 0.4 m，`pointcloud_to_laserscan` TF 容忍提高到 0.20 s，Nav2 障碍观测增加短时保留并提高障碍 critic 权重。该修改优先解决时间戳 drop、近距离障碍缺失和局部避障偏弱问题。
- 2026-07-06：新增 `diagnose_runtime.sh` 和 `diagnose_runtime.py` 运行时诊断工具，用于采样 `/clock`、关键传感器话题、Nav2 输入输出和 TF 链路，辅助定位时间戳错位、话题断流和坐标变换断链问题。
- 2026-07-06：新增 `tasks/task2/ROBUST_NAVIGATION_UPGRADE_PLAN.md`，把后续鲁棒导航增强拆分为感知分割、时空体素代价地图、全向路径跟踪控制器、插件化脱困、安全底盘适配和点云重定位等可执行阶段。
- 2026-07-06：扩展 `perception_adapter`，在保留 `/cloud_nav_filtered` 的基础上新增 `/nav_obstacle_cloud` 和 `/nav_ground_cloud` 输出，作为后续导航障碍点云和地面点云分离的可视化验证入口。
- 2026-07-06：根据 RViz 中机器人周围大面积误判障碍的现象，收敛 `nav2_params_3d.yaml` 默认观测源：VoxelLayer 改为订阅 `/nav_obstacle_cloud`，不再默认订阅完整 `/cloud_nav_filtered` 和 `/visual_obstacles`，避免地面点、历史点云或未验证 RGB-D 点云污染代价地图。
- 2026-07-06：扩展 `diagnose_runtime.py` 的可选采样范围，加入 `/cloud_nav_filtered`、`/nav_obstacle_cloud`、`/nav_ground_cloud`、`/visual_obstacles` 和导航深度相机话题，方便区分 LiDAR 点云、RGB-D 点云和 Nav2 costmap 输入是否异常。
- 2026-07-06：继续增强 `diagnose_runtime.py`，新增 costmap 观测源订阅检查；当 local/global costmap 仍订阅完整 `/cloud_nav_filtered` 或未验证的 `/visual_obstacles` 时会给出警告，避免旧参数导致机器人周围被大面积误判为障碍。
- 2026-07-06：补齐完整 3D 地形导航链路：迁入并泛化 `terrain_analysis` 与 `terrain_analysis_ext` 两级滚动地形分析，默认接 `/cloud_registered_body` 与 `/Odometry`，发布 `/terrain_map` 与 `/terrain_map_ext`。
- 2026-07-06：迁入 Nav2 强度体素层和自由空间后退行为插件；`nav2_params_3d.yaml` 改为使用 `pb_nav2_costmap_2d::IntensityVoxelLayer`，并将 `backup` 恢复行为替换为 `pb_nav2_behaviors/BackUpFreeSpace`，用于拥挤场景下优先向更空方向脱困。
- 2026-07-06：`navigation_3d.launch.py` 与 `robust_navigation.launch.py` 默认拉起两级地形分析；`diagnose_runtime.py` 增加 `/terrain_map`、`/terrain_map_ext` 采样和 costmap 订阅检查。
- 2026-07-06：迁入并泛化全向 PID 路径跟踪控制器 `pb_omni_pid_pursuit_controller`，整理为通用全向底盘路径跟踪模块；`nav2_params_3d.yaml` 将 `FollowPath` 从 DWB 切换为 `pb_omni_pid_pursuit_controller::OmniPidPursuitController`，保留前瞻点跟踪、接近目标减速、曲率限速和基于里程计的卡住后退脱困逻辑。
- 2026-07-06：修复 `IntensityVoxelLayer` 导致 `Navigation inactive` 的插件库冲突：自定义 costmap 插件库原名 `liblayers.so` 与 Nav2 官方 `liblayers.so` 重名，导致 local costmap 首次加载时找不到 `nav2_costmap_2d::ObstacleLayer` 符号。现已改名为 `libpb_intensity_voxel_layer.so`，并显式链接官方 Nav2 costmap layers 库。
- 2026-07-06：修正行为树插件加载方式：`ReactiveSequence` 是 BehaviorTree.CPP 内置控制节点，不是 Humble 中的 Nav2 动态插件，不能写入 `plugin_lib_names`，否则 `bt_navigator` 会因找不到 `libnav2_reactive_sequence_bt_node.so` 而配置失败。当前保留 XML 中的 `ReactiveSequence`，但不再把它作为动态库加载。
- 2026-07-06：参考既有工程的 Nav2 启动方式，将 `navigation.launch.py`、`navigation_3d.launch.py` 和 `robust_navigation.launch.py` 默认切换为 composition 模式，启动 `component_container_mt` 并把 Nav2 localization/navigation 组件加载到同一个 `nav2_container`。该调整用于降低 WSL 高负载下 lifecycle service 响应超时的概率，避免 `smoother_server/get_state` 偶发超时后导致 `bt_navigator` 停留在 `unconfigured`。
- 2026-07-06：新增 Piper 移动操作扩展包族 `slam_nav_piper_*`，包括项目侧 action/msg、Piper 占位 TF、独立 `/piper/arm_camera/*` 感知、MoveIt2/SDK 控制边界、pick/place action server 和独立 bringup。该扩展不接入 task1 默认建图/导航脚本，不复用 `/nav_camera/*`，也不默认进入 Nav2 costmap。
- 2026-07-06：安装 `ros-humble-moveit-ros-planning-interface` 及 MoveIt2 规划相关依赖，确认 `moveit_ros_planning_interface`、`moveit_ros_move_group` 和 `moveit_core` 均来自 `/opt/ros/humble`。当前 Piper 控制层仍保持安全占位后端，后续接真实 MoveIt2 时在 `slam_nav_piper_control` 内部适配。
- 2026-07-06：为导航启动增加 `localization_mode` 参数，支持 `amcl` 与 `static` 两种定位模式。`amcl` 保留 `/scan` 与静态地图匹配后的重定位能力，但初始位姿不准或场景局部相似时可能把 `map->odom` 拉偏；`static` 模式只启动 `map_server` 并发布固定 `map->odom`，适合同一仿真地图、同一起点的短程导航对齐测试。`start_navigation_3d.sh` 与 `start_robust_navigation.sh` 默认改为 `localization_mode:=static`，用于先排除 AMCL 对地图/点云对齐的干扰。
- 2026-07-06：新增 `cloud_relocalization` 点云地图辅助重定位包，提供 `/relocalization/trigger` 触发服务、`/relocalization/status`、`/relocalization/pose` 和 `/relocalization/aligned_cloud` 输出；默认不发布 `map -> odom`，先用于观测和验证。
- 2026-07-06：增强 `cloud_relocalization` 的可靠性：启动时缓存初始位姿，避免运行时重复声明参数；ICP 目标从全图改为围绕初值裁剪局部 PCD 子图，并加入局部地图点数、fitness 和位姿跳变门限。
- 2026-07-06：将 `cloud_relocalization` 从单一 ICP 升级为可选 `icp/gicp/ndt` 三后端配准入口，新增 `registration_method`、`ndt_resolution`、`ndt_step_size` 参数和 `start_relocalization_gicp.sh` 便捷脚本；默认仍保持 `publish_tf:=false`，不接管主导航坐标链。
- 2026-07-06：增强 `perception_adapter` 点云预处理链路，新增局部网格地面参考估计，在 `/cloud_nav_filtered` 之外稳定输出 `/nav_obstacle_cloud` 与 `/nav_ground_cloud`，降低简单高度阈值在起伏地形下的误分割概率。
- 2026-07-06：增强 `safe_cmd_bridge` 实机底盘闭环入口，新增可选反馈看门狗；接入底盘里程计后可检测速度指令有输出但反馈速度过低、反馈断流等情况，并发布 `/base_feedback_fault` 与 `/base_feedback_health`。
- 2026-07-06：修正 RGB-D 松耦合避障链路：`depth_obstacle_projector` 不再直接发布相机光学坐标系下的整张深度视锥点云，而是先通过 TF 转到 `base_footprint`，再按前向距离、横向范围和障碍物高度过滤后发布 `/visual_obstacles`；同时将 RGB-D costmap 观察源与 RViz 显示 topic 对齐到 `/nav_camera/depth/image_raw` 和 `/visual_obstacles`，避免空旷前方被误判为扇形障碍。
- 2026-07-06：新增顶配仿真/导航入口 `start_simulation_dynamic_rgbd.sh` 与 `start_navigation_full.sh`，用于组合动态障碍物场景、导航 RGB-D 相机、3D 地形代价地图、RGB-D 近场补盲、全向路径跟踪、行为树恢复、定位健康监控与速度安全桥观测。该组合仍保持动态障碍只进入局部实时避障链路，不写入全局静态地图。
- 2026-07-06：新增 `auto_explore_mapper` 自动探索建图节点和 `start_auto_mapping.sh` 入口，用 `/scan` 做保守巡航、避障、周期旋转补扫，用于减少手动键盘建图成本；新增 `save_pcd_map.sh` 调用 FAST-LIO `/map_save` 服务保存并命名 PCD 地图。
- 2026-07-06：优化静态/动态仿真场地中的斜坡模型，将原单个倾斜长方体改为入口引导板、缓坡和顶部平台组合，降低入口硬边缘和坡度突变对底盘运动、点云建图和地形分析的干扰。
- 2026-07-06：新增 `tasks/task1/TASK1_FINAL_RUNBOOK.md` 作为 task1 权威运行与交付流程，明确静态场地用于课程验收、动态障碍物用于扩展演示，并修复 `start_simulation_dynamic.sh` / `start_simulation_dynamic_rgbd.sh` 中 `SCRIPT_DIR` 未定义导致 `./run.sh sim-dynamic` 入口失效的问题。
- 2026-07-07：新增 `./run.sh sim-static` 静态验收场地快捷入口，保留 `./run.sh sim` 当前默认动态场地用于扩展示范；同步 README、task1 Runbook、实验记录、Markdown 报告草稿和 LaTeX 报告中的建图/导航命令，避免动态障碍物混入静态避障成功率统计。
- 2026-07-07：修正 task1 报告中的 Nav2 算法描述：默认验收链路为 `SmacPlanner2D + DWBLocalPlanner + BackUp/ClearCostmap/Spin` 恢复行为树，`nav-3d/nav-full` 才使用地形分析、强度体素代价地图和全向 PID 控制器作为增强展示。
- 2026-07-07：为 Piper MoveIt2 plan-only 增加 `./run.sh piper-plan-test` 冒烟测试入口，向 `/piper/plan_kinematic_path` 发送一次关节目标规划请求，只验证返回轨迹是否非空，不执行轨迹、不连接 SDK，也不接入 task1 默认导航链路。
- 2026-07-07：新增并验证 `./run.sh piper-task-smoke`，在独立 `ROS_DOMAIN_ID` 下启动 Piper fake 感知/任务链路，确认 `/piper/arm_camera/*`、`/piper/perception/target_pose`、`/piper/grasp_candidates`、`/piper/task/pick_object` 和 `/piper/task/place_object` 均可连通；该冒烟只验证项目侧任务边界，不启动 Nav2、不连接真实 SDK。
- 2026-07-07：调整 `piper_mobile_manipulation.launch.py` 的 TF 边界，默认 `start_description:=false`，适合与已启动的整车仿真或真实机器人并行，避免重复发布 Piper TF；脱离 Gazebo 单独冒烟时才显式使用 `start_description:=true publish_joint_states:=true`。
- 2026-07-07：清理根目录旧版 `start_*.sh` 和 `setup_piper_open_class.sh` wrapper，统一保留 `./run.sh` 作为根目录导航入口，实际脚本集中在 `scripts/` 目录；同步修正 `perception_adapter` 文档中的旧脚本调用。
- 2026-07-07：统一 task1 创新与扩展示范截图口径：静态 80% 成功率只使用静态场地 10 次目标点测试，动态障碍物写作扩展验证；RGB-D `/visual_obstacles` 和 `perception_adapter` 截图作为可选扩展材料。同步更新 Markdown 报告、LaTeX 报告、实验记录和交付清单。
- 2026-07-07：校准 task2 鲁棒导航路线图状态，将已接入的 `IntensityVoxelLayer`、`BackUpFreeSpace`、全向 PID 控制器、`cloud_relocalization` 和 `safe_cmd_bridge` 从“待设计/待实现”更新为“已有初版、待实测调参或闭环验证”。
- 2026-07-07：新增 `./run.sh task1-check` 无 GUI 交付预检入口，检查根目录统一入口、脚本可执行性、默认地图、task1 文档、报告源文件、截图文件、实验记录占位和无关比赛业务字段；普通模式把截图/实验未完成作为 warning，`--strict` 用于最终打包前确认无 warning。
- 2026-07-07：新增 `./run.sh task1-runtime-check [mapping|nav|dynamic]` 运行时链路检查入口，用短超时采样当前 ROS 图中的 `/clock`、Livox 点云/IMU、`/Odometry`、`/scan`、`/map`、TF、`/cmd_vel`、Nav2 lifecycle 和 `/navigate_to_pose` action；它不启动 Gazebo/RViz/Nav2，只用于你已经跑起仿真、建图或导航后快速判断能否继续保存地图、发目标点或截图。
- 2026-07-07：新增 `./run.sh real-preflight` 无 GUI/无硬件实机部署前预检入口，检查 `safe_cmd_bridge` 默认关闭 UDP 输出、保留定位故障停车和速度超时，检查 `localization_guard`、`cloud_relocalization`、RGB-D 松耦合链路、Piper/task1 隔离边界和网络信息；`--strict` 用于上车前把 warning 也视为未通过。
- 2026-07-07：补强 task1 最短验收路线和现场记录口径：`README.md` 与 `TASK1_FINAL_RUNBOOK.md` 增加从 `task1-check`、静态建图、保存地图、静态导航到 10 次静态避障测试的最短闭环；`RUN_AND_SCREENSHOT_STEPS.md` 增加 `task1-runtime-check` 输出如何转写到实验记录；`EXPERIMENT_RECORD.md` 增加 10 个目标区域建议、失败判定和动态障碍物不计入静态成功率的记录边界。
- 2026-07-07：为 `./run.sh clean` 增加 `--dry-run` 安全预览模式，并把交互菜单补齐到静态/动态/RGB-D 仿真、RGB-D 导航、完整增强导航和 task1 runtime 检查；task1 文档同步建议在正式清理前先预览将被终止的进程和 FastDDS/FastRTPS 共享内存残留，降低多终端调试时误关当前流程的风险。
- 2026-07-07：新增 `slam_nav_piper_calibration` 手眼标定配置包和 `./run.sh piper-hand-eye-check` 静态检查入口，先固定 Piper 腕部 RGB-D eye-in-hand 的 frame、topic、输出目录和安全开关；默认不启用真实采样、不运动机械臂、不发布最终 TF、不写入 URDF，只作为 task2 后续实机标定前的边界检查。
- 2026-07-07：新增 `./run.sh task1-delivery-check` 打包交付前自查入口，独立于运行时 ROS 检查，聚焦结课提交材料：建议压缩包名、源码/脚本/文档/报告材料、截图缺口、实验记录待填字段、报告占位和 Git 中是否误跟踪 build/install/log/rosbag/点云/模型权重等重型产物。
- 2026-07-07：为 Piper pick 任务层增加手眼标定门禁：当 `fake_execution=false` 且声明真实 MoveIt2/SDK 后端接入时，若 `hand_eye_calibrated=false` 或标定结果文件不存在，`/piper/task/pick_object` 必须安全 ABORT；新增 `./run.sh piper-hand-eye-gate` 在独立 ROS domain 下验证该安全拒绝路径，不接入 task1 默认导航链路。
- 2026-07-07：为 Piper 真实机械臂动作增加底盘停止门禁：当 `fake_execution=false` 且声明真实后端接入时，若未显式确认 `base_stop_confirmed=true` 且任务层也没有 `publish_base_stop=true` 主动停车，pick/place 必须安全 ABORT；新增 `./run.sh piper-base-stop-gate` 验证该拒绝路径，防止后续机械臂真实执行与底盘运动互相干扰。
