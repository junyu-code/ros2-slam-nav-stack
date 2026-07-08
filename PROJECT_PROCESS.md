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

2026-07-07 维护记录：新增 `./run.sh task1-snapshot`，用于把 task1 当前证据状态写入 `tasks/task1/TASK1_STATUS_SNAPSHOT.md`。它不启动 Gazebo/RViz/Nav2，只记录默认地图、报告 PDF、必需截图、实验表、预检状态和下一步建议，方便每次完成一批截图或实验记录后留下可追踪的进度快照；`./run.sh task1-finalize` 也已接入该步骤，最终检查时会自动刷新快照。

2026-07-07 维护记录：新增 `./run.sh task1-report-audit`，专门审计结课报告中的个人信息、截图引用、缺失截图、PNG 文件有效性、待填字段和 PDF 更新时间。该检查已接入 `task1-delivery-check` 与 `task1-finalize`，用于把“报告写完了吗”和“工程能跑吗”分开检查，减少最终打包时漏图、误放空截图或 PDF 未重新编译的风险。

2026-07-07 维护记录：增强 `./run.sh task1-package-preview`，预览压缩包时会列出将进入包内的最大文件，并在估算体积超过 250MiB 时给出提醒。该功能用于最终提交前确认没有误打包 rosbag、点云、数据集、模型权重或无关外部依赖。

2026-07-07 维护记录：增强 `./run.sh task1-runtime-check`，支持 `--save` 将真实运行时的 ROS 话题、TF、Nav2 生命周期和 action 检查输出保存到 `tasks/task1/TASK1_RUNTIME_LAST.md`。该文件用于辅助填写 `EXPERIMENT_RECORD.md` 中的建图/导航运行检查表，但不替代 Gazebo/RViz 截图。

2026-07-08 维护记录：复盘实机 RViz 空白问题，确认根因不是 RViz，而是 `mapping` 只启动 FAST-LIO、`pointcloud_to_laserscan`、slam_toolbox 和 RViz，没有先启动 Livox MID360 驱动与 IMU complementary filter，导致 FAST-LIO 缺少 `/livox/lidar` 和 `/imu/data` 输入，后续 `/cloud_registered`、`/scan`、`/map` 都没有有效数据。已新增 `scripts/real_sensor_inputs.sh`，并接入 `mapping`、`auto-mapping`、`nav`、`nav-3d`、`nav-rgbd`、`nav-full` 和 `robust-nav`：入口会自动检测 `/livox/lidar`、`/livox/imu`、`/imu/data`，缺失时自动拉起 Livox 驱动和 IMU filter，已有发布者时复用现有节点。同步修复 `clean` 清理 Livox/IMU filter 残留、`mapping.launch.py` 加载 RViz 配置、`/scan` QoS 兼容性和 FAST-LIO 空地图时间戳刷屏问题。

2026-07-08 维护记录：复盘 Jetson clean build 问题。当前实机为 ARM64 环境，`uname -m` 为 `aarch64`，系统已存在可用的 Livox-SDK2 ARM64 安装：`/usr/local/lib/liblivox_lidar_sdk_shared.so` 为 aarch64，`/usr/local/include/livox_lidar_api.h` 与 `livox_lidar_def.h` 存在。因此构建不再执行额外的手动 Livox-SDK2 安装流程，而是复用系统 SDK。`livox_ros_driver2` 的 CMake 已调整为 ARM64 下优先查找系统 Livox-SDK2，避免继续链接仓库中架构绑定的 x86_64 预编译库。`scripts/build.sh` 默认并行度改为 `nproc`，并使用 colcon parallel executor，同时保留 `BUILD_JOBS` 与 `COLCON_WORKERS` 供低内存机器降档。为保证 clean checkout 能编过，还修复了多个可选资源/依赖的硬失败：`slam_nav_piper_description` 不再强制安装不存在的 `rviz` 目录，`slam_nav_bringup` 对 `report_assets` 做存在性保护，`ros2_livox_simulation` 和 `slam_nav_simulation` 在缺少 Gazebo 开发依赖时跳过插件编译、只安装资源资产。最终全量 `./run.sh build` 已验证通过，结果为 27 packages finished。

2026-07-08 后续构建治理建议：核心导航/感知包应与仿真、Gazebo、RViz 插件形成明确构建边界，避免 headless 或实机部署环境被可选 GUI/仿真依赖阻塞；仓库不应默认链接架构绑定的二进制 `.so`，Livox 这类原生库应优先通过 `find_package`、导出的 CMake target 或系统安装路径探测；CMake 中应避免 `/home/...` 形式硬编码绝对路径；可选目录统一使用 `if(EXISTS ...)` 或提交占位目录；部署前预检应覆盖 CPU 架构、Livox 头文件和 `.so` 架构、Gazebo/RViz 依赖、必需/可选资产目录、并行构建参数；长期建议增加 CI 矩阵，至少覆盖 x86_64/aarch64、clean build、有/无 Gazebo、核心包/全量构建。

2026-07-08 维护记录：实机导航链路已在 Jetson + MID360 输入下验证到 Nav2 层：`/map_server`、`/amcl`、`/planner_server`、`/controller_server`、`/bt_navigator` 均进入 active，`/scan` 与 `/Odometry` 约 10 Hz，`map -> base_footprint` TF 可用，`/compute_path_to_pose` 成功，近距离自目标 `NavigateToPose` 成功且 `/cmd_vel_safe` 保持零速。Unitree GO2 速度输出采用仓库内已有 `safe_cmd_bridge`：`Nav2 /cmd_vel -> safe_cmd_bridge -> /cmd_vel_safe + UDP 192.168.123.22:15000`，UDP payload 为 ASCII `vx,vy,wz\n`，对应 GO2 侧 `SportClient.Move(vx, vy, vyaw)`。`/home/jetson/go2` 可作为协议参考，不作为本仓库运行依赖。

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
  -> ./run.sh save-map base1
```

实机 MID360 输入链路与仿真不同，入口脚本会先保证真实传感器数据可用：

```text
Livox MID360
  -> /livox/lidar
  -> /livox/imu
  -> imu_complementary_filter
  -> /imu/data
  -> FAST-LIO2
  -> /Odometry + /cloud_registered
  -> pointcloud_to_laserscan
  -> /scan
  -> slam_toolbox / Nav2 / RViz
```

实机排障顺序固定为先数据、再 FAST-LIO、再投影和地图：`/livox/lidar` 与 `/livox/imu` 是驱动原始输入，`/imu/data` 是 FAST-LIO 使用的滤波 IMU，`/cloud_registered` 是 FAST-LIO 输出，`/scan` 是二维投影，`/map` 是 slam_toolbox 输出。若 RViz 空白，先检查这些话题频率和 `odom -> base_footprint` TF，不要优先调整 RViz。

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
  -> Nav2 + map_server(base1.yaml) + AMCL initial pose publisher
  -> /cmd_vel
  -> Gazebo robot
```

当前 `./run.sh nav` 会调用 `scripts/start_navigation.sh`，启动 FAST-LIO2、`pointcloud_to_laserscan`、Nav2 bringup 和初始位姿发布器，并默认加载 `base1.yaml`。

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

这个模块和 `localization_guard` 的关系是：`localization_guard` 判断定位是否可疑，`cloud_relocalization` 提供地图辅助校准手段。普通重定位入口仍保持 `publish_tf:=false`，用于先观察配准质量；大场地鲁棒导航入口进一步加入 `relocalization_amcl_bridge.py`，采用“AMCL 启动引导 + FAST-LIO 常态定位 + GICP 异常恢复”的结构。启动阶段 AMCL 只负责把机器人配准到全局地图；收敛后桥接节点捕获 `map -> odom`，停用 AMCL，并继续发布冻结后的 `map -> odom`。正常行驶时不让 AMCL 持续拉扯位姿，只有定位守护报警且底盘/雷达近似静止后，GICP/ICP/NDT 的结果才会更新冻结 TF。


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

### 3.6.1 大场地鲁棒导航与碰撞扰动测试

为验证系统在更大场地和外部扰动下的稳定性，新增了大场地仿真与鲁棒导航入口：

```text
./run.sh large-arena-collision
  -> large_arena_collision.world
  -> 自动机器人 + 手动扰动车

./run.sh large-arena-nav
  -> large_arena_robust_navigation.launch.py
  -> navigation.launch.py
  -> localization_guard.launch.py
  -> cloud_relocalization/icp_relocalization.launch.py
  -> relocalization_amcl_bridge.py
```

该链路使用 `large_arena.yaml` 作为 2D 先验地图，使用 `/livox/lidar/pointcloud` 投影生成 `/scan` 供 AMCL 启动配准和 Nav2 使用，同时用 `/cloud_registered` 与 `src/FAST_LIO/PCD/scan.pcd` 做 GICP 辅助重定位。重定位节点默认不发布 TF；桥接节点只在 AMCL 严格收敛后接管并冻结 `map -> odom`。后续主定位来源回到 FAST-LIO/odom，GICP 只作为碰撞、打滑或明显位姿跳变后的恢复手段。

这个设计的目的不是替代 task1 的静态 10 次避障统计，而是作为扩展能力：当机器人被手动小车碰撞、初始位姿有偏差或里程计短时漂移时，可以观察系统是否能通过先验 PCD 地图重新校准。实验截图建议包括 Gazebo 大场地、RViz 全局路径、`/amcl_converged`、`/relocalization/status` 或 `Updated frozen map->odom from relocalization` 日志，以及手动扰动车接近自动机器人前后的对比。

该链路的启动顺序要求 `/scan` 先出现。若日志中持续出现 `scan_timeout:never_seen`、`Waiting for topics: /scan` 或 `map frame does not exist`，说明 AMCL 启动引导尚未完成，桥接节点不会冻结 `map -> odom`，GICP 结果也不会提前生效。排查顺序应先看 `/livox/lidar` 自定义消息、`/livox/lidar/pointcloud` 点云、点云转 `/scan`、`scan_target_frame` 和仿真是否正常，再看重定位参数。

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
./run.sh save-map base1
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
- 成功保存 `base1.yaml` 和 `base1.pgm`。

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

- 2026-06-23 至 2026-06-24：梳理结课设计要求，明确项目必须覆盖环境地图构建、指定目标点自主导航、静态避障成功率统计、至少 5 个障碍物以及可选创新扩展；确定主线采用“先稳定建图，再加载地图导航”的路线。
- 2026-06-25 至 2026-06-26：准备 Ubuntu 22.04、ROS2 Humble、Gazebo Classic、FAST-LIO2、Nav2 和 slam_toolbox 运行环境，排查 WSL/图形化窗口、rosdep、编译线程和 FastDDS shared-memory 残留等基础问题。
- 2026-06-27 至 2026-06-29：验证仿真启动、LiDAR/IMU 话题、FAST-LIO2 输出、RViz 可视化、键盘控制和地图实时查看流程，记录 `/scan`、`/cloud_registered`、`/Odometry`、TF 与 Gazebo 窗口相关问题。
- 2026-06-30 至 2026-07-02：围绕建图和导航主链路做初步调参，确认点云到 LaserScan 的转换关系、保存地图流程、AMCL 初始位姿、Nav2 目标点发送和静态避障测试口径。
- 2026-07-03 至 2026-07-04：抽取通用 `slam_nav_ws` 工作空间，整理统一 `run.sh` 入口、脚本目录、task1 交付文档、报告草稿和动态障碍物扩展场地，减少项目中与特定场景绑定的残留表述。
- 2026-07-05 至 2026-07-06：集中修复鲁棒性问题，包括 `map -> odom` 建立、传感器时间戳、旧工作空间覆盖层污染、局部避障、行为树后退恢复、SmacPlanner2D、3D 地形增强、RGB-D 松耦合入口和点云地图辅助重定位雏形。
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
- 2026-07-06：曾尝试优化仿真场地中的斜坡模型，以降低坡面硬边缘对底盘运动、点云建图和地形分析的干扰；后续复查发现该改动会和已保存的默认地图产生版本错位，因此未作为当前 task1 默认场地保留。
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
- 2026-07-07：新增 `./run.sh task1-package-preview`，用于预览 task1 最终压缩包的包含文件、估算体积和排除规则；默认不创建文件，材料补齐后可用 `--create` 输出到 `dist/3232072072234+佘俊谕.zip`，同时将 `dist/` 和 `*.zip` 加入 `.gitignore`，避免打包产物误提交。
- 2026-07-07：新增 `tasks/task1/TASK1_EVIDENCE_TODO.md` 剩余证据采集清单，把必需截图、建图/导航运行检查、10 次静态避障实验字段、动态障碍物扩展示范和最终严格打包命令集中到一处；`task1-check` 与 `task1-delivery-check` 已将该清单纳入必备文档检查。
- 2026-07-07：修正 `mission_behavior` 的 Piper pick/place demo 自动启动逻辑，`auto_start=true` 时直接顺序调用 `run_once()`，避免依赖一次性 timer 与 `rclpy.spin()` 的退出时机；该 demo 仍只通过 `/piper/task/*` action 调用机械臂任务层，不直接接触 MoveIt2 或 SDK。
- 2026-07-07：优化 task1 LaTeX 结课报告截图接入方式，新增 `\taskfigure{文件名}{图题}{缺图提示}` 自动插图宏；后续只需按证据清单把 Gazebo/RViz 截图放入 `tasks/task1/report_latex/figures/`，重新编译后 PDF 会自动插入真实截图，缺图时仍保留占位框。
- 2026-07-07：新增 `./run.sh piper-viz` Piper RViz 可视化入口，默认启动 Piper 独立 fake runtime 并打开 `piper_visualization.rviz`，用于观察官方 URDF 适配链、TF、假腕部相机和目标位姿；可选 `start_moveit_plan:=true` 只启动 MoveIt2 plan-only 服务，仍保持不执行轨迹、不连接 SDK、不接入 task1/Nav2。
- 2026-07-07：新增 `./run.sh task1-build-report` 结课报告编译入口，优先使用 WSL `xelatex`，若不可用则自动调用 Windows TeX Live `xelatex.exe` 通过 UNC 路径编译 `tasks/task1/report_latex/main.tex` 两遍；补完截图后可先生成 `main.pdf`，再执行 strict 交付检查和打包。
- 2026-07-07：新增 `./run.sh task1-status` 无 GUI 状态页，集中显示默认地图、结课报告 PDF、必需截图、可选扩展截图、实验记录待填字段、报告占位和 Git 工作区状态，并根据最早缺失证据给出下一步运行建议，方便每次恢复任务时快速判断该先建图、导航、动态演示还是整理报告。
- 2026-07-07：将 `./run.sh piper-real-readiness` 纳入 Piper 静态验收链路，形成 OK/WAIT/FAIL 实机前状态报告；默认 WAIT 表示真实执行、SDK、手眼标定或底盘停止确认仍未满足但系统保持安全未接入状态，真正上机前可使用 `--require-ready` 把 WAIT 也作为失败。
- 2026-07-07：新增 `./run.sh task1-finalize` 最终交付编排入口，按“状态页 -> 静态避障实验记录检查 -> 报告 PDF 编译 -> task1 strict 预检 -> strict 交付检查 -> 压缩包预览/可选创建”的顺序执行；默认不启动 GUI、不创建 zip，材料补齐后使用 `--create` 生成正式包，草稿包可显式使用 `--allow-warnings`。
- 2026-07-07：新增 `./run.sh task1-experiment-check` 静态避障实验记录检查入口，自动解析 `EXPERIMENT_RECORD.md` 中 10 次静态避障表，统计已填完整行数、成功次数、碰撞次数和成功率；`--strict` 用于最终提交前确认成功率不低于 80%，`--show-rows` 用于填表后逐行核对，`--next` 用于提示下一条待补记录和推荐填写格式。该入口已接入 `task1-finalize` 和 `task1-delivery-check`。
- 2026-07-07：新增 `./run.sh task1-figures` 截图文件名辅助入口，可列出必需/可选截图状态、打印指定图号目标路径，并把已截好的 PNG 导入到 `tasks/task1/report_latex/figures/` 的标准文件名；该工具不启动 GUI、不生成假图，只减少后续截图归档和报告插图时的手工命名错误。
- 2026-07-07：新增 `./run.sh task1-sync-report` 实验记录同步入口，从 `EXPERIMENT_RECORD.md` 解析 10 次静态避障表，生成 `STATIC_TRIALS_TABLE.md` 与 `report_latex/generated_static_trials.tex`；LaTeX 报告改为 `\input{generated_static_trials.tex}`，`task1-build-report` 与 `task1-finalize` 会在编译前自动同步，避免实验记录和报告表格重复手工维护。
- 2026-07-07：增强 `./run.sh task1-runtime-check --save` 证据保存方式，除继续覆盖 `TASK1_RUNTIME_LAST.md` 作为最新快照外，同时按时间和模式归档到 `tasks/task1/runtime_checks/<时间>_<模式>.md`；`task1-status` 和 `task1-snapshot` 同步显示运行时 latest/历史快照状态，避免建图、导航、动态演示检查结果互相覆盖。
- 2026-07-07：继续做 task1 无 GUI 交付审计，确认 `task1-check` 结构错误为 0，主要剩余缺口是 9 张真实截图、运行时快照和 10 次静态避障实验记录；清理 `PROJECT_DELIVERY_GUIDE.md` 中旧式“待替换”截图示例，统一改为正式图注 + `taskfigure`/`task1-figures import` 的证据导入流程，减少最终报告中残留草稿痕迹的风险。
- 2026-07-07：将 `STUDENT_INFO.md` 与 `PROJECT_DELIVERY_GUIDE.md` 纳入 task1 无 GUI 结构预检/交付检查入口，确保个人信息、课程材料理解、报告写作规范和工程脚本检查属于同一条交付门禁链路。
- 2026-07-07：增强 `task1-experiment-check --next`，在 10 次静态避障表未填完时自动提示下一条待补记录、缺失字段、推荐表格行格式和成功/碰撞判定口径；README、Runbook、证据清单、运行截图步骤和交付清单同步改为填表阶段优先使用 `--next`，降低实测后漏填字段或写法不一致的概率。
- 2026-07-07：将 `--next` 接入 `task1-delivery-check` 与 `task1-finalize` 的静态避障实验检查调用；最终交付编排失败时会同步输出下一条待补记录和推荐格式，不需要再单独手动追查 `EXPERIMENT_RECORD.md` 的缺口。
- 2026-07-07：优化 `task1-sync-report` 的生成文件写入策略，只有 Markdown/LaTeX 静态避障表内容发生变化时才替换目标文件；重复运行同步命令不再无意义刷新 mtime，避免 `task1-report-audit` 误报“生成表格比 PDF 新”。
- 2026-07-07：增强 `task1-runtime-check` 与实验记录的对应关系，新增 `/cloud_registered` 发布者和频率检查，并在 mapping/nav/dynamic 三种模式结束时输出应填写到 `EXPERIMENT_RECORD.md` 的记录建议；`--save` 快照也会保留这些建议，方便真实运行后转写建图检查、导航检查和动态障碍物扩展示范说明。
- 2026-07-07：删除未被当前启动链路引用的旧场景地图文件，保留通用 `base1` 作为 task1 默认导航地图，减少源码层遗留场景命名对泛用项目定位的干扰。
- 2026-07-07：新增 `tasks/task2/REAL_ROBOT_DEPLOYMENT_CHECKLIST.md`，把后续实机部署拆成无硬件预检、UDP 底盘通信、传感器/TF/外参、定位健康监控、PCD 辅助重定位、低速上车流程和实地记录模板；`real-preflight` 已把该文档纳入长期维护文档检查。
- 2026-07-07：新增 `./run.sh task2-status` 无 GUI/无硬件状态页，用于检查 task2 长期文档、实机/鲁棒导航入口、扩展包目录、安全默认值、RGB-D/重定位/机械臂隔离边界，并把 task1 真实证据未完成项标成 WAIT；可追加 `--with-preflight` 顺带运行 `real-preflight`。
- 2026-07-07：扩充 task1 LaTeX 结课报告正式稿，按本科毕业设计（论文）规范补充中文宋体/英文 Times New Roman 字体设置、页脚居中页码、摘要与 Abstract、公式/图/表按章编号、一级章节另起页和三线表排版；新增 LiDAR-IMU 状态方程、点到面残差、占据栅格 log-odds、AMCL 粒子权重、代价地图膨胀、全局规划目标函数、DWB 局部控制评分、静态避障成功率公式，以及调试问题与方案改进表。PDF 已通过 `./run.sh task1-build-report` 重新生成，当前剩余真实证据缺口仍是 9 张截图和 10 次静态避障实验数据。
- 2026-07-07：复查 task1 Gazebo 场地模型时发现，后续试做的新坡道/左上角场地方案会改变已保存 `base1` 对应的环境几何，导致“场地已更新但地图仍是旧版本”的错位风险；已撤回该 world 内容改动，当前默认静态/动态场地回到 Git 基线，与现有默认地图兼容。
- 2026-07-07：将 task1 交付检查中的“地图是否过期”逻辑从单纯 mtime 判断改为“world 文件晚于地图且内容相对 Git 基线确实变化”才提醒重建图；这样恢复文件或 touch 文件不会误报，但真正修改场地时仍会提醒重新保存 `base1`。
- 2026-07-07：继续优化 task1 状态页与快照的下一步建议优先级；当 world 内容确实相对 Git 基线发生变化且晚于 `base1` 时，`task1-status` 和 `task1-snapshot` 才会把“重新建图、保存 `base1`”列为首要动作，避免因文件时间变化误导进入不必要的重扫。
- 2026-07-07：保留新坡道几何复查经验作为问题记录：坡面方向、连接缝隙和与墙体/窄门区域的空间关系都可能影响底盘运动与点云建图；但当前 task1 默认场地不启用该新坡道方案，避免和已有默认地图不一致。
- 2026-07-07：收敛 task1 交付文档口径，明确区分“当前默认地图存在且元数据检查通过”和“若后续修改 world 内容则必须重新建图、保存地图、补 GUI 截图和 10 次静态避障复测”的条件，避免把历史地图或试做场地混入最终证据。
- 2026-07-07：修复 `Ubuntu-22.04` 用户级 `~/.bashrc` 环境污染问题：移除文件开头误多出的单引号，并取消全局自动 source `~/0glut2/install/setup.bash`；当前交互式终端只自动加载 `/opt/ros/humble/setup.bash`，具体工作区覆盖层由 `slam_nav_ws/run.sh` 或工程根目录手动 `source install/setup.bash` 控制，避免多个 ROS2 工作区互相串包。
- 2026-07-07：增强 `task1-check` 的入口完整性检查，自动解析 `run.sh` 中所有映射到 `scripts/*.sh` 的命令，逐一确认目标脚本存在且可执行；这样后续新增 task2、实机部署或 Piper 相关入口时，如果忘记提交脚本或执行权限，普通 task1 预检阶段就能提前发现。
- 2026-07-07：新增 `./run.sh task1-world-check`，在不启动 Gazebo/RViz 的情况下检查静态/动态 `.world` 文件、SDF 语法、固定障碍物、动态障碍物插件和旧场地兼容性；该检查已接入 `task1-check` 与 `task1-delivery-check`，用于提前发现 world 文件缺失、插件缺失或固定障碍布局被误改等问题。
- 2026-07-07：修复动态障碍物不再让行的问题。`dynamic_obstacle_plugin.cpp` 已支持根据机器人模型距离暂停运动，但 `nav_test_world_dynamic.world` 中未显式配置 `yield_radius`，导致插件按固定轨迹往返而不会在接近机器人时停车；现已为两个动态障碍物补充 `robot_model=mobile_robot`、`yield_radius` 和 `yield_resume_radius`，并把该字段纳入 `task1-world-check`，避免后续退化成直接碰撞机器人。
- 2026-07-07：新增 `./run.sh task1-map-check`，在不启动 ROS/Gazebo 的情况下解析 `base1.yaml/pgm`，输出 resolution、origin、PGM 尺寸、地图覆盖范围、文件大小、像素统计以及地图是否晚于场地文件；该检查已接入 `task1-check`、`task1-delivery-check`、`task1-status` 和 `task1-snapshot`，用于保存地图后快速转写实验记录并避免继续沿用旧地图。
- 2026-07-07：新增大场地鲁棒导航扩展：整理 `large_arena.world`、`large_arena_collision.world`、`large_arena.yaml/pgm` 与 `teleop_manual_car`，将 `./run.sh large-arena-nav` 指向 `large_arena_robust_navigation.launch.py`。该入口组合 AMCL 启动引导、原始 LiDAR 点云投影 `/scan`、`localization_guard`、GICP/ICP/NDT PCD 重定位和 `relocalization_amcl_bridge.py`；AMCL 收敛后桥接节点冻结 `map -> odom` 并停用 AMCL，正常行驶回到 FAST-LIO/odom，只有碰撞扰动或明显漂移后才用 PCD 重定位更新冻结 TF。

## 2026-07-07 大场地 /scan 与 AMCL 初始化修复记录

大场地鲁棒导航试跑时曾出现 localization_guard fault: scan_timeout:never_seen、publish_initial_pose.py: Waiting for topics: /scan、map -> odom 不存在以及 RViz 发送目标点被拒绝的问题。复查后确认根因不是 GICP 重定位本身失效，而是 AMCL 的 LaserScan 输入链路没有建立：仿真同时发布 /livox/lidar 和 /livox/lidar/pointcloud，其中 /livox/lidar 是 livox_ros_driver2/msg/CustomMsg，不能直接作为 pointcloud_to_laserscan 的输入；真正可用于投影 /scan 的话题是 /livox/lidar/pointcloud，类型为 sensor_msgs/msg/PointCloud2。

当前修复如下：large_arena_robust_navigation.launch.py 和 scripts/start_large_arena_navigation.sh 均显式使用 /livox/lidar/pointcloud -> /scan；localization_guard_node.py 的 /Odometry、/cloud_registered、/scan 订阅改为 qos_profile_sensor_data，避免 best-effort 传感器数据被默认 reliable QoS 订阅漏接。修复后启动大场地鲁棒导航时，应先看到 /scan 有连续输出，再由初始位姿发布器触发 AMCL，随后出现 map -> odom 或 map -> base_footprint TF；若仍只看到 scan_timeout:never_seen，优先检查 /livox/lidar/pointcloud 的发布、pointcloud_to_laserscan 是否启动以及是否误用 /livox/lidar。

## 2026-07-07 最终定位链路收口

大场地增强导航链路调整为“AMCL 启动引导、FAST-LIO 常态定位、GICP 异常恢复”。AMCL 不再作为行驶过程中的长期 `map -> odom` 主发布者：启动阶段先由 `publish_initial_pose.py` 发布初始位姿并等待 `/amcl_converged=true`，随后 `relocalization_amcl_bridge.py` 捕获当前 `map -> odom`，调用 `/amcl/change_state` 停用 AMCL，并以 20 Hz 发布冻结后的 `map -> odom`。这样正常行驶时 `map -> odom` 稳定不被 AMCL 粒子更新拉动，局部连续位姿由 FAST-LIO 的 `odom -> base_footprint` 维持。

GICP/ICP/NDT 仍然默认 `publish_tf=false`，只输出 `/relocalization/pose`、`/relocalization/status` 和对齐点云。自动写回导航 TF 的条件被收紧为：`localization_guard` 已经判定定位异常，底盘 `/Odometry` 处于低速，`odom -> livox_frame` 在 1.5 s 窗口内近似静止，并且机器人仍在已知 `/map` 区域内。配准结果还必须通过 PCD 局部子图点数、fitness、最大平移/航向跳变门限。满足这些条件后，桥接节点直接更新冻结的 `map -> odom`，日志会出现 `Updated frozen map->odom from relocalization`。

AMCL 收敛判断由 `amcl_convergence_monitor.py` 给出，综合 covariance、`map -> odom` 稳定性、particle cloud 集中程度和 scan-map 残差：

```text
AMCL 收敛分数 =
  40% covariance 是否足够低
+ 25% map->odom 是否稳定
+ 20% particle_cloud 是否集中
+ 15% scan-map 残差是否足够低
```

当前严格阈值为 `xy_cov <= 0.025 m^2`、`yaw_cov <= 0.030 rad^2`、总分不低于 85 且连续稳定 2 s。测试时可用 `ros2 lifecycle get /amcl` 确认 bootstrap 后 AMCL 进入 inactive，用 `ros2 run tf2_ros tf2_echo map odom` 观察冻结 TF 是否持续发布。

## 2026-07-07 低速重定位触发门控修正

为了避免 small-gicp/GICP 在机器人尚未真正稳定时介入，当前大场地鲁棒导航入口不再把“底盘速度为 0”直接等价为“可以重定位”。`relocalization_amcl_bridge.py` 的自动触发条件调整为三重门控：`localization_guard` 已经判定定位异常、底盘 `/Odometry` 线速度和角速度低于阈值、`odom -> livox_frame` 在短时间窗口内近似静止。只有三者同时成立时，才会调用 `/relocalization/trigger`，并在配准结果通过跳变量和 fitness 门限后更新冻结的 `map -> odom`。

当前阈值在 `large_arena_robust_navigation.launch.py` 中显式维护：雷达静止窗口为 1.5 s，窗口内平移变化不超过 0.025 m，yaw 变化不超过 0.025 rad。这样可以覆盖“底盘停住但云台或雷达仍在转”的情况，避免在点云仍明显运动时启动点云地图配准，减少重定位卡顿、错误回灌和地图/点云短时错位。

观察日志时可关注 `Low-speed relocalization gate triggered`、`sensor_d` 和 `sensor_yaw`。如果日志显示 `LiDAR frame is still moving`，说明系统已经识别到雷达本体尚未静止，此时等待是预期行为，不应简单放宽 GICP 权限。
## 2026-07-07 已知地图区域门控补充

大场地鲁棒导航入口新增 known-area 门控，用于限制自动点云重定位的触发范围。`relocalization_amcl_bridge.py` 订阅 `/map`，在准备自动触发 `/relocalization/trigger` 前查询 `map -> base_footprint`，检查机器人当前位置周围是否仍属于占据栅格地图的已知区域。当前默认半径为 `0.80 m`，要求窗口内已知栅格比例不低于 `0.75`。如果机器人已经在地图边界外、靠近未知区域，或 `/map`/TF 尚未准备好，桥接节点会跳过自动触发，日志中会出现 `Skip relocalization trigger`。

这个门控不替代 GICP/ICP/NDT 自身的配准质量判断。若 AMCL 使用的 2D 栅格地图范围大于先验 PCD 地图范围，known-area 只能说明机器人仍在 AMCL 的已知栅格内；随后 `icp_relocalization_node` 仍会用 PCD 局部裁剪点数、fitness、最大平移/航向跳变量继续判断是否接受结果。当前形成两层保护：先用 `/map` 拦明显出界，再用 PCD 配准质量拦“2D 地图覆盖但 PCD 不覆盖或几何不匹配”的情况。

即使手动调用 `/relocalization/trigger` 得到了配准输出，桥接节点在更新冻结 `map -> odom` 前也会再检查 known-area 门控；未知区域中的配准结果可以用于诊断，但默认不会强行写回导航 TF。

## 2026-07-07 大场地 AMCL 启动门控回调修正

实际试跑 `./run.sh large-arena-nav` 时发现，严格等待 `/amcl_converged=true` 会把 Nav2 长时间卡在启动前：AMCL 已经能接收 RViz 的 `2D Pose Estimate` 并产生 `map -> base_footprint`，但收敛监控中的 covariance、scan-map 残差或粒子云辅助指标仍可能迟迟不满足，从而出现 `Timed out waiting for strict AMCL convergence; keeping Nav2 gated`。这会让系统无法进入后续导航，也无法通过行驶获得更多观测来改善匹配。

本次将大场地启动策略调整为“可用 TF 放行、严格分数诊断、桥接节点冻结”：`publish_initial_pose.py` 只负责等待 `/map`、`/scan`、`/Odometry` 和 AMCL 产生可用的 `map -> base_footprint`，不再把严格收敛分数作为 Nav2 启动硬门；`amcl_convergence_monitor.py` 继续发布 `/amcl_converged` 和 `/amcl_convergence_status` 作为调试参考；`relocalization_amcl_bridge.py` 在启动期捕获当前 `map -> odom` 后停用 AMCL，并由桥接节点持续发布冻结 TF，使正常行驶重新回到 FAST-LIO/odom 主定位。

同时拆分启动冻结和低速 GICP 的雷达静止门控：新增 `require_sensor_still_for_bootstrap`，大场地启动冻结不再因为 `sensor stillness window is not ready` 被卡住；低速自动重定位仍保留 `require_sensor_still_for_low_speed=true`，只有底盘低速且 `odom -> livox_frame` 在短窗口内近似静止时才允许触发点云重定位。这样启动流程更容易进入可测试状态，而异常恢复阶段仍保持较严格的安全门控。
补充：为避免 AMCL 刚收到默认初始位姿就被 bridge 立即冻结，大场地入口增加 `bootstrap_min_age_sec=15.0`。启动后前 15 秒仍可通过 RViz `2D Pose Estimate` 修正 AMCL 初始位姿；宽限期结束且底盘近似静止后，bridge 才会捕获当前 `map -> odom`、停用 AMCL，并进入 FAST-LIO 主定位 + GICP 异常恢复模式。
