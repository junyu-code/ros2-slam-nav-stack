# Ubuntu 22.04 ROS2 SLAM Navigation System

## 根目录入口

根目录只保留统一导航入口：

```bash
cd ~/slam_nav_ws
./run.sh
```

也可以直接带命令运行：

```bash
./run.sh clean --dry-run
./run.sh sim-static
./run.sh mapping
./run.sh auto-mapping
./run.sh nav
./run.sh diagnose --duration 5
./run.sh task1-status
./run.sh task1-snapshot
./run.sh task1-check
./run.sh task1-world-check
./run.sh task1-map-check
./run.sh task1-runtime-check nav
./run.sh task1-experiment-check --next
./run.sh task1-figures
./run.sh task1-sync-report
./run.sh task1-delivery-check
./run.sh task1-package-preview
./run.sh task1-report-audit
./run.sh task1-build-report
./run.sh task1-finalize
./run.sh task2-status
./run.sh real-preflight
```

实际脚本统一收纳在 `scripts/` 目录，例如 `scripts/start_navigation.sh`。日常建议优先使用 `./run.sh <命令>`，这样根目录更干净，也不需要记住每个脚本文件名。

实机 MID360 建图/导航入口已经带传感器自检。`mapping`、`auto-mapping`、`nav`、`nav-3d`、`nav-rgbd`、`nav-full` 和 `robust-nav` 会先检查 `/livox/lidar`、`/livox/imu` 和 `/imu/data` 是否已有发布者；没有时自动启动 Livox MID360 驱动和 IMU complementary filter，再启动 FAST-LIO、`/scan` 投影、slam_toolbox/Nav2 和 RViz。若你已经手动运行了 `./run.sh livox-mid360` 或 `./run.sh imu-filter`，入口会复用现有数据源，不会重复启动端口。

实机 RViz 空白时，先按数据链路排查，而不是先改 RViz：

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic hz /imu/data
ros2 topic hz /cloud_registered
ros2 topic hz /scan
ros2 topic hz /map
ros2 run tf2_ros tf2_echo odom base_footprint
```

正常情况下，MID360 点云约 7-10 Hz，IMU 和 `/imu/data` 约 200 Hz，`/cloud_registered` 和 `/scan` 约 10 Hz，`/map` 约 1 Hz。若 `/cloud_registered` 没有频率，说明 FAST-LIO 没拿到 LiDAR/IMU 输入；若 `/cloud_registered` 有而 `/scan` 没有，再看 `pointcloud_to_laserscan` 和 TF。

Task1 课程作业的最终运行、截图、静态/动态场地区分和打包清单已经集中到：

```text
tasks/task1/TASK1_FINAL_RUNBOOK.md
```

后续跑验收时优先看这份 Runbook，再回到本文档查系统背景。
实际截图、实验记录和最终严格检查的剩余证据项集中在：

```text
tasks/task1/TASK1_EVIDENCE_TODO.md
```

如果你希望把“当前还缺什么、下一步该跑什么”保存成一份可继续维护的记录，可以运行：

```bash
./run.sh task1-snapshot
```

它会生成或覆盖 `tasks/task1/TASK1_STATUS_SNAPSHOT.md`。这个快照不会启动 Gazebo、RViz 或 Nav2，适合每次跑完一批截图/实验后更新一次，后续写报告时也能直接看出实验推进过程。

如果 `./run.sh task1-status` 或 `./run.sh task1-map-check` 提示 world 内容相对 Git 基线确实发生变化，并且晚于默认地图，说明当前 `base1` 可能不再对应当前场地。此时先按下面路线重新执行静态建图并保存 `base1`，再继续导航截图和 10 次静态避障统计。当前默认静态/动态场地已经恢复到与 `base1` 兼容的版本；单纯文件时间变化不会再要求重扫图。

如果只是想在打开 Gazebo 前确认场地文件本身没有明显几何错误，可以先运行：

```bash
./run.sh task1-world-check
```

它只检查 `.world` 文件和斜坡/动态障碍物结构，不启动 GUI，也不替代实际建图、截图和导航测试。

重新执行 `./run.sh save-map base1` 后，可运行下面命令查看地图分辨率、origin、PGM 尺寸、覆盖范围、文件大小和像素统计，方便转写到实验记录：

```bash
./run.sh task1-map-check
```

当前最短验收路线是：

```bash
cd ~/slam_nav_ws
./run.sh task1-check
./run.sh task1-world-check
./run.sh task1-map-check
./run.sh task1-status
./run.sh clean --dry-run
./run.sh clean
./run.sh sim-static
# 另开终端：./run.sh mapping
# 另开终端：./run.sh teleop
# 建图确认：./run.sh task1-runtime-check mapping
./run.sh save-map base1
./run.sh clean --dry-run
./run.sh clean
./run.sh sim-static
# 另开终端：./run.sh nav
# 导航确认：./run.sh task1-runtime-check nav
```

随后在 RViz 中完成 10 次静态目标点导航测试，把结果写入 `tasks/task1/EXPERIMENT_RECORD.md`，截图放到 `tasks/task1/report_latex/figures/`。动态障碍物、RGB-D 和 3D 地形链路只作为扩展演示，不计入静态避障 80% 成功率。

如果你想直接体验当前“最终增强版”链路，可以先不用于 80% 静态避障统计，只作为扩展截图和鲁棒性观察：

```bash
cd ~/slam_nav_ws
./run.sh clean
./run.sh sim-dynamic-rgbd
```

另开终端：

```bash
cd ~/slam_nav_ws
./run.sh nav-full
```

稳定后保存一次运行时快照：

```bash
cd ~/slam_nav_ws
./run.sh task1-runtime-check dynamic --save
```

这条链路会同时叠加动态障碍、RGB-D 松耦合近场感知、3D 地形增强和增强导航参数，负载最大，适合观察最终效果；如果出现定位被撞偏、点云和地图错位或局部代价地图误判，应如实记录为扩展链路现象，不要把它混入静态 10 次避障成功率。


### 大场地鲁棒导航与重定位扩展

如果想在更大的场地中观察当前“最强稳定版”链路，可以使用大场地入口。该入口不是 task1 静态避障 80% 成功率的主数据来源，而是用于展示鲁棒性扩展、碰撞扰动恢复和先验点云地图辅助重定位。

终端 1 启动大场地碰撞扰动仿真：

```bash
cd ~/slam_nav_ws
./run.sh clean
./run.sh large-arena-collision
```

终端 2 启动大场地鲁棒导航：

```bash
cd ~/slam_nav_ws
./run.sh large-arena-nav
```

终端 3 可选启动手动小车，用来模拟外部碰撞或遮挡扰动：

```bash
cd ~/slam_nav_ws
./run.sh teleop-manual-car
```

`large-arena-nav` 当前会调用 `large_arena_robust_navigation.launch.py`，组合 AMCL 启动配准、原始 LiDAR 点云投影 `/scan`、定位健康监控、GICP/ICP/NDT PCD 重定位和 `map -> odom` 冻结桥。流程是：启动阶段先用 AMCL 对齐全局位姿；`/amcl_converged` 达到严格收敛后，`relocalization_amcl_bridge.py` 捕获当前 `map -> odom`，请求停用 AMCL，并以固定频率继续发布冻结后的 `map -> odom`。正常导航阶段主要依赖 FAST-LIO 输出的 `odom -> base_footprint`，避免 AMCL 在行驶过程中持续拉扯位姿。

大场地入口依赖 `/scan` 先正常出现。如果日志中持续出现 `scan_timeout:never_seen`、`Waiting for topics: /scan` 或 `map frame does not exist`，说明 AMCL 启动配准还没有拿到 LaserScan，桥接节点也不会接管 `map -> odom`。这时先检查大场地仿真是否已经启动、`/livox/lidar` 自定义消息和 `/livox/lidar/pointcloud` 点云是否有数据，以及点云转 `/scan` 的输入话题是否正确；不要急着放宽 GICP 门限。

10 次实验表只需要优先填写 `EXPERIMENT_RECORD.md`。填完或修改后运行：

```bash
./run.sh task1-sync-report
```

它会从实验记录生成 `tasks/task1/STATIC_TRIALS_TABLE.md` 和 `tasks/task1/report_latex/generated_static_trials.tex`，LaTeX 报告会自动引用后者，避免在实验记录和报告中重复手改 10 行数据。

跑完截图或准备打包前，可以先执行无 GUI 交付预检：

```bash
cd ~/slam_nav_ws
./run.sh task1-status
./run.sh task1-snapshot
./run.sh task1-world-check
./run.sh task1-map-check
./run.sh task1-check
./run.sh task1-experiment-check --next
./run.sh task1-sync-report
./run.sh task1-report-audit
./run.sh task1-delivery-check
```

这些命令都不会启动 Gazebo、RViz 或 Nav2。`task1-status` 是最短状态页，会告诉你当前还缺哪些截图、实验记录、报告 PDF 或 Git 提交；`task1-world-check` 专门检查静态/动态 `.world` 文件、斜坡连接、坡向、动态障碍物插件和两份场地静态部分的一致性；`task1-map-check` 专门检查 `base1.yaml/pgm` 的分辨率、origin、PGM 尺寸、覆盖范围、文件大小、像素统计和地图是否比场地旧；`task1-check` 主要检查入口脚本、默认地图、task1 文档、报告源文件、截图文件和实验记录占位，并会顺带调用场地与地图检查；`task1-experiment-check` 会解析 `EXPERIMENT_RECORD.md` 中 10 次静态避障实验表，统计成功次数、碰撞次数和是否达到 80% 成功率；普通模式下，缺截图和待填实验记录只会显示 warning。填表过程中追加 `--next` 可以显示下一条待补实验记录、缺失字段和推荐填写格式。

当仿真、建图或导航已经启动后，可以把运行时检查结果保存成可转写到实验记录的快照：

```bash
./run.sh task1-runtime-check mapping --save
./run.sh task1-runtime-check nav --save
```

默认会更新 `tasks/task1/TASK1_RUNTIME_LAST.md`，同时归档到 `tasks/task1/runtime_checks/<时间>_<模式>.md`。这些快照记录 ROS 话题、TF、Nav2 生命周期和 action 检查结果，适合填写 `EXPERIMENT_RECORD.md` 的“运行检查/导航检查”表格，但不能替代 GUI 截图。

截图保存完成后，可以用统一入口检查或导入到报告要求的标准文件名：

```bash
./run.sh task1-figures
./run.sh task1-figures path 6-1
./run.sh task1-figures import 6-1 /mnt/c/Users/32149/Pictures/gazebo.png
```

`task1-figures` 不会启动 GUI，也不会生成假截图；它只负责列出缺图、打印目标路径，或把你已经截好的 PNG 复制到 `tasks/task1/report_latex/figures/`。

`task1-report-audit` 专门审计结课报告侧材料：姓名学号、截图引用、截图文件是否存在、PNG 文件是否有效、报告待填字段、PDF 是否比源文件旧。`task1-delivery-check` 更偏向打包视角：它会列出建议压缩包名、必须包含的源码/文档/报告材料、仍缺的截图、实验记录待填字段，以及 Git 中是否误跟踪了 `build/`、`install/`、`log/`、rosbag、点云或模型权重等重型产物。最终打包前建议执行：

`task1-delivery-check` 和 `task1-finalize` 会在静态避障实验表未填完时自动显示 `--next` 提示，指出下一条该补哪一次、缺哪些字段和推荐填写格式。

```bash
./run.sh task1-check --strict
./run.sh task1-experiment-check --strict
./run.sh task1-report-audit --strict
./run.sh task1-delivery-check --strict
```

如果只想预览最终压缩包会包含什么，不创建文件：

```bash
./run.sh task1-package-preview
./run.sh task1-package-preview --list
```

预览会同时打印将进入压缩包的最大文件列表，用来确认没有误打包 rosbag、点云、数据集或模型权重。

材料全部补齐后，可用下面命令创建压缩包到 `dist/`；`dist/` 和 `*.zip` 已加入 `.gitignore`，不会误提交：

```bash
./run.sh task1-build-report
./run.sh task1-package-preview --create
```

也可以使用最终交付编排入口自动串起状态检查、报告编译、strict 检查和压缩包预览：

```bash
./run.sh task1-finalize
```

材料全部补齐后创建正式压缩包：

```bash
./run.sh task1-finalize --create
```

如果只是想提前生成草稿包，可使用 `./run.sh task1-finalize --allow-warnings --create`；正式提交前仍应回到不带 `--allow-warnings` 的严格流程。

这是一个面向 Ubuntu 22.04 + ROS2 Humble + Gazebo Classic 的通用移动机器人 SLAM 与自主导航工作区。当前主目标是稳定完成仿真建图、保存地图、加载地图导航、目标点到达和静态避障验证。

项目长期会继续扩展 RGB-D 深度相机、语义识别、行为树和机械臂。

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
    slam_nav_piper_description/ # Piper 官方 URDF 适配链、底盘挂载和腕部相机 TF
    slam_nav_piper_perception/  # Piper 独立 RGB-D 感知，使用 /piper/arm_camera/*
    slam_nav_piper_control/     # Piper MoveIt2/SDK 控制边界和安全 owner 管理
    slam_nav_piper_manipulation/# Piper pick/place 任务 action server
    slam_nav_piper_calibration/ # Piper 腕部 RGB-D 手眼标定配置和安全检查
    slam_nav_piper_learning/    # Piper 后续学习/强化学习策略层，默认不接入
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
./run.sh build
source install/setup.bash
```

## 主流程 1：建图

建图阶段只使用原始稳定链路，不接入 `perception_adapter`。

终端 1 启动静态验收场地仿真：

```bash
cd ~/slam_nav_ws
./run.sh sim-static
```

WSL 中 Gazebo Classic 有时启动较慢，看到下面日志后再继续启动建图或导航更稳：

```text
SpawnEntity: Successfully spawned entity [mobile_robot]
```

终端 2 启动 FAST-LIO2、点云转 `/scan`、slam_toolbox：

```bash
cd ~/slam_nav_ws
./run.sh mapping
```

终端 3 键盘控制机器人探索场地：

```bash
cd ~/slam_nav_ws
./run.sh teleop
```

如果不想手动键盘探索，可以用自动探索建图入口替代手动 `mapping + teleop`：

```bash
cd ~/slam_nav_ws
./run.sh auto-mapping
```

该入口会启动 FAST-LIO2、`pointcloud_to_laserscan`、slam_toolbox、RViz 和 `auto_explore_mapper`。自动探索节点只使用 `/scan` 做保守巡航：前方安全时慢速前进，遇到近距离障碍时后退/转向，周期性原地旋转补全点云视角。它适合懒人扫静态地图，但不是完整 frontier explorer；如果发现某个角落没有扫到，仍然可以临时关闭它后用 `teleop.sh` 补扫。

保存地图：

```bash
cd ~/slam_nav_ws
./run.sh save-map base1
```

保存 FAST-LIO 累计出的 PCD 地图：

```bash
cd ~/slam_nav_ws
./run.sh save-pcd nav_test_static
```

默认 PCD 会写入：

```text
src/FAST_LIO/PCD/scan.pcd
src/FAST_LIO/PCD/nav_test_static.pcd
```

保存结果应位于：

```text
src/slam_nav_bringup/map/base1.yaml
src/slam_nav_bringup/map/base1.pgm
```

## 主流程 2：加载地图导航

导航阶段不需要继续运行建图链路。先清理旧进程，再启动同一张静态验收场地和导航。

终端 1：

```bash
cd ~/slam_nav_ws
./run.sh clean
./run.sh sim-static
```

终端 2：

```bash
cd ~/slam_nav_ws
./run.sh nav
```

当前 `./run.sh nav` 会调用 `scripts/start_navigation.sh` 并启动：

```text
FAST-LIO2
pointcloud_to_laserscan -> /scan
Nav2 bringup
map_server 加载 base1.yaml
publish_initial_pose.py 初始化 AMCL
RViz
```


当前默认 `nav` 规划链路：

```text
全局规划：planner_server / GridBased / SmacPlanner2D
局部路径跟踪：controller_server / FollowPath / DWBLocalPlanner
局部障碍物输入：local_costmap / obstacle_layer / /scan
全局静态地图与障碍物输入：global_costmap / static_layer + obstacle_layer
速度平滑：velocity_smoother
恢复行为：behavior_server + 行为树 BackUp/ClearCostmap/Spin
```

增强入口 `nav-3d` 和 `nav-full` 会切换到 `pb_omni_pid_pursuit_controller::OmniPidPursuitController`，并叠加 LiDAR 地形分析、强度体素代价地图、RGB-D 近场障碍物或安全桥等扩展能力。课程验收主线优先使用默认 `nav`，增强入口用于创新展示和后续实机调试。
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

正常情况下应输出 `/opt/ros/humble`。如果输出 `/home/junyu/0glut2/...` 等旧工作区路径，说明当前终端环境混入了旧覆盖层。项目脚本已经在启动时清理 `AMENT_PREFIX_PATH`、`COLCON_PREFIX_PATH`、`CMAKE_PREFIX_PATH` 和 `ROS_PACKAGE_PATH`，遇到这种情况时执行 `./run.sh clean` 后重新从 `~/slam_nav_ws` 运行启动脚本。

若 RViz 中发送目标点后出现 `Planning algorithm GridBased failed to generate a valid path`，优先确认机器人当前位置和目标点都在白色空旷区域内，并与障碍物黑边或灰色膨胀区保持一定距离。机器人贴近障碍物时，起点可能已经落入代价地图膨胀区，规划器会合理地拒绝生成路径。

Livox Mid-360 传感器已设置为 `always_on`，无 GUI 模式也可以发布 `/livox/lidar`。作业截图和交互调试建议使用 `./run.sh sim-static`，动态障碍物扩展示范再使用 `./run.sh sim-dynamic`。

静态测试场地中的斜坡已经从单个倾斜长方体调整为“入口引导板 + 缓坡 + 顶部平台”的组合，坡度更缓，入口碰撞边缘更低；2026-07-07 进一步修正了缓坡俯仰方向，避免入口侧被抬高、平台侧反而降低。这样更适合自动探索和 PCD 建图，也方便后续观察 3D 地形代价地图对坡面/平台的响应。


## 增强流程：3D 地形代价地图导航

默认导航链路主要依赖 `/scan`，它稳定、容易调试，适合当前作业验收。为了后续提升复杂障碍环境下的鲁棒性，项目新增了两级 LiDAR 地形分析和强度体素代价地图入口：

```bash
cd ~/slam_nav_ws
./run.sh nav-3d
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

## 增强流程：RGB-D 松耦合近场避障

RGB-D 深度相机已经以松耦合方式接入 local costmap：深度图先由 `depth_obstacle_projector` 投影为 `/visual_obstacles`，再作为 `ObstacleLayer` 的额外 PointCloud2 观察源。默认 3D 导航不启用这一路，方便对比纯 LiDAR 3D 与 LiDAR + RGB-D。

终端 1 启动带深度相机的仿真：

```bash
cd ~/slam_nav_ws
./run.sh sim enable_nav_rgbd_camera:=true
```

终端 2 启动 RGB-D 增强导航：

```bash
cd ~/slam_nav_ws
./run.sh nav-rgbd
```

对应配置：

```text
src/slam_nav_bringup/config/nav2_params_3d_rgbd.yaml
start_navigation_rgbd.sh
```

当前融合边界：

```text
/nav_camera/d435i/depth/image_rect_raw + /nav_camera/d435i/depth/camera_info
  -> depth_obstacle_projector
  -> /visual_obstacles
  -> local_costmap obstacle_layer
```

`/visual_obstacles` 只接入 local costmap，用于近距离动态感知和视觉补盲；global costmap 仍主要依赖静态地图、LiDAR 地形分析和 `/scan`，避免视觉误检长期污染全局路径。

## 顶配流程：动态障碍 + RGB-D + 3D 鲁棒导航

如果要展示当前仿真中能组合起来的最完整配置，使用下面两个脚本：

终端 1 启动动态障碍物场景，并给机器人挂载导航 RGB-D 相机：

```bash
cd ~/slam_nav_ws
./run.sh sim-dynamic-rgbd
```

终端 2 启动完整导航链路：

```bash
cd ~/slam_nav_ws
./run.sh nav-full
```

该入口等价于组合以下能力：

```text
动态障碍物 world
  + /livox/lidar 与 /livox/imu
  + FAST-LIO2 /Odometry 与 /cloud_registered
  + terrain_analysis /terrain_map 与 /terrain_map_ext
  + IntensityVoxelLayer 3D 地形代价地图
  + /scan LaserScan 障碍物兜底
  + RGB-D depth_obstacle_projector -> /visual_obstacles
  + SmacPlanner2D 全局规划
  + OmniPidPursuitController 全向路径跟踪
  + 行为树后退恢复
  + localization_guard 定位健康监控
  + safe_cmd_bridge 速度安全桥观测
```

注意：动态障碍物不写入保存地图，也不作为 global costmap 的长期障碍。它主要通过 `/scan`、`/terrain_map` 和 `/visual_obstacles` 进入 local costmap，用于实时避障。这样可以避免动态物体污染全局静态路径，同时保留近场避障响应。

## 增强流程：鲁棒导航入口

`start_robust_navigation.sh` 是面向后续长期开发的统一增强入口，它会启动 3D 点云导航、定位健康监控和速度安全桥：

```bash
cd ~/slam_nav_ws
./run.sh robust-nav
```

默认组合：

```text
navigation_3d.launch.py
localization_guard.launch.py
safe_cmd_bridge.launch.py
```

默认情况下，速度安全桥只把 `/cmd_vel` 整形成 `/cmd_vel_safe`，不会改变 Gazebo 底盘仍然订阅 `/cmd_vel` 的事实；定位健康监控也只发布状态，不主动停车。这让增强入口适合先做观察和调试。进入真实底盘部署时，可以逐步打开：

```bash
./run.sh robust-nav \
  publish_zero_on_fault:=true \
  safe_enable_fault_stop:=true \
  safe_enable_udp_output:=true \
  safe_udp_host:=192.168.123.22
```

当前作业演示仍建议优先使用 `./run.sh nav` 或 `./run.sh nav-3d`，`./run.sh robust-nav` 更适合后续实机级鲁棒性测试。

进入实机联调前，先做一次无 GUI、无硬件预检：

```bash
cd ~/slam_nav_ws
./run.sh real-preflight
```

该入口不会启动 Gazebo、RViz、Nav2，也不会连接底盘或机械臂。它会检查 `safe_cmd_bridge` 默认是否关闭 UDP 输出、是否开启定位故障停车、速度限幅和超时参数是否合理；检查 `localization_guard`、`cloud_relocalization`、RGB-D 松耦合链路、Piper/task1 隔离边界和网络信息。真正上车前可使用严格模式：

```bash
./run.sh real-preflight --strict
```

严格模式会把 warning 也视为未通过，适合在打开 UDP 输出或真实控制后端前做最后确认。


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
./run.sh safe-bridge
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

Unitree GO2 不需要依赖外部 `/home/jetson/go2` 工作区发送速度；本仓库已有 `safe_cmd_bridge` 的独立 UDP 输出。推荐链路如下：

```text
Nav2 /cmd_vel
  -> safe_cmd_bridge
  -> /cmd_vel_safe + UDP 192.168.123.22:15000
  -> Unitree GO2 侧 UDP 接收/运动控制进程
  -> unitree_sdk2py SportClient.Move(vx, vy, vyaw)
```

先只观察 topic 输出，不让底盘动：

```bash
cd ~/slam_nav_ws
./run.sh safe-bridge enable_udp_output:=false enable_fault_stop:=true
```

确认 `/cmd_vel_safe` 的限速、超时停车和方向都正确后，再打开 Unitree GO2 UDP 输出：

```bash
./run.sh safe-bridge \
  input_topic:=/cmd_vel \
  output_topic:=/cmd_vel_safe \
  enable_fault_stop:=true \
  enable_udp_output:=true \
  udp_host:=192.168.123.22 \
  udp_port:=15000 \
  max_vx:=0.2 max_vy:=0.1 max_wz:=0.3
```

UDP 包格式为 ASCII `vx,vy,wz\n`，与 Unitree GO2 侧常见 `SportClient.Move(vx, vy, vyaw)` 接收器兼容。UDP 没有握手，必须确认 GO2 侧接收器已经运行，且 `ping -c 2 192.168.123.22` 正常。联调时先看 `ros2 topic info /cmd_vel -v`、`ros2 topic info /cmd_vel_safe -v` 和 `/cmd_vel_safe` 的零速/限速表现，再发送近距离 Nav2 目标点。

真实底盘联调前，先按 `tasks/task2/REAL_ROBOT_DEPLOYMENT_CHECKLIST.md` 的顺序执行无硬件预检、低速 topic 输出、UDP 输出和反馈看门狗测试。不要直接把仿真导航速度接到真实底盘。

如果底盘能提供实际里程计反馈，可以打开反馈看门狗，检查“持续发送速度但底盘没有响应”或“底盘反馈断流”的情况：

```bash
./run.sh robust-nav \
  safe_enable_feedback_watchdog:=true \
  safe_feedback_topic:=/base/odom
```

反馈异常时安全桥会发布：

```text
/base_feedback_fault
/base_feedback_health
```

## 可选流程：PCD 地图辅助重定位

`src/cloud_relocalization/` 提供一个可触发的点云地图重定位入口，用于部署阶段处理初始位姿不准、局部漂移或需要从已保存 PCD 地图恢复的情况。它支持 `icp`、`gicp` 和 `ndt` 三种配准后端，默认只发布状态、估计位姿和对齐点云，不直接接管 `map -> odom`。

在大场地鲁棒导航入口中，AMCL 只负责启动期全局配准；收敛后桥接节点冻结 `map -> odom`，正常行驶交给 FAST-LIO 维护局部连续位姿。GICP/ICP/NDT 不周期运行，也不会在车体仍明显运动时强行接管；只有 `localization_guard` 判断出现明显定位异常，并且底盘和 `odom -> livox_frame` 都进入低速/近似静止状态后，才会触发一次 PCD 辅助重定位，成功结果会直接更新冻结的 `map -> odom`。若只单独运行 `./run.sh relocalization` 或 `./run.sh relocalization-gicp`，仍建议先观察 `/relocalization/status`、`/relocalization/pose` 和 `/relocalization/aligned_cloud`，确认稳定后再考虑闭环接入。

```bash
cd ~/slam_nav_ws
./run.sh relocalization \
  map_pcd_path:=/home/junyu/slam_nav_ws/src/FAST_LIO/PCD/scan.pcd \
  input_cloud_topic:=/cloud_registered \
  publish_tf:=false
```

GICP 便捷入口：

```bash
cd ~/slam_nav_ws
./run.sh relocalization-gicp \
  map_pcd_path:=/home/junyu/slam_nav_ws/src/FAST_LIO/PCD/scan.pcd \
  input_cloud_topic:=/cloud_registered
```

NDT 可直接通过参数切换：

```bash
./run.sh relocalization \
  registration_method:=ndt \
  ndt_resolution:=1.0 \
  map_pcd_path:=/home/junyu/slam_nav_ws/src/FAST_LIO/PCD/scan.pcd \
  publish_tf:=false
```

触发一次匹配：

```bash
ros2 service call /relocalization/trigger std_srvs/srv/Trigger {}
```

建议先在 RViz 中观察 `/relocalization/aligned_cloud` 和 `/relocalization/pose`，确认匹配方向正确后，再显式使用 `publish_tf:=true`。当前实现会围绕初始位姿裁剪局部 PCD 子图，并同时检查 fitness、局部地图点数和位姿跳变门限，避免相似场景下错误匹配把坐标系突然拉偏。

三种后端的取舍：

```text
ICP：速度快、参数少，适合初值较准的场景。
GICP：利用局部几何协方差，通常比普通 ICP 更稳，但计算更重。
NDT：适合体素化先验地图，需要调 ndt_resolution 和初值。
```

注意不要同时让多个节点发布同一个 `map -> odom`。默认增强导航入口中，`icp_relocalization_node` 保持 `publish_tf:=false`，只输出配准结果；真正写回导航 TF 的动作由 `relocalization_amcl_bridge.py` 统一管理。普通观测模式或单独调试时，也应先保持 `publish_tf:=false`。

## 可选流程：定位健康监控

`src/localization_guard/` 是面向实机部署和长期运行新增的定位健康监控层。它订阅 `/Odometry`、`/cloud_registered` 和 `/scan`，检测断流、速度异常和位姿跳变，并发布统一健康状态。

```bash
cd ~/slam_nav_ws
./run.sh guard
```

默认只监控，不接管控制。输出话题：

```text
/localization_health
/localization_fault
/diagnostics
```

实机保守模式可以打开故障零速度输出：

```bash
./run.sh guard publish_zero_on_fault:=true
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
./run.sh diagnose --duration 5
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
./run.sh clean
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

当前 MoveIt2 核心规划接口已经安装；项目侧 plan-only 还需要 OMPL planner 和 simple controller manager 插件：

```text
ros-humble-moveit-ros-planning-interface 2.5.9
ros-humble-moveit-planners-ompl
ros-humble-moveit-simple-controller-manager
```

可用下面命令确认：

```bash
ros2 pkg prefix moveit_ros_planning_interface
ros2 pkg prefix moveit_ros_move_group
ros2 pkg prefix moveit_core
ros2 pkg prefix moveit_planners_ompl
ros2 pkg prefix moveit_simple_controller_manager
```

推荐系统安装：

```bash
sudo apt-get install ros-humble-moveit-planners-ompl ros-humble-moveit-simple-controller-manager
```

如果当前终端不能 sudo，可以使用 Piper 专用本地 overlay；它只把 deb 解到 `external/`，不会改系统，也不会影响 task1 默认环境：

```bash
./run.sh setup-piper-moveit
./run.sh piper-preflight
```

Piper 全链路烟测：

```bash
./run.sh piper-full-smoke
```

它会顺序运行安全配置检查、边界检查、依赖预检、官方 frame 审计、MoveIt2 配置映射审计、手眼标定配置检查、运行时 TF 链、runtime 命名空间图、控制桥安全边界、实机入口 dry-run 安全拒绝、headless Gazebo 组合模型、fake 感知 + pick/place action、Piper RViz 可视化配置、移动操作组合入口、mission_behavior 到 Piper action 边界、学习层候选排序、任务层 ranked 候选显式消费门禁、任务层 MoveIt2 plan-only 缺失服务拒绝门禁、任务层 MoveIt2 plan-only 通过门禁、MoveIt2 plan-only。全部都在 `/piper` 边界内验证，不接入 task1 导航主链路。

只检查实机前安全默认值：

```bash
./run.sh piper-safety-check
```

该检查会确认 `auto_enable=false`、`allow_real_motion=false`、`real_backend_connected=false`、`sdk_driver_ready=false`、`moveit_execution_ready=false`、初始限速比例不超过 `0.10`、workspace 上下界有效，并确认默认 Nav2 costmap 禁止接入 Piper 感知/学习话题。

只检查 Piper 是否保持在独立边界内：

```bash
./run.sh piper-boundary-check
```

该检查会确认 `enable_piper_arm` 与 `enable_nav_rgbd_camera` 默认关闭、默认 robot description 不含 Piper、显式打开 Piper 时使用官方 `piper_joint1...piper_joint8` 适配链且不含占位关节，并确认 task1/Nav2 默认入口没有引用 `/piper`。

只检查 Piper 运行时 TF：

```bash
./run.sh piper-tf-smoke
```

该入口会实际查询 `base_link -> piper_base_link`、`piper_base_link -> piper_tcp`、`piper_tcp -> piper_arm_camera_optical_frame`，并确认独立 Piper TF 图没有发布 `map -> odom`、`odom -> base_footprint` 或 `nav_camera` frame。

只检查 Piper runtime 命名空间图：

```bash
./run.sh piper-namespace-smoke
```

该入口会启动 `piper_sim`，等待 `/piper/arm_camera/*`、`/piper/perception/*`、`/piper/grasp_candidates`、`/piper/control/state` 和 `/piper/task/*` action 出现，并确认运行图里没有 `/nav_camera`、costmap、Nav2 action 或 AMCL/Nav2 节点。

先看 Gazebo 里的底盘 + Piper 臂：

```bash
cd ~/slam_nav_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
./run.sh sim enable_piper_arm:=true
```

这个开关默认关闭；不传 `enable_piper_arm:=true` 时，现有仿真机器人保持 task1 默认形态。显式打开 Piper 时，默认 `piper_arm_model:=official`，会使用 AgileX 官方 Piper URDF 适配链；缺少官方包、只做接口冒烟时可传 `piper_arm_model:=placeholder` 临时退回占位模型。

如果要让 Gazebo 也发布 Piper 腕部 RGB-D 相机话题，显式打开独立开关：

```bash
./run.sh sim enable_piper_arm:=true enable_piper_gazebo_camera:=true
```

该插件只发布 `/piper/arm_camera/color/*`、`/piper/arm_camera/depth/*` 和可选点云，不 remap 到 `/nav_camera/*`，也不会成为 Nav2 默认 costmap 观测源。

注意：该 Gazebo RGB-D 插件是显式仿真开关，当前不纳入默认 headless 全链路烟测；WSL/headless Gazebo Classic 下深度相机插件可能不稳定。日常验证 RGB-D 感知链路仍使用 `./run.sh piper-task-smoke` 的 fake camera。

不打开 GUI、只做一次自动验证：

```bash
./run.sh piper-gazebo-smoke
```

该入口会在独立 `ROS_DOMAIN_ID` 和 `GAZEBO_MASTER_URI` 下 headless 启动静态 Gazebo 场地，检查 `/robot_description` 中是官方 `piper_joint*` 适配链，并确认 Gazebo 中已生成 `mobile_robot` 实体；结束后自动清理本次仿真进程。它默认不打开 Gazebo 腕部相机插件，避免 headless Gazebo Classic 深度相机不稳定影响基础模型验收。

占位 fallback：

```bash
./run.sh sim enable_piper_arm:=true piper_arm_model:=placeholder
```

官方适配链会把 AgileX 原始 `base_link/link1...` 重命名为项目侧 `piper_base_link/piper_link1...`，避免和移动底盘 `base_link` 冲突；官方 MoveIt2 v4/v5 demo 仍作为独立 wrapper 启动，不会自动接入 task1。

Piper 冒烟启动：

```bash
cd ~/slam_nav_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
./run.sh piper-sim
```

该入口只启动 Piper TF、假腕部 RGB-D 相机、目标位姿估计、控制桥和 fake pick/place action。默认 TF 使用官方 Piper URDF 适配链；缺少官方包时可加 `arm_model:=placeholder` 做占位冒烟。项目侧上层接口保持 `/piper/task/pick_object` 和 `/piper/task/place_object`。

想直接看模型、TF、假腕部相机和目标位姿：

```bash
./run.sh piper-viz
```

该命令会打开 RViz，并默认同时启动 Piper 独立 fake runtime；它能显示官方 Piper 适配链、腕部相机图像、debug 检测框、目标 pose 和 `/piper/visualization/grasp_candidates` 抓取候选 marker。它不启动 Nav2、不接 SDK、不执行 MoveIt2 轨迹，RViz 配置里也不提供 `/initialpose` 或 `/goal_pose` 这类 Nav2 交互工具。需要把项目侧 MoveIt2 plan-only 也一起开起来观察时，再显式使用：

```bash
./run.sh piper-viz start_moveit_plan:=true
```

该模式仍保持 `allow_trajectory_execution=false`。

不打开 GUI、只检查 Piper RViz 配置和可视化边界：

```bash
./run.sh piper-viz-smoke
```

该检查会确认 RViz 只订阅 `/piper/robot_description`、`/piper/arm_camera/color/image_raw`、`/piper/perception/debug_image`、`/piper/perception/target_pose` 和 `/piper/visualization/grasp_candidates` 等 Piper 专用话题，不引用 `/nav_camera`、`/goal_pose`、`/initialpose` 或 costmap。

一键验证 fake 感知和任务 action：

```bash
./run.sh piper-task-smoke
```

该入口会在独立 `ROS_DOMAIN_ID` 下启动 `piper_sim`，等待 `/piper/arm_camera/*`、`/piper/perception/detections_2d`、`/piper/perception/detections_3d`、`/piper/perception/debug_image`、`/piper/perception/target_pose`、`/piper/grasp_candidates` 和 `/piper/visualization/grasp_candidates`，并确认抓取候选继承 3D detection 的 id/class/score/tag，然后向 `/piper/task/pick_object` 和 `/piper/task/place_object` 各发送一次 fake goal。它已经完成无 GUI 冒烟验证，只检查项目侧任务边界，不启动 Nav2、不连接真实 SDK。

一键验证移动操作组合入口：

```bash
./run.sh piper-mobile-sequence
```

该入口会在独立 `ROS_DOMAIN_ID` 下启动 `piper_mobile_manipulation.launch.py start_description:=true fake_camera:=true fake_execution:=true publish_base_stop:=true`，等待假相机、目标位姿、抓取候选和 action server，然后调用 pick/place，并确认任务层发布过零速度 `/cmd_vel` 停车意图。它不启动 Nav2、不连接 SDK，也不改变 task1 默认入口。

一键验证 `mission_behavior` 调用 Piper action 边界：

```bash
./run.sh piper-mission-demo
```

该入口会先启动 Piper fake runtime，再启动 `mission_behavior piper_pick_place_demo.launch.py`。demo 只调用 `/piper/task/pick_object` 和 `/piper/task/place_object`，不直接碰 MoveIt2、SDK 或厂家话题。

一键验证控制桥安全服务和 owner 边界：

```bash
./run.sh piper-control-smoke
```

该入口会在独立 `ROS_DOMAIN_ID` 下启动控制桥，验证 `/piper/control/enable`、`disable`、`estop`、`clear_estop`、`home` 服务发现，以及 `moveit/disabled` owner 状态切换。它不连接 MoveIt2 执行器、SDK 或真实机械臂。

一键验证实机入口默认安全拒绝：

```bash
./run.sh piper-real-dry-run
```

该入口会启动 `piper_real.launch.py` 的默认安全配置，确认 `auto_enable=false`、`real_backend_connected=false` 时，`home` 服务失败，`/piper/task/pick_object` 和 `/piper/task/place_object` 都返回安全拒绝，不会假装真实执行成功。

项目侧 MoveIt2 plan-only 配置已经独立放在 `slam_nav_piper_moveit_config`，默认不接入 task1、不执行轨迹、不连接 SDK：

```bash
./run.sh piper-moveit-plan
```

该入口使用项目侧 `piper_base_link/piper_joint*/piper_tcp` 配置和假关节状态发布器，并会自动加载 `external/ros_humble_debs/overlay` 里的 Piper 专用本地 MoveIt2 插件。当前已验证 `move_group` 能加载 OMPL，并输出 `You can start planning now!`。

只审计 MoveIt2 配置和官方 AgileX 映射，不启动 ROS 节点：

```bash
./run.sh piper-official-frame-audit
./run.sh piper-moveit-config
```

`piper-official-frame-audit` 会解析 AgileX 官方 `piper_description`，并确认项目侧适配链已经生成 `piper_base_link`、`piper_tcp`、`piper_arm_camera_optical_frame`，避免把官方原生 `base_link/link*/joint*` 直接混入移动底盘。

该检查会渲染官方 Piper URDF 适配链，确认 `piper_joint1` 到 `piper_joint8`、`piper_tcp` 和腕部相机 frame 存在，并核对项目侧 SRDF、`joint_limits.yaml`、`ros2_controllers.yaml`、`moveit_controllers.yaml` 与 AgileX 官方 `piper_moveit_config_v5` 的映射一致。

只检查腕部 RGB-D 手眼标定配置边界，不启动相机或移动机械臂：

```bash
./run.sh piper-hand-eye-check
```

该检查确认标定输入只来自 `/piper/arm_camera/*`，标定服务/结果留在 `/piper/calibration/*`，frame 使用 `piper_base_link`、`piper_tcp`、`piper_arm_camera_optical_frame`，默认不允许 live motion、不发布最终 TF、不写 URDF，样本和结果默认放入已忽略的 `datasets/piper_hand_eye/`。

验证真实 pick 路径会强制等待手眼标定验收：

```bash
./run.sh piper-hand-eye-gate
```

该检查会让控制桥进入 `enabled=true owner=moveit`，并让任务层声明 `real_backend_connected=true`，但保持 `hand_eye_calibrated=false`。此时 `/piper/task/pick_object` 必须返回 ABORT，拒绝原因包含“手眼标定”，防止真实后端接入后绕过 eye-in-hand 验收。

验证真实机械臂动作前必须先确认底盘停止，或由任务层显式发布停车：

```bash
./run.sh piper-base-stop-gate
```

该检查会让任务层声明 `real_backend_connected=true`、`hand_eye_calibrated=true`，但保持 `base_stop_confirmed=false` 且 `publish_base_stop=false`。此时 `/piper/task/pick_object` 必须返回 ABORT，拒绝原因包含“底盘停止”或“导航暂停”，防止后续机械臂真实执行时底盘仍在运动。

另开终端可以发送一次 plan-only 规划请求，用于确认 `/piper/plan_kinematic_path` 服务、`piper_arm` planning group、关节目标约束和 OMPL pipeline 是连通的：

```bash
./run.sh piper-plan-test
```

该测试只检查 MoveIt2 是否能返回非空轨迹，不执行轨迹、不连接 SDK，也不接入 task1 默认导航链路。

一键启动 MoveIt2 plan-only、等待服务、发送规划请求并清理本次测试进程：

```bash
./run.sh piper-moveit-smoke
```

验证任务层 action 会显式通过 MoveIt2 plan-only 门禁：

```bash
./run.sh piper-task-moveit-gate
```

该入口启动项目侧 MoveIt2 plan-only，再启动 Piper fake 任务链并显式打开 `require_moveit_plan_before_fake_execution:=true`。随后通过 `/piper/task/pick_object` 和 `/piper/task/place_object` 触发规划请求，确认 action 成功前已从 `/piper/plan_kinematic_path` 拿到非空轨迹；它仍不执行轨迹、不连接 SDK、不改变 task1 默认导航链路。默认配置保持 `require_moveit_plan_before_fake_execution=false`。

反向验证 MoveIt2 plan-only 服务缺失时必须安全拒绝：

```bash
./run.sh piper-task-moveit-gate-fail
```

该入口不启动 MoveIt2，只启动任务层并打开同一个 plan-only 门禁；pick/place 必须返回失败，原因包含“等待规划服务超时”。这防止后续把门禁打开后，在 MoveIt2 未启动时仍然 fake 成功。

想同时看 Gazebo 模型并跑 Piper 假感知/假执行，可以先开 `./run.sh sim enable_piper_arm:=true`，再另开终端运行：

```bash
source /opt/ros/humble/setup.bash
source ~/slam_nav_ws/install/setup.bash
ros2 launch slam_nav_piper_bringup piper_mobile_manipulation.launch.py use_sim_time:=true fake_camera:=true fake_execution:=true
```

这个组合入口默认 `start_description:=false`，会复用已启动的整车仿真/实机 TF，避免重复发布 `robot_state_publisher`。脱离 Gazebo 单独跑时再显式加 `start_description:=true publish_joint_states:=true`。

如果你在 GUI/本机 Gazebo 环境中确认 `enable_piper_gazebo_camera:=true` 已经稳定发布腕部 RGB-D，相机节点可以保持关闭，只运行感知和任务层：

```bash
ros2 launch slam_nav_piper_bringup piper_mobile_manipulation.launch.py use_sim_time:=true fake_camera:=false fake_execution:=true
```

外部依赖先只记录在 `piper_external.repos`，不并入主仓库：

```bash
mkdir -p external
vcs import external < piper_external.repos
```

AgileX Piper 课程参考已记录在外部清单里：

```text
https://github.com/agilexrobotics/agilex_open_class/tree/master/piper
```

它里面的 `piper_description`、`piper_moveit_config_v4`、`piper_moveit_config_v5` 作为官方模型和 MoveIt2 示例来源。Piper 项目侧入口默认读取官方 `piper_description`，但没有把官方 demo 接入 task1，也没有默认启动 MoveIt2 执行控制器。

只准备官方 Piper open class 包时，可以使用：

```bash
./run.sh setup-piper
source install/setup.bash
```

该脚本默认只下载 AgileX open class 的 `piper/` 子目录中三个官方包，目标目录是 `external/agilex/agilex_open_class`，可重复执行并跳过已完成文件。GitHub API 额度用尽时，可等待重置后重跑，或显式让脚本自动等待：

```bash
PIPER_OPEN_CLASS_WAIT_RATE_LIMIT=1 ./run.sh setup-piper
```

当前工作区已补齐官方 `piper_description` 的 `base_link.STL` 和 `link1.STL` 到 `link8.STL`，预检中 AgileX open class 下载目录应显示 66 个文件。

Piper 预检：

```bash
./run.sh piper-preflight
./run.sh piper-official-frame-audit
ros2 run slam_nav_piper_bringup piper_preflight_check.py
ros2 run slam_nav_piper_bringup piper_preflight_check.py --require-official
ros2 run slam_nav_piper_bringup piper_official_frame_audit.py
ros2 run slam_nav_piper_bringup piper_official_frame_audit.py --check-project-adapter
```

`./run.sh piper-preflight` 和 `./run.sh piper-official-frame-audit` 会自动加载当前工作区环境；直接 `ros2 run` 时只检查当前 shell 已 source 的环境。

导入并构建官方包后，可以单独跑官方 demo wrapper：

```bash
ros2 launch slam_nav_piper_bringup piper_official_moveit_demo.launch.py
ros2 launch slam_nav_piper_bringup piper_official_gazebo_demo.launch.py
```

官方 MoveIt2/RViz demo wrapper 还需要：

```bash
sudo apt-get install ros-humble-moveit-configs-utils ros-humble-moveit-ros-visualization
```

官方描述适配入口：

```bash
ros2 launch slam_nav_piper_description piper_official_description.launch.py
```

后续可能使用强化学习时，先单独启动学习层做抓取候选排序冒烟：

```bash
./run.sh piper-learning-smoke
ros2 launch slam_nav_piper_learning piper_learning.launch.py enable_learning:=true policy_backend:=heuristic
```

`piper-learning-smoke` 会发布 3 个假抓取候选，确认 `/piper/learning/grasp_candidates_ranked` 按分数排序并带上学习后端标签。默认任务层不会消费 ranked 输出，训练数据、模型权重、checkpoint 和 rosbag 也都被 `.gitignore` 排除，避免把 GitHub 仓库撑大。

显式验证任务层消费 ranked 候选：

```bash
./run.sh piper-ranked-gate
```

该入口只启动任务层并手动发布 ranked 候选，确认 `use_ranked_grasp_candidates:=true` 时 pick 会使用 ranked pre-grasp；默认配置仍保持 `false`。

担心后续 Piper、相机数据或强化学习产物把仓库撑大时，运行：

```bash
./run.sh piper-size-check
```

该检查会确认 `external/`、`datasets/`、`models/`、`checkpoints/`、`runs/`、`weights/`、rosbag、点云、ONNX/PT 等权重文件没有进入 Git 跟踪；当前历史里已有的非 Piper 大文件只作为 warning 提示。

日常改 Piper 配置后，先跑轻量静态验收：

```bash
./run.sh piper-static-check
```

它不会启动 Gazebo、Nav2、MoveIt2 `move_group` 或真实硬件，只检查安全默认值、task1 隔离、仓库体积边界、官方 URDF/MoveIt2 映射、手眼标定边界，以及 `piper-viz`、`piper-real` launch 参数能正常展开。

查看实机接入前还差哪些条件：

```bash
./run.sh piper-real-readiness
```

该报告默认只汇总状态，不要求真实执行 ready；当前阶段会把 `real_backend_connected=false`、手眼标定结果缺失、底盘停止确认未满足等列为 WAIT。真正准备接实机前可用 `./run.sh piper-real-readiness --require-ready` 让这些 WAIT 变成失败门禁。

实机入口默认不会假装执行真实机械臂：`piper_real.launch.py` 里 `real_backend_connected=false`，可先用 `./run.sh piper-real-dry-run` 验证默认拒绝路径；只有 MoveIt2/SDK 后端完成隔离验证后才显式打开。


## 常见问题：FastDDS 共享内存端口锁

如果终端出现类似日志：

```text
[RTPS_TRANSPORT_SHM Error] Failed init_port fastrtps_port7421: open_and_lock_file failed
```

通常不是 FAST-LIO2 或 Nav2 算法错误，而是 FastDDS/FastRTPS 在 WSL 或异常退出后留下了共享内存端口锁。处理方式是先停止当前流程，再清理残留：

```bash
cd ~/slam_nav_ws
./run.sh clean
```

不要在 ROS/Gazebo 正在运行时手动删除 `/dev/shm/fastrtps_*` 或 `sem.fastrtps_*`，否则可能打断正在通信的节点。`clean.sh` 会先终止本工作区相关进程，再清理 FastDDS/FastRTPS 的共享内存残留。

## 后续长期扩展

长期规划放在：

```text
tasks/task2/FUTURE_ROADMAP.md
tasks/task2/REAL_ROBOT_DEPLOYMENT_CHECKLIST.md
tasks/task2/ROBUST_NAVIGATION_UPGRADE_PLAN.md
tasks/task2/PIPER_MOBILE_MANIPULATION.md
```

恢复后续开发状态时，先看无 GUI 状态页：

```bash
cd ~/slam_nav_ws
./run.sh task2-status
./run.sh task2-status --with-preflight
```

它会检查长期文档、实机/鲁棒导航入口、RGB-D/重定位/安全桥/机械臂边界，并提醒 task1 真实证据是否还没补齐。

包括：

- 鲁棒导航升级路线；
- 实机部署前 UDP、传感器、TF、外参、重定位和安全桥检查；
- RGB-D 深度相机近场感知；
- 本地语义任务理解；
- 行为树任务决策；
- 机械臂抓取与放置；
- 是否评估 Fast-LIVO2 等紧耦合视觉-激光-惯性算法。
