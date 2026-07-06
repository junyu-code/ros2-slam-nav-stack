# perception_adapter

`perception_adapter` 是一个松耦合感知适配层，用于把 FAST-LIO2 输出的三维点云整理成更适合导航使用的点云。它既可以交给 `pointcloud_to_laserscan` 转成 `/scan`，也可以直接作为 Nav2 `VoxelLayer` 的 PointCloud2 观测源。

当前实现的节点：

```text
adaptive_cloud_filter
```

## 输入输出

输入：

```text
/cloud_registered  # FAST-LIO2 注册后的点云
/Odometry          # FAST-LIO2 里程计，用于估计机器人速度
```

输出：

```text
/cloud_nav_filtered  # 经过 ROI 和体素滤波后的导航点云
/nav_obstacle_cloud  # 按高度带分离出的导航障碍点云，用于后续 3D costmap 输入评估
/nav_ground_cloud    # 按高度带分离出的地面点云，用于可视化和调试
/perception_mode     # 当前感知模式：DETAIL / NORMAL / FAST / SAFE
```

## 模式逻辑

节点根据机器人速度和点云状态选择感知模式：

| 模式 | 触发条件 | 策略 |
|---|---|---|
| `DETAIL` | 低速 | 使用较小体素，保留更多建图细节 |
| `NORMAL` | 常规速度 | 使用中等体素，平衡细节和实时性 |
| `FAST` | 高速 | 使用较大体素，降低点云数量和延迟 |
| `SAFE` | 点云过少或前方障碍密集 | 使用安全策略参数，便于后续接入降速/恢复行为 |

当前节点只发布模式、过滤后的点云和可选的地面/障碍分割点云，不直接控制 Nav2，也不修改 FAST-LIO2 内核。

注意：`/cloud_registered` 通常在 FAST-LIO2 的全局/里程计坐标系下，不是雷达自身坐标系。为了避免在全局坐标下误删远离地图原点的墙体，当前默认关闭适配层内部的高度/距离 ROI 裁剪，只保留体素降采样。导航高度和距离筛选继续由 `pointcloud_to_laserscan` 在 `livox_frame` 下完成。

地面/障碍分割当前采用简单高度带，默认把 `[-0.15, 0.06] m` 视为地面候选，把 `[0.08, 1.20] m` 视为障碍候选。它的定位是导航部署阶段的可视化和后续 3D costmap 输入评估，不作为首次建图依赖。后续如果要上实机或复杂地形，应替换为地面拟合、法向量判断或时空体素层中的 marking/clearing 分流。

当前默认体素参数采用保守策略：低速 `DETAIL=0.03 m`，常规 `NORMAL=0.08 m`，高速 `FAST=0.18 m`。这样优先保证建图鲁棒性，再在高速场景下体现降采样带来的实时性优化。

## 使用定位

`perception_adapter` 不建议用于首次建图阶段。建图阶段应优先保证地图完整性和稳定性，继续使用原始链路：

```bash
cd ~/slam_nav_ws
./run.sh mapping
```

`perception_adapter` 的定位是部署/导航阶段的松耦合感知接口。地图构建完成后，它可以接在 FAST-LIO2 与 Nav2 感知输入之间，用于根据机器人速度和任务状态调整导航点云处理策略，并为后续 RGB-D 深度相机、语义识别和行为树决策预留接口。

当前已经提供 3D 点云代价地图导航入口：

```bash
cd ~/slam_nav_ws
./run.sh nav-3d
```

该入口会同时拉起 LiDAR 地形分析、强度体素代价地图和 `/scan` 兜底链路。`/nav_obstacle_cloud`、`/nav_ground_cloud` 和 `/cloud_nav_filtered` 主要用于 RViz 可视化、诊断和后续 costmap 输入评估；默认导航不再把完整 `/cloud_nav_filtered` 直接作为障碍源，避免地面点或历史点云污染局部代价地图。

当前可以单独启动节点做话题验证：

```bash
ros2 launch perception_adapter adaptive_cloud_filter.launch.py
```

后续应在导航部署 launch 中接入，而不是在建图 launch 中接入。

## 常用检查

```bash
ros2 topic hz /cloud_registered
ros2 topic hz /cloud_nav_filtered
ros2 topic hz /nav_obstacle_cloud
ros2 topic hz /nav_ground_cloud
ros2 topic echo /perception_mode
ros2 topic hz /scan
```

如果 `/cloud_nav_filtered`、`/nav_obstacle_cloud`、`/nav_ground_cloud` 有频率，且 `/perception_mode` 能输出 `DETAIL`、`NORMAL`、`FAST` 或 `SAFE`，说明适配层正在工作。也可以直接运行：

```bash
cd ~/slam_nav_ws
./run.sh diagnose --duration 5
```

## 后续扩展接口

这个包后续可以继续扩展为更完整的感知适配层：

```text
RGB-D depth -> visual_obstacles -> local_costmap
RGB image -> semantic_objects -> behavior tree
behavior tree -> perception_mode / speed limit / recovery action
```

当前阶段先保持简单，重点服务后续部署阶段扩展和结课报告中的系统可扩展性说明，不参与首次建图主链路。
