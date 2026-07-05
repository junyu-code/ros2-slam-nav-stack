# SLAM 导航系统实现过程记录

本文档用于持续记录工程实现过程、参数调整依据和验收截图清单。平时它是开发笔记；整理报告时，可以直接抽取其中的系统架构、实验步骤、结果分析和创新点说明。

## 1. 工程定位

目标是在 Ubuntu 22.04 + ROS2 Humble + Gazebo Classic 中实现一个通用移动机器人 SLAM 导航系统，完成：

- 构建仿真环境地图。
- 指定目标点自主导航。
- 在不少于 5 个静态障碍物的场地中完成避障。
- 预留动态避障或实物模型演示等扩展方向。

工程从已有机器人比赛开发经验中抽取通用能力，但新工作区不再面向比赛任务，不包含自瞄、裁判系统、串口通信、比赛决策等模块。

## 2. 抽离原则

保留内容：

- Gazebo 仿真、移动机器人模型和 LiDAR/IMU 传感器。
- FAST_LIO，用于 LiDAR-Inertial odometry 和点云建图。
- pointcloud_to_laserscan，用于把 3D 点云压成 2D `/scan`。
- slam_toolbox，用于在线构建 2D 栅格地图。
- Nav2，用于目标点导航和静态避障。

移除内容：

- 比赛专用地图、资源和外层启动入口。
- 自瞄、串口、裁判系统、比赛行为树和比赛决策。
- 旧工程中的 `build/`、`install/`、`log/` 等生成产物。

## 3. 系统链路

仿真层：

```text
Gazebo -> mobile_robot.xacro -> /livox/lidar + /livox/imu + /cmd_vel
```

定位与建图层：

```text
/livox/lidar + /livox/imu -> FAST_LIO -> /Odometry + /cloud_registered
/cloud_registered -> pointcloud_to_laserscan -> /scan
/scan + /Odometry -> slam_toolbox -> /map + map->odom
```

导航层：

```text
/map + /scan + /Odometry -> Nav2 -> /cmd_vel -> Gazebo robot
```

## 4. 仿真场地设计

场地文件：

```text
src/slam_nav_simulation/world/nav_test_world/nav_test_world.world
```

场地由 SDF 基础几何体搭建，不依赖大型比赛地图模型。设计元素包括：

- 外围边界墙。
- 长走廊。
- 窄门。
- 两个柱状障碍。
- 两个低矮障碍。
- 一个斜坡平台。

这样可以覆盖建图、局部避障、全局规划和代价地图膨胀等关键问题，同时保持 Gazebo 加载速度较快。

## 5. 当前启动步骤

构建：

```bash
cd ~/slam_nav_ws
./build.sh
source install/setup.bash
```

启动仿真：

```bash
./start_simulation.sh
```

启动建图：

```bash
./start_mapping.sh
```

键盘控制探索：

```bash
./teleop.sh
```

保存地图：

```bash
./save_map.sh nav_test_map
```

启动导航：

```bash
./start_navigation.sh
```

## 6. 验收指标对应关系

成功构建环境地图：

- RViz 中显示 `/map`。
- `src/slam_nav_bringup/map/nav_test_map.yaml` 和 `.pgm` 保存成功。

实现指定目标点自主导航：

- RViz 中使用 `2D Goal Pose` 设置目标点。
- Nav2 输出全局路径和局部控制速度。
- 机器人到达目标点附近。

静态避障成功率不低于 80%：

- 在 5 个以上不同目标点测试中记录成功/失败。
- 至少 5 次测试中 4 次成功，或 10 次测试中 8 次成功。

地图障碍物不少于 5 个：

- 当前场地包含长墙、窄门、柱子、低矮障碍、斜坡平台等多类障碍。
- 报告截图中需要标注至少 5 个障碍物位置。

创新部分候选：

- 在 Gazebo 中添加可移动障碍物，测试局部 costmap 对动态障碍的响应。
- 调整 inflation radius、controller 参数，比较不同避障效果。
- 用实物模型搭建简化地图，复用 Nav2 地图与路径规划截图进行对比说明。

## 7. 截图清单

建议报告至少包含：

- Gazebo 测试场地总览，标注障碍物。
- RViz 中 FAST_LIO 点云或 `/cloud_registered`。
- RViz 中 slam_toolbox 生成的 `/map`。
- 地图保存后的 `.pgm` 图片。
- Nav2 全局路径和局部代价地图。
- 机器人到达目标点前后对比。
- 避障测试统计表。

## 8. 后续维护记录

- 2026-07-03：创建独立工作区 `~/slam_nav_ws`，抽取通用 SLAM/导航链路，新增通用仿真包与启动脚本。
- 2026-07-03：补齐 `livox_ros_driver2` 与 `Livox-SDK2` 依赖，移除会牵引比赛接口的自定义控制器包，改用 Nav2 标准 DWB 控制器；完成全工作区构建验证，`colcon build` 最终显示 8 个包构建成功。
- 2026-07-03：验证 `mobile_robot.xacro` 可正常生成 URDF，验证 `simulation.launch.py`、`mapping.launch.py`、`navigation.launch.py` 均可被 ROS2 launch 解析。
- 2026-07-03：处理 slam_toolbox 启动时的 `minimum laser range setting (0.0 m)` 警告。原因是 `/scan` 消息的最近量程约为 0.3-0.35 m，而 slam_toolbox 默认最小量程为 0.0 m；已在建图参数中加入 `min_laser_range: 0.35`，与点云转 LaserScan 的 `range_min` 对齐。这类警告不代表仿真雷达无数据，看到 `Registering sensor: [Custom Described Lidar]` 反而说明 `/scan` 已被 slam_toolbox 识别。
## 9. 动态障碍物扩展记录

- 场地现在分成两个版本：`nav_test_world.world` 是原始静态场地，`nav_test_world_dynamic.world` 是带动态障碍物的场地。
- 默认脚本 `./start_simulation.sh` 启动静态场地；`./start_simulation_dynamic.sh` 或 `./start_simulation.sh world:=dynamic` 启动动态场地。
- 2026-07-03：在 `slam_nav_simulation` 中新增 Gazebo Classic `ModelPlugin`：`dynamic_obstacle_plugin`。
- 在 `nav_test_world_dynamic.world` 中添加红色圆柱模型 `moving_obstacle`，轨迹为场地中部沿 y 方向往返移动，参数包括起点、终点、速度和停顿时间。
- `moving_obstacle` 带有 collision，不只是视觉动画，因此可以进入 LiDAR 扫描结果和 Nav2 costmap，可作为动态避障创新点的基础。
- 验证方式：无 GUI 加载 world 后执行 `gz model -m moving_obstacle -p`，间隔查询显示 y 坐标从约 `-2.49` 变化到约 `-0.88`，说明插件和运动轨迹正常。
- 2026-07-04：新增第二个速度更快的蓝色圆柱动态障碍物 `fast_moving_obstacle`。它在场地中部通道出口附近沿 x 方向往返移动，速度约 `0.85 m/s`，用于和原来的低速动态障碍物形成对比，便于测试 Nav2 local costmap 对不同速度障碍物的响应。
- 2026-07-04：将 `fast_moving_obstacle` 从场地上方边界附近移到中部通道出口附近，轨迹调整为 `(-0.8, 0.0)` 到 `(3.6, 0.0)`。这个位置更容易被常用导航路线遇到，同时不会直接贴墙或穿过右侧柱状静态障碍物。
