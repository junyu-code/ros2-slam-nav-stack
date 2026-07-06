# terrain_analysis

`terrain_analysis` 是导航部署阶段的一阶段 LiDAR 地形分析节点。它订阅 FAST-LIO2 注册点云和里程计，维护机器人附近的滚动地形体素图，估计局部地面高度，并把点相对地面的高度差写入 PointCloud2 的 `intensity` 字段。

默认链路：

```text
/cloud_registered_body + /Odometry
  -> terrain_analysis
  -> /terrain_map
  -> terrain_analysis_ext
```

它不参与首次建图。首次建图仍使用 FAST-LIO2、`pointcloud_to_laserscan` 和 `slam_toolbox` 的稳定链路；地形分析主要用于加载地图后的导航避障和后续实机部署。

## 启动

```bash
cd ~/slam_nav_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch terrain_analysis terrain_analysis.launch.py
```

常用检查：

```bash
ros2 topic hz /cloud_registered_body
ros2 topic hz /terrain_map
ros2 topic info /terrain_map -v
```

常用可调项：

```bash
ros2 launch terrain_analysis terrain_analysis.launch.py \
  input_cloud_topic:=/cloud_registered_body \
  odometry_topic:=/Odometry \
  output_terrain_topic:=/terrain_map
```
