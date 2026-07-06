# terrain_analysis_ext

`terrain_analysis_ext` 是二阶段扩展地形分析节点。它订阅原始注册点云、一阶段 `/terrain_map` 和 `/Odometry`，维护更大范围的滚动地形图，并输出 `/terrain_map_ext` 给全局 costmap 使用。

默认链路：

```text
/cloud_registered_body + /Odometry + /terrain_map
  -> terrain_analysis_ext
  -> /terrain_map_ext
  -> Nav2 IntensityVoxelLayer
```

一阶段更偏近场，二阶段更偏扩展范围。当前 3D 导航默认使用：

```text
local_costmap  <- /terrain_map
global_costmap <- /terrain_map_ext
```

## 启动

```bash
cd ~/slam_nav_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch terrain_analysis_ext terrain_analysis_ext.launch.py
```

常用检查：

```bash
ros2 topic hz /terrain_map
ros2 topic hz /terrain_map_ext
ros2 topic info /terrain_map_ext -v
```
