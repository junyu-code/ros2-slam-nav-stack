# cloud_relocalization

`cloud_relocalization` 提供通用点云地图重定位入口，用于在机器人初始位姿不准、定位明显漂移或需要从已有 PCD 地图恢复时，使用当前点云和离线点云地图做 ICP 匹配。

当前节点：

```text
icp_relocalization_node
```

## 输入输出

输入：

```text
/cloud_registered      # 当前注册点云，默认认为在 odom/LIO 局部坐标系下
map_pcd_path           # 离线 PCD 地图路径
```

输出：

```text
/relocalization/status         # 匹配状态
/relocalization/pose           # ICP 估计出的 map 下位姿
/relocalization/aligned_cloud  # 对齐后的点云
map -> odom TF                 # 可选，默认关闭
```

触发服务：

```bash
ros2 service call /relocalization/trigger std_srvs/srv/Trigger {}
```

## 启动示例

```bash
ros2 launch cloud_relocalization icp_relocalization.launch.py \
  map_pcd_path:=/home/junyu/slam_nav_ws/src/FAST_LIO/PCD/scan.pcd \
  input_cloud_topic:=/cloud_registered \
  publish_tf:=false
```

首次调试建议先使用 `publish_tf:=false`，只观察 `/relocalization/pose`、`/relocalization/status` 和 RViz 中的 `/relocalization/aligned_cloud`。确认匹配方向和坐标系正确后，再允许发布 `map -> odom`。

当前节点会围绕初值裁剪局部 PCD 子图，并同时检查 ICP fitness、局部地图点数和位姿跳变门限。这样可以降低相似结构场景中错误匹配被直接接受的概率。

## 定位边界

这个包是部署阶段的重定位增强，不参与首次建图。它不替代前端里程计，也不负责回环优化；它的职责是提供一个可触发、可验证、可独立调参的地图辅助重定位入口。
