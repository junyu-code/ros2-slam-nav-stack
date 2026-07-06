# cloud_relocalization

`cloud_relocalization` 提供通用点云地图辅助重定位入口，用于在机器人初始位姿不准、定位明显漂移或需要从已有 PCD 地图恢复时，把当前点云与离线点云地图进行配准。

它是部署阶段的增强模块，不参与首次建图，也不默认接管主导航 TF。第一次调试应保持 `publish_tf:=false`，只观察输出位姿和对齐点云。

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
/relocalization/status         # 配准状态
/relocalization/pose           # 估计出的 map 下位姿
/relocalization/aligned_cloud  # 对齐后的当前点云
map -> odom TF                 # 可选，默认关闭
```

触发服务：

```bash
ros2 service call /relocalization/trigger std_srvs/srv/Trigger {}
```

## 配准后端

通过 `registration_method` 选择后端：

```text
icp   # 默认，速度快，参数少，适合初值较准、结构清楚的场景
gicp  # 考虑局部几何协方差，通常比普通 ICP 更稳，但计算更重
ndt   # 基于体素正态分布，适合较粗的地图匹配，需要调 ndt_resolution
```

常用参数：

```text
registration_method           # icp/gicp/ndt
map_leaf_size                 # 离线地图降采样尺寸
scan_leaf_size                # 当前点云降采样尺寸
max_correspondence_distance   # ICP/GICP 最大对应距离
fitness_score_threshold       # 接受匹配的 fitness 门限
ndt_resolution                # NDT 体素分辨率
ndt_step_size                 # NDT 优化步长
local_map_radius              # 围绕初值裁剪局部 PCD 地图半径
max_result_translation_jump   # 单次重定位允许的最大平移跳变
max_result_yaw_jump           # 单次重定位允许的最大 yaw 跳变
```

## 启动示例

普通 ICP：

```bash
cd ~/slam_nav_ws
./start_relocalization.sh \
  map_pcd_path:=/home/junyu/slam_nav_ws/src/FAST_LIO/PCD/scan.pcd \
  input_cloud_topic:=/cloud_registered \
  publish_tf:=false
```

GICP：

```bash
cd ~/slam_nav_ws
./start_relocalization_gicp.sh \
  map_pcd_path:=/home/junyu/slam_nav_ws/src/FAST_LIO/PCD/scan.pcd \
  input_cloud_topic:=/cloud_registered
```

NDT：

```bash
cd ~/slam_nav_ws
./start_relocalization.sh \
  registration_method:=ndt \
  ndt_resolution:=1.0 \
  ndt_step_size:=0.1 \
  map_pcd_path:=/home/junyu/slam_nav_ws/src/FAST_LIO/PCD/scan.pcd \
  input_cloud_topic:=/cloud_registered \
  publish_tf:=false
```

## 调试顺序

建议按下面顺序调：

```text
1. 启动仿真或实机定位前端，确认 /cloud_registered 正常。
2. 启动 cloud_relocalization，保持 publish_tf:=false。
3. 手动调用 /relocalization/trigger。
4. 在 RViz 中观察 /relocalization/aligned_cloud 是否和地图结构重合。
5. 查看 /relocalization/status 的 fitness、accepted/rejected 状态。
6. 多次验证方向正确后，再考虑 publish_tf:=true。
```

不要同时让多个节点发布同一个 `map -> odom`。如果 AMCL、静态 TF 或其他重定位模块已经在发布 `map -> odom`，本模块应继续保持 `publish_tf:=false`，只作为观测和诊断入口。

## 定位边界

这个包不替代 FAST-LIO2、AMCL 或 Nav2。它只提供“当前点云对齐到先验 PCD 地图”的辅助能力。

它适合解决的问题：

```text
初始位姿大致知道，但需要用点云地图校准
运行中怀疑定位有漂移，需要手动触发一次验证
实机部署前，想评估先验 PCD 地图和当前点云的一致性
```

它不适合直接解决的问题：

```text
完全无初值的全局定位
动态障碍物造成的短时碰撞
建图阶段的回环优化
长期无人值守的自动闭环重定位
```

后续如果要把它接入自动流程，应先由 `localization_guard` 判断定位健康，再由任务层决定是否触发重定位，并且仍要保留 fitness、跳变门限和人工验证开关。
