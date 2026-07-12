# cloud_relocalization

`cloud_relocalization` 提供通用点云地图辅助重定位入口，用于在机器人初始位姿不准、定位明显漂移或需要从已有 PCD 地图恢复时，把当前点云与离线点云地图进行配准。

它是部署阶段的增强模块，不参与首次建图，也不默认接管主导航 TF。第一次调试应保持 `publish_tf:=false`，只观察输出位姿和对齐点云。

当前节点：

```text
bnb_localization_node  # 2D 多分辨率 Branch-and-Bound 粗定位
icp_relocalization_node # small_gicp/ICP/GICP/NDT 三维精配准
```

大场地启动阶段不再要求机器人先运动至 AMCL 协方差收敛。机器人静止后，BnB 使用
`/scan` 在二维地图中搜索粗略 `map -> base_footprint`，bridge 将其换算为
`map -> odom` 并发送给 small_gicp。连续两次 BnB + GICP 结果一致后，bridge 才发布
权威 `map -> odom`，并通过 `/localization_ready` 放行 Nav2。AMCL 保留为二维观察器和
三维后端失败时的回退候选。

## 输入输出

输入：

```text
/map                  # BnB 使用的二维占据栅格
/scan                 # BnB 使用的当前二维扫描
/cloud_registered      # 当前注册点云，默认认为在 odom/LIO 局部坐标系下
map_pcd_path           # 离线 PCD 地图路径
```

输出：

```text
/relocalization/coarse_pose    # BnB 输出的 map 下机器人粗位姿
/relocalization/coarse_quality # BnB 分数、第二候选上界、分差和搜索量
/relocalization/status         # 配准状态
/relocalization/quality        # JSON 质量分数、fitness、重叠率和局部点数
/relocalization/pose           # 估计出的 map 下位姿
/relocalization/aligned_cloud  # 对齐后的当前点云
map -> odom TF                 # 可选，默认关闭
```

大场地桥接管理器还会发布：

```text
/localization/decision_status  # 当前后端、TF 所有者以及二维/三维质量状态
/localization_ready            # BnB + GICP bootstrap 完成后为 true
```

`quality.score` 是用于工程门控的质量分数，不是经过统计标定的真实概率。

在大场地鲁棒导航入口中，AMCL 保持运行但关闭自身 TF 广播，只输出二维候选位姿；
桥接节点是唯一的 `map -> odom` 发布者。PCD 缺失时桥接节点持续采用 AMCL，三维接管后若
连续配准失败或后端失联，则只在机器人静止、AMCL 位姿新鲜且协方差通过门限时切回二维。
回退后本次运行锁定二维模式，避免二维/三维后端反复切换。

由大场地鲁棒导航入口接管 TF 时，桥接节点默认要求连续两次重定位结果在时间窗口内
彼此一致，才更新 `map -> odom`。单次偶然成功不会直接改变全局定位。

触发服务：

```bash
ros2 service call /relocalization/coarse_trigger std_srvs/srv/Trigger {}
ros2 service call /relocalization/trigger std_srvs/srv/Trigger {}
```

地图就绪检查：

```bash
ros2 service call /relocalization/ready std_srvs/srv/Trigger {}
```

桥接节点只有在该服务确认 PCD 加载成功、过滤后点数达到门限时，才允许从 AMCL 动态跟随
切换到 FAST-LIO + 三维校正模式；AMCL 节点本身仍作为不广播 TF 的二维观察器运行。

## 配准后端

二维粗定位使用多分辨率占据概率金字塔和 best-first Branch-and-Bound。只有最高分达到
`min_score` 时才有资格继续。若相对第二候选的分差达到 `min_score_gap`，只精配准第一名；
若两个候选都达到最低分但二维结构存在歧义，bridge 会依次用 small_gicp 验证前两名，并按
三维质量选择结果。BnB 本身不发布 TF。

通过 `registration_method` 选择后端：

```text
small_gicp # 默认，使用 small_gicp 的并行 GICP 实现，适合 3D 点云重定位
icp   # 速度快，参数少，适合初值较准、结构清楚的场景
gicp  # 考虑局部几何协方差，通常比普通 ICP 更稳，但计算更重
ndt   # 基于体素正态分布，适合较粗的地图匹配，需要调 ndt_resolution
```

常用参数：

```text
registration_method           # small_gicp/icp/gicp/ndt
map_leaf_size                 # 离线地图降采样尺寸
scan_leaf_size                # 当前点云降采样尺寸
max_correspondence_distance   # ICP/GICP 最大对应距离
fitness_score_threshold       # 接受匹配的 fitness 门限
ndt_resolution                # NDT 体素分辨率
ndt_step_size                 # NDT 优化步长
small_gicp_num_threads        # small_gicp 并行线程数
small_gicp_correspondence_randomness # small_gicp 协方差估计邻居数
local_map_radius              # 围绕初值裁剪局部 PCD 地图半径
max_result_translation_jump   # 单次重定位允许的最大平移跳变
max_result_yaw_jump           # 单次重定位允许的最大 yaw 跳变
max_result_z_jump             # 地面机器人允许的最大 Z 跳变
max_result_roll_pitch_jump    # 允许的最大横滚/俯仰跳变
min_overlap_ratio             # 对齐点云与局部 PCD 的最低重叠率
```

## 启动示例

单独观察 BnB：

```bash
cd ~/slam_nav_ws
./run.sh relocalization-bnb
ros2 topic echo /relocalization/coarse_quality
ros2 service call /relocalization/coarse_trigger std_srvs/srv/Trigger {}
```

默认 small_gicp：

```bash
cd ~/slam_nav_ws
./run.sh relocalization \
  map_pcd_path:=/home/junyu/slam_nav_ws/src/FAST_LIO/PCD/scan.pcd \
  input_cloud_topic:=/cloud_registered \
  publish_tf:=false
```

GICP：

```bash
cd ~/slam_nav_ws
./run.sh relocalization-gicp \
  map_pcd_path:=/home/junyu/slam_nav_ws/src/FAST_LIO/PCD/scan.pcd \
  input_cloud_topic:=/cloud_registered
```

NDT：

```bash
cd ~/slam_nav_ws
./run.sh relocalization \
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
1. 启动仿真或实机定位前端，确认 /scan、/cloud_registered 和 /Odometry 正常。
2. 单独启动 BnB，观察 coarse_pose 和 coarse_quality，不发布 TF。
3. 确认最高分、候选分差和位姿正确后，再启动 small_gicp 精配准。
4. 在 RViz 中观察 /relocalization/aligned_cloud 是否和地图结构重合。
5. 使用大场地完整入口验证连续两次一致后 /localization_ready 才变成 true。
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
动态障碍物造成的短时碰撞
建图阶段的回环优化
长期无人值守的自动闭环重定位
```

BnB 可以覆盖整张二维地图，但在重复走廊或对称结构中可能因最高候选分差不足而拒绝；
这是安全门控，不应通过单纯降低 `min_score_gap` 绕过。

## 大场地同源建图

二维地图与 PCD 必须使用同一轨迹坐标基准。大场地专用建图配置关闭 slam_toolbox 的
扫描匹配和回环修正，让二维栅格与 FAST-LIO PCD 都固定在 `odom` 初始原点：

```bash
# 终端 1
./run.sh large-arena

# 终端 2
./run.sh large-arena-mapping

# 遥控覆盖场地后，终端 3
./run.sh save-large-arena-maps large_arena_aligned
```

生成 `large_arena_aligned.yaml/.pgm` 和 `large_arena_aligned.pcd` 后，
`./run.sh large-arena-nav` 会自动优先使用这对地图。不要把其他建图运行生成的二维地图
与当前 PCD 混用。

后续如果要把它接入自动流程，应先由 `localization_guard` 判断定位健康，再由任务层决定是否触发重定位，并且仍要保留 fitness、跳变门限和人工验证开关。
