# ROS2 SLAM 与导航工作空间

本仓库运行于 Ubuntu 22.04、ROS2 Humble 和 Gazebo Classic，包含建图、定位、导航、地形感知、Web/Qt 操作界面和移动操作扩展。

## 使用边界

- 在 WSL 终端中执行 Git、构建和运行命令，不要用 Windows Git 直接操作 WSL 共享路径。
- 用户入口统一为 `./run.sh <命令>`；`scripts/` 保存实现脚本，`src/` 保存 ROS2 包。
- `build/`、`install/`、`log/`、`dist/` 和 `artifacts/` 都是本地产物，不进入 Git。
- `tasks/` 包含课程报告、实验记录和个人材料，仅保留在本地并由 Git 整目录忽略。

## 快速开始

```bash
cd ~/slam_nav_ws
./run.sh build
./run.sh help
```

清理残留进程：

```bash
./run.sh clean --dry-run
./run.sh clean
```

## 导航

二维静态地图导航：

```bash
# 终端 1
./run.sh clean
./run.sh sim-static

# 终端 2
./run.sh nav
```

增强入口使用 `nav-3d` 或 `nav-full`。两者使用 footprint-aware MPPI 局部控制器，实时读取 local costmap 中的激光、地形和可选 RGB-D 障碍代价。

增强导航的底盘运动学和速度限制通过 profile 选择，不需要复制整套 Nav2 参数：

```bash
# 当前全向底盘，默认 profile
./run.sh nav-full base_profile:=omni

# 普通差速轮式底盘，强制 vy=0
./run.sh nav-full base_profile:=diff_drive

# Go2 保守限制
./run.sh nav-full base_profile:=go2

# 自定义底盘 profile
./run.sh nav-full base_profile_file:=/absolute/path/my_robot.yaml
```

profile 同时覆盖 MPPI 运动模型、机器人 footprint、velocity smoother 和 `safe_cmd_bridge` 的速度、加减速度、死区与超时参数。内置配置和字段说明见 `src/slam_nav_bringup/config/base_profiles/`。

## 建图与地图

```bash
# 终端 1
./run.sh clean
./run.sh sim-static

# 终端 2
./run.sh mapping

# 终端 3
./run.sh teleop
```

完成覆盖后保存地图：

```bash
./run.sh save-map nav_test_map
./run.sh save-pcd nav_test_static
./run.sh task1-map-check
```

保存后的三态栅格地图可以做保守去噪；默认输入和输出分别为
`large_arena.yaml` 与 `large_arena_filtered.yaml`：

```bash
./run.sh preprocess-map
# 或显式指定输入和输出
./run.sh preprocess-map path/to/input.yaml path/to/output.yaml
```

处理会保留原地图，填充小型封闭孔洞、修补单栅格断边，并生成同名
`*_diff.png` 差异图。红色表示新增障碍格，绿色表示删除的孤立障碍格。
橙色表示被恢复为未知区的细自由射线或孤立自由点。

## UI 与诊断

```bash
./run.sh ui
./run.sh operator
./run.sh diagnose --duration 5
```

`./run.sh ui` 启动统一的 **SLAM Nav Web 主界面**，默认地址为 `http://127.0.0.1:8765`。界面借鉴并适配了 GUI_go2 的地图和航点交互，并以 2×2 方式同时显示导航 RGB-D 与 Piper 腕部 RGB-D 四路视野；项目名称、流程和配置均以 SLAM Nav 为准。旧命令 `nav-ui`、`dashboard`、`mission-control`、`navigation-ui` 和 `nav-console` 仍进入同一服务。

主界面的“专业界面”入口可以启动 Qt/RViz Operator。即使从另一台电脑远程访问 Web 页面，Qt/RViz 窗口也会出现在运行本工作区的主机桌面，而不是远程浏览器所在电脑；仍可用 `./run.sh operator` 直接启动。

服务默认只监听 `127.0.0.1`。需要在可信局域网访问时必须显式执行：

```bash
./run.sh ui --host 0.0.0.0
```

该端口可以向 Nav2 下发控制命令，不应直接暴露到公网。UI 实现、环境变量和上游来源说明保留在 `ui/navigation/`。

## 实机预检

无硬件预检：

```bash
./run.sh task2-status
./run.sh real-preflight
```

真实底盘 UDP 输出默认关闭。任何实机接管都必须先完成传感器、TF、定位、安全桥、低速 dry-run 和急停验证。

## 目录职责

| 路径 | 职责 |
|---|---|
| `run.sh` | 统一命令入口 |
| `scripts/` | 启动、诊断、检查和交付脚本 |
| `src/` | ROS2 包与第三方算法包 |
| `tasks/` | 仅本地保存的课程、实验与个人材料；不进入 Git |
| `ui/` | Web 和 Qt/RViz 操作界面 |
| `external/` | 不进入主仓库的外部依赖 |

## 文档维护规则

- 根 README 只维护稳定入口和目录边界，不记录截图数量、实验成功率、个人信息或日期流水账。
- 课程报告、实验事实和个人证据只维护在本地 `tasks/`，不得加入 Git 暂存区。
- 实机能力的“完成”必须说明验证层级，不能用文档或目录存在代替运行验证。
- 开发过程由 Git 提交记录，不另维护重复的过程日志 Markdown。
