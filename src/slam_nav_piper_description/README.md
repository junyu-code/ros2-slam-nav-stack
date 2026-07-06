# slam_nav_piper_description

Piper 机械臂扩展的 TF/描述包。当前仓库不复制厂家 URDF；默认读取 AgileX 官方 `piper_description` 并生成项目侧 `piper_*` 适配链，轻量占位模型只作为缺少官方包时的显式 fallback。

## TF 约定

```text
base_link
  -> piper_mount_link
  -> piper_base_link
  -> piper_link1
  -> ...
  -> piper_link6
  -> piper_tcp
  -> piper_arm_camera_link
  -> piper_arm_camera_optical_frame
```

真实机械臂控制阶段保持 `piper_base_link`、`piper_tcp`、`piper_arm_camera_optical_frame` 这些项目侧 frame 名称稳定。AgileX 官方原始 URDF 使用 `base_link/link1...`，项目侧适配器会把它们转换为 `piper_base_link/piper_link1...`，避免和移动底盘 `base_link` 冲突。

运行时 TF 冒烟：

```bash
./run.sh piper-tf-smoke
```

该脚本只启动 Piper 官方描述和假关节状态发布器，验证关键 TF 可查询，同时确认不会发布 task1 的 `map -> odom` 或 `odom -> base_footprint` 权威链路。

## 单独启动

```bash
ros2 launch slam_nav_piper_description piper_description.launch.py
```

该 launch 默认使用官方 Piper URDF 关节链，只发布 Piper 相关 TF，不启动 FAST-LIO2、Nav2 或 task1 主流程。

需要给 Gazebo 模型挂腕部 RGB-D 插件时显式打开：

```bash
ros2 launch slam_nav_piper_description piper_description.launch.py enable_piper_gazebo_camera:=true
```

该插件的话题固定在 `/piper/arm_camera/*`，不会 remap 到 `/nav_camera/*`。普通 TF/MoveIt2 plan-only 验证默认不开这个插件，避免和 fake camera 或实机相机抢话题；WSL/headless Gazebo Classic 下深度相机插件可能不稳定，默认烟测仍使用 fake camera 验证 RGB-D 数据链路。

缺少官方包、只想做接口冒烟时可以显式退回占位模型：

```bash
ros2 launch slam_nav_piper_description piper_description.launch.py arm_model:=placeholder
```

## 官方模型后端

导入 AgileX open class 并构建后，可以用专用入口显式启动官方 Piper 描述：

```bash
ros2 launch slam_nav_piper_description piper_official_description.launch.py
```

该入口会读取官方 `piper_description/urdf/piper_description.xacro`，生成项目侧 `piper_*` frame 的适配 URDF，并补充移动底盘挂载、`piper_tcp` 和腕部相机 TF。官方包未安装时会直接退出并提示缺失；此时可临时使用 `arm_model:=placeholder` 做非运动学接口检查。

官方描述 wrapper 同样支持 `enable_piper_gazebo_camera:=true`，只用于 Gazebo 相机仿真，不代表接入真实相机或 MoveIt2 执行。

官方 MoveIt2 配置原生仍引用 `base_link/link1.../joint1...`；项目侧已经维护 `slam_nav_piper_moveit_config`，把官方关节链映射为 `piper_base_link/piper_link*/piper_joint*`，用于移动底盘组合场景的 plan-only 验证。

注意：如果官方 URDF 的末端 frame 不是 `piper_tcp`，启动时需要显式指定：

```bash
ros2 launch slam_nav_piper_description piper_official_description.launch.py official_tcp_parent_frame:=<官方末端frame>
```
