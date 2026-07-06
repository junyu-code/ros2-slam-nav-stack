# slam_nav_piper_description

Piper 机械臂扩展的 TF/描述包。当前仓库不复制厂家 URDF；默认提供轻量占位模型，也可以在安装 AgileX 官方 `piper_description` 后显式生成官方 URDF 适配链。

## TF 约定

```text
base_link
  -> piper_mount_link
  -> piper_base_link
  -> piper_flange
  -> piper_tcp
  -> piper_arm_camera_link
  -> piper_arm_camera_optical_frame
```

真实机械臂控制阶段保持 `piper_base_link`、`piper_tcp`、`piper_arm_camera_optical_frame` 这些项目侧 frame 名称稳定。AgileX 官方原始 URDF 使用 `base_link/link1...`，项目侧适配器会把它们转换为 `piper_base_link/piper_link1...`，避免和移动底盘 `base_link` 冲突。

## 单独启动

```bash
ros2 launch slam_nav_piper_description piper_description.launch.py
```

该 launch 只发布 Piper 相关 TF，不启动 FAST-LIO2、Nav2 或 task1 主流程。

显式使用官方 URDF 适配链：

```bash
ros2 launch slam_nav_piper_description piper_description.launch.py arm_model:=official
```

## 官方模型后端

默认 launch 仍使用项目侧轻量占位模型。导入 AgileX open class 并构建后，可以显式启动官方 Piper 描述：

```bash
ros2 launch slam_nav_piper_description piper_official_description.launch.py
```

该入口会读取官方 `piper_description/urdf/piper_description.xacro`，生成项目侧 `piper_*` frame 的适配 URDF，并补充移动底盘挂载、`piper_tcp` 和腕部相机 TF。官方包未安装时会直接退出并提示缺失，不会影响默认 `piper_description.launch.py`。

这个入口首先用于对照官方关节链和 frame。官方 MoveIt2 配置原生仍引用 `base_link/link1.../joint1...`；后续要把 MoveIt2 和移动底盘组合成同一规划场景时，需要生成或维护一份对应 `piper_*` 名称的 SRDF、controller 和 kinematics 配置。

注意：如果官方 URDF 的末端 frame 不是 `piper_tcp`，启动时需要显式指定：

```bash
ros2 launch slam_nav_piper_description piper_official_description.launch.py official_tcp_parent_frame:=<官方末端frame>
```
