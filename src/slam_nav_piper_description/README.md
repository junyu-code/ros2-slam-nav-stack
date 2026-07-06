# slam_nav_piper_description

Piper 机械臂扩展的 TF/描述包。当前仓库不复制厂家 URDF，只提供一个轻量占位模型，用来验证移动底盘挂载、腕部相机和项目侧话题命名。

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

真实机械臂控制阶段应接入 AgileX 官方 `agx_arm_urdf`/MoveIt2 配置，并保持 `piper_base_link`、`piper_tcp`、`piper_arm_camera_optical_frame` 这些项目侧 frame 名称稳定。

## 单独启动

```bash
ros2 launch slam_nav_piper_description piper_description.launch.py
```

该 launch 只发布 Piper 相关 TF，不启动 FAST-LIO2、Nav2 或 task1 主流程。
