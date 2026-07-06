# rgbd_navigation_perception

`rgbd_navigation_perception` 是导航用 RGB-D 松耦合感知包。当前默认面向
ORBBEC / WHEELTEC Gemini Pro，但节点只依赖标准 ROS2 深度图和相机内参话题，
后续更换相机时只需要在 launch 层 remap 话题。

## 输入输出

输入：

```text
/nav_camera/depth/image_raw
/nav_camera/depth/camera_info
```

输出：

```text
/visual_obstacles
```

运行时开关服务：

```text
/rgbd_nav/set_enabled
```

## 单独启动

```bash
ros2 launch rgbd_navigation_perception depth_obstacle_projector.launch.py enabled:=true
```

关闭导航 RGB-D 感知：

```bash
ros2 service call /rgbd_nav/set_enabled std_srvs/srv/SetBool "{data: false}"
```

重新开启：

```bash
ros2 service call /rgbd_nav/set_enabled std_srvs/srv/SetBool "{data: true}"
```

## Gemini Pro 接入约定

实机驱动由 Orbbec ROS2 wrapper 单独启动。本包建议把导航相机统一 remap 到：

```text
/nav_camera/color/image_raw
/nav_camera/depth/image_raw
/nav_camera/depth/camera_info
/nav_camera/depth/points
```

Piper 机械臂相机后续应使用独立命名空间，例如 `/arm_camera/...`，避免和导航相机混用。
