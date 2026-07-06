# 模型与导航几何对齐记录

本文档记录仿真机器人模型、传感器坐标系和 Nav2 footprint 之间的对应关系，避免后续调参时只改其中一处导致建图、定位和避障表现不一致。

## 当前模型

模型文件：

```text
src/slam_nav_simulation/urdf/mobile_robot.xacro
```

关键坐标系：

```text
base_footprint -> base_link -> imu_link
                         -> livox_frame
                         -> nav_camera_link -> nav_camera_optical_frame
```

当前底盘几何参数：

```text
底盘碰撞体：0.46 m x 0.34 m x 0.12 m
轮半径：0.06 m
轮厚度：0.05 m
轮心位置：x = +/-0.16 m, y = +/-0.16 m
IMU 安装位：x = 0.10 m, z = 0.15 m
MID360 安装位：x = 0.12 m, z = 0.20 m
可选 RGB-D 相机安装位：x = 0.24 m, z = 0.20 m
```

## 与导航参数的关系

Nav2 当前 footprint 为：

```text
[[0.28, 0.22], [0.28, -0.22], [-0.28, -0.22], [-0.28, 0.22]]
```

这个 footprint 对应 0.56 m x 0.44 m 的导航占用轮廓。它比当前底盘碰撞体每侧大约多留 0.05 m，用于覆盖轮子、模型误差和局部避障安全余量。

涉及文件：

```text
src/slam_nav_bringup/config/nav2_params.yaml
src/slam_nav_bringup/config/nav2_params_3d.yaml
```

## 参考工程取舍

参考工程中的机器人模型已经包含底盘、四轮、IMU、MID360 和 Gazebo 控制插件，适合作为结构参考。但当前项目不直接照搬其车体尺寸和比赛相关结构，原因是：

- 当前主流程已经稳定，直接改变底盘几何可能导致已保存地图和定位初始姿态重新对齐。
- 当前模型已经去掉比赛云台和附加块，命名更适合作为通用移动底盘。
- 当前模型预留了 RGB-D 相机接口，便于后续接入视觉导航和机械臂感知。

因此当前策略是保留现有几何数值，只把尺寸和传感器安装位参数化，便于后续统一调参。

## 修改原则

- 修改底盘碰撞体后，需要同步检查 Nav2 footprint。
- 修改 MID360 或 IMU 安装位后，需要同步检查 TF、FAST-LIO 配置和点云投影效果。
- 修改 spawn 起点或仿真地图后，需要重新确认 `map -> odom -> base_footprint` 对齐关系。
- 已保存地图正在用于课程交付时，不建议临时改变场地几何或机器人几何。
