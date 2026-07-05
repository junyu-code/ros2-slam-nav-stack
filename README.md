# Ubuntu 22.04 SLAM Navigation System

这是一个面向 Ubuntu 22.04 + ROS2 Humble + Gazebo Classic 的通用 SLAM 与自主导航工作区。工程目标是完成仿真建图、目标点导航和静态避障验证，并保留后续扩展动态避障或实物演示的空间。

## 工作区结构

```text
slam_nav_ws/
  src/
    slam_nav_simulation/       # Gazebo 场地、机器人模型、仿真启动
    slam_nav_bringup/          # 建图、导航参数和启动入口
    FAST_LIO/                  # 3D LiDAR-Inertial odometry and mapping
    pointcloud_to_laserscan/   # 3D 点云转 2D LaserScan
    ros2_livox_simulation/     # Livox Mid-360 仿真插件
    imu_complementary_filter/  # IMU 姿态滤波工具
```

## 构建

```bash
cd ~/slam_nav_ws
./build.sh
source install/setup.bash
```

## 建图流程

终端 1 启动仿真：

```bash
cd ~/slam_nav_ws
./start_simulation.sh
```

终端 2 启动 FAST_LIO、点云转 scan、slam_toolbox：

```bash
cd ~/slam_nav_ws
./start_mapping.sh
```

终端 3 键盘控制机器人探索场地：

```bash
cd ~/slam_nav_ws
./teleop.sh
```

保存地图：

```bash
cd ~/slam_nav_ws
./save_map.sh nav_test_map
```

## 导航流程

保持仿真和建图链路运行，再启动 Nav2：

```bash
cd ~/slam_nav_ws
./start_navigation.sh
```

在 RViz 中使用 `2D Goal Pose` 指定目标点，观察全局路径、局部路径、代价地图和机器人实际运动。

## 常用检查

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic hz /Odometry
ros2 topic hz /cloud_registered
ros2 topic hz /scan
ros2 topic hz /map
ros2 topic info /cmd_vel
```

清理残留进程：

```bash
cd ~/slam_nav_ws
./clean.sh
```
## 动态障碍物

默认启动的是原来的静态场地，不包含动态障碍物：

```bash
./start_simulation.sh
```

启动带动态障碍物的场地：

```bash
./start_simulation_dynamic.sh
```

也可以直接传 launch 参数：

```bash
./start_simulation.sh world:=dynamic
```

动态场地内包含两个圆柱动态障碍物：

- `moving_obstacle`：红色，沿场地中部 y 方向往返移动，速度约 `0.35 m/s`。
- `fast_moving_obstacle`：蓝色，在场地中部通道出口附近沿 x 方向往返移动，速度约 `0.85 m/s`。

它们都由 `slam_nav_simulation` 内的 Gazebo 插件驱动，带有 collision，可以被仿真雷达和 Nav2 costmap 感知。

确认动态障碍物是否运动：

```bash
gz model -m moving_obstacle -p
gz model -m fast_moving_obstacle -p
```

间隔几秒重复执行，如果坐标变化，说明动态障碍物插件正在工作。
