# slam_nav_piper_calibration

Piper 腕部 RGB-D 相机的手眼标定配置包。当前只固定 eye-in-hand 标定边界和安全检查，不启动真实采样、不移动机械臂、不写 URDF，也不发布最终 TF。

## 默认边界

```text
camera: /piper/arm_camera/*
robot_base_frame: piper_base_link
tcp_frame: piper_tcp
camera_frame: piper_arm_camera_optical_frame
calibration namespace: /piper/calibration/*
output: datasets/piper_hand_eye/
```

`datasets/` 已在 `.gitignore` 中忽略，后续 rosbag、标定图片和求解结果不要放进 `src/`。

## 静态检查

```bash
./run.sh piper-hand-eye-check
ros2 run slam_nav_piper_calibration piper_hand_eye_config_check.py
```

该检查会确认：

- 标定相机只使用 `/piper/arm_camera/*`，不引用 `/nav_camera/*`。
- frame 约定保持 `piper_base_link -> piper_tcp -> piper_arm_camera_optical_frame`。
- 标定输出和服务隔离在 `/piper/calibration/*`。
- 默认不允许真实运动、不发布最终 TF、不自动写入机器人描述。
- 标定数据默认进入 `datasets/piper_hand_eye/`，避免撑大 Git 仓库。

## 后续接入

真实标定时建议顺序是：先验证急停和失能，再确认 MoveIt2 plan-only 可用，再手动采样多个 TCP 姿态下的标定板观测，最后人工验收重投影误差和 TF 方向。通过验收后，再把结果作为显式参数加载到描述或静态 TF 发布入口；不要让标定脚本自动修改 task1 主链路。
