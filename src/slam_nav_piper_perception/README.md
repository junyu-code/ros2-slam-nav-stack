# slam_nav_piper_perception

Piper 独立腕部 RGB-D 感知包。它只消费 `/piper/arm_camera/*`，不复用导航相机 `/nav_camera/*`，也不把输出接入默认 Nav2 costmap。

## 话题

输入：

```text
/piper/arm_camera/color/image_raw
/piper/arm_camera/color/camera_info
/piper/arm_camera/depth/image_raw
/piper/arm_camera/depth/camera_info
```

输出：

```text
/piper/perception/detections_2d
/piper/perception/detections_3d
/piper/perception/target_pose
/piper/perception/debug_image
```

当前 `target_pose_estimator_node.py` 是可替换的假检测实现：取深度图中心窗口的中值深度，发布一个
`center_depth_target` 的 `vision_msgs/Detection2DArray`、`vision_msgs/Detection3DArray`，并把目标中心转换到
`piper_base_link` 后发布 `/piper/perception/target_pose`，同时在最近一帧彩色图上画检测框并发布
`/piper/perception/debug_image`。这样下游抓取候选和 action 冒烟能使用真实消息形态；
后续接入 YOLO/分割网络、点云聚类和抓取位姿估计时，保持这些话题接口不变即可。
