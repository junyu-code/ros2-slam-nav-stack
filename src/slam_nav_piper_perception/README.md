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
```

当前 `target_pose_estimator_node.py` 是占位实现：取深度图中心窗口的中值深度并转换到 `piper_base_link`。后续可以替换为 YOLO/分割网络 + 点云聚类 + 抓取位姿估计。
