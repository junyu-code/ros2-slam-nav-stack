# slam_nav_piper_interfaces

Piper 移动操作扩展的项目侧接口包。它只定义上层任务接口，不绑定 MoveIt2、厂家 SDK 或具体 RGB-D 相机驱动。

## 接口

```text
/piper/task/pick_object   slam_nav_piper_interfaces/action/PickObject
/piper/task/place_object  slam_nav_piper_interfaces/action/PlaceObject
/piper/grasp_candidates   slam_nav_piper_interfaces/msg/GraspCandidateArray
```

约定：所有抓取候选和任务目标最终应转换到 `piper_base_link` 或 `map` 下的明确坐标系，不能复用导航相机 `/nav_camera/*`。
