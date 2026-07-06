# slam_nav_piper_manipulation

Piper 抓取/放置任务层。它暴露项目侧 action，不直接暴露 MoveIt2 或厂家 SDK。

## Action

```text
/piper/task/pick_object
/piper/task/place_object
```

当前实现是安全的占位执行：会读取 `/piper/perception/target_pose`，发布 `/piper/grasp_candidates`，并向 `/piper/control/owner_request` 申请 `moveit` owner。默认 `publish_base_stop=false`，不会主动改写 `/cmd_vel`，避免影响 task1 主流程。

实机移动操作时，应由任务编排层显式暂停导航或打开 `publish_base_stop`。
