# slam_nav_piper_manipulation

Piper 抓取/放置任务层。它暴露项目侧 action，不直接暴露 MoveIt2 或厂家 SDK。

## Action

```text
/piper/task/pick_object
/piper/task/place_object
```

当前实现是安全的占位执行：会读取 `/piper/perception/target_pose`，发布 `/piper/grasp_candidates`，并向 `/piper/control/owner_request` 申请 `moveit` owner。默认 `publish_base_stop=false`，不会主动改写 `/cmd_vel`，避免影响 task1 主流程。

实机移动操作时，应由任务编排层显式暂停导航或打开 `publish_base_stop`。

一键任务层烟测：

```bash
./run.sh piper-task-smoke
```

该脚本会启动 Piper fake 感知链路，等待目标位姿与抓取候选，再向 pick/place action 各发送一次 goal。它只验证任务接口和状态机，不接真实 MoveIt2 执行后端或厂家 SDK。当前已验证 fake pick/place 均返回成功。

## 学习层边界

配置里已预留 `use_ranked_grasp_candidates` 和 `/piper/learning/grasp_candidates_ranked`，但默认关闭。后续只有在离线评估和仿真冒烟通过后，任务层才应显式消费学习排序结果。
