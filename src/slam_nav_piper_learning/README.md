# slam_nav_piper_learning

Piper 后续强化学习/学习策略的预留包。当前它不参与 `piper_sim.launch.py`、`piper_real.launch.py`、Nav2 或 task1 主流程。

## 当前边界

推荐先把学习模块放在抓取候选排序层：

```text
/piper/grasp_candidates
  -> learning ranker
  -> /piper/learning/grasp_candidates_ranked
```

任务层后续可以在明确验证后再选择是否消费 ranked 输出。不要让强化学习策略直接控制 Piper 关节，也不要绕过 MoveIt2/SDK 安全边界。

## 单独启动

默认不启动任何学习节点：

```bash
ros2 launch slam_nav_piper_learning piper_learning.launch.py
```

仅做启发式排序冒烟：

```bash
ros2 launch slam_nav_piper_learning piper_learning.launch.py enable_learning:=true policy_backend:=heuristic
```

`policy_backend:=rl` 和 `policy_backend:=onnx` 只是未来接口占位；模型权重、训练数据、日志和 checkpoint 不放进主仓库。
