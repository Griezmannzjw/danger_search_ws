# Evaluator 兼容与验收口径

SimEnv evaluator 的必要输入保持不变：

```json
{
  "exploration_time": 98.76,
  "detected_danger_sources": [
    {"position": [2.34, -1.56, 0.25]}
  ]
}
```

本实现新增顶层 `coordinate_frame` 和 `mission_status`，不修改必要字段。当前 SimEnv
evaluator 以键读取上述必要字段，因此可忽略新增元数据。

## 坐标

- `coordinate_frame=world`：位置是 Gazebo world 坐标。只用公开
  `team_scene_info.json.robot_start` 将内部起点相对坐标变换到 world。
- `coordinate_frame=start_relative`：位置以任务起点为原点，x/y 随起始 yaw 旋转到机器人
  初始朝向，z 相对任务起点。
- `result_coordinate_frame=auto`：公开 scene contract 声明 world 时选 world，否则回退为
  start_relative。

当前 SimEnv 文档要求 world，比赛 PDF 描述可解释为起点相对坐标，因此最终赛前仍需书面
确认。代码不依赖确认结果，可通过 launch 参数固定任一口径。

## 两套离线评估

评估只能在算法进程退出后由独立测试端读取真值：

```bash
python3 /home/ruilinli/SimEnv/src/building_obstacles/scripts/evaluate_danger.py \
  --truth-file /home/ruilinli/SimEnv/results/danger_truth.json \
  --detected-file /home/ruilinli/SimEnv/results/detected_danger.json \
  --output-file /home/ruilinli/SimEnv/results/evaluation_1m.json

python3 /home/ruilinli/SimEnv/src/building_obstacles/scripts/evaluate_danger.py \
  --truth-file /home/ruilinli/SimEnv/results/danger_truth.json \
  --detected-file /home/ruilinli/SimEnv/results/detected_danger.json \
  --output-file /home/ruilinli/SimEnv/results/evaluation_ratio.json \
  --use-scene-ratio
```

算法模块、launch 参数和 preflight 均不得读取 truth 文件。正式 12-seed 验收还需独立记录
覆盖率、轨迹、CPU/内存、耗时、返航状态和失败原因；自动化单元/ROS 集成测试不能替代该
物理仿真矩阵。
