# danger_search_mission

比赛任务总控和结果文件唯一写入方：

```text
IDLE -> ENTERING -> EXPLORING -> RETURNING -> FINISHED
                           \----------------> ERROR
```

## 任务语义

- 正式启动需等待 preflight、位姿、稳定地图、导航、检测器、入口门和 exploration 服务。
- 任务开始先清空跟踪器，并原子覆盖为本轮空的 `mission_status=RUNNING` 结果。
- 保存 `home_floor=0` 和完整起点 `x/y/z/yaw`，分段进入建筑后启动探索。
- 只融合与当前 `floor_id/map_epoch` 一致的确认红球，按楼层和三维距离去重。
- `/exploration/complete`、FinishMission、ReturnHome 或任务超时都先停止探索，再进入相同
  的返航流程。
- 当前楼层不是 0 时先调用 `/danger_search/transit_floor` 回 0 层并退出轿厢，然后发送
  二维 home goal。
- 返航只有在位置误差 `<=0.5 m`、yaw `<=20°`、导航不活动且
  `/danger_search/cmd_vel_sent` 连续 2 秒静止时成功。
- 成功后才冻结 `exploration_time`、原子写结果并进入 `FINISHED`；Action、返航验证或写盘
  失败均进入 `ERROR`，迟到回调由 return epoch 丢弃。

## ROS 接口

订阅：`/localization/pose`、`/mapping/status`、`/navigation/health`、
`/danger_detector/status`、`/danger_detector/detections`、`/exploration/status`、
`/exploration/complete`、`/entrance/ready`、`/danger_search/preflight_ready` 和
`/danger_search/cmd_vel_sent`。

调用：`/danger_search/start_exploration`、`/danger_search/stop_exploration`、
`/move_base` 和 `/danger_search/transit_floor`。

提供：

| 服务 | 行为 |
|---|---|
| `/danger_search/start` | 开始本轮任务；重复活动请求不会重置状态 |
| `/danger_search/finish` | 提前结束覆盖，但仍执行完整返航 |
| `/danger_search/return_home` | 立即停止探索并执行相同返航 |

发布 `/mission/status` 和 `/mission/active`。`/exploration/complete` 不是 mission 终态。

## 结果和坐标

必要 evaluator 字段之外增加 `coordinate_frame`、`mission_status`、运行 profile 和定位后端：

```json
{
  "exploration_time": 98.76,
  "coordinate_frame": "world",
  "mission_status": "FINISHED",
  "run_profile": "formal",
  "localization_backend": "gicp",
  "official_eligible": true,
  "detected_danger_sources": []
}
```

内部统一保存任务起点相对三维坐标。`result_coordinate_frame=auto` 时，只读取公开
`team_scene_info.json`：若声明 world，则执行 `Rz(yaw0)*p_relative+t_robot_start`；否则
输出 `start_relative`。非零起点、非零 yaw 和 z 变换均有回归测试。0 个危险源必须输出
空数组，不能残留上轮结果。

默认结果路径由 bringup 解析到同级 `SimEnv/results/detected_danger.json`，也可通过绝对
`result_file` 覆盖。

正式调用 evaluator 前先运行 fail-closed profile 门禁：

```bash
rosrun danger_search_mission validate_result.py --official \
  /home/ruilinli/SimEnv/results/detected_danger.json
```

只有 `mission_status=FINISHED`、`run_profile=formal`、`localization_backend=gicp` 且
`official_eligible=true` 时退出码为 0。字段缺失、任务失败、元数据自相矛盾或
`simulation_truth` 均以退出码 2 拒绝，验收脚本必须在退出码非 0 时停止评分。

## 正式与隔离运行

正式 profile 要求 `competition_mode=true`、`multifloor_enabled=true`、GICP 后端、公开
scene contract 和 preflight ready。真值 profile 固定要求
`competition_mode=false`、`multifloor_enabled=true` 和 `gazebo_truth` 后端；其结果文件名必须
为 `detected_danger.simulation_truth.json`，并始终写出 `official_eligible=false`。两个 profile
均需 preflight ready；请通过 bringup 的对应 wrapper 启动，而不是混用参数。

```bash
roslaunch danger_search_mission mission.launch autostart:=false
```

入口分段和返航参数见 `config/default.yaml`。默认 `mission_timeout_s=0`，探索期间不会
因为累计耗时自动触发返航；探索自然完成或收到 FinishMission/ReturnHome 请求后才进入
RETURNING 闭环。600 秒仍是比赛硬门槛/评分口径，由测试端统计；只有实际回到起点并
满足位置、朝向和连续静止门槛才写 FINISHED。
