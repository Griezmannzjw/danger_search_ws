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

必要 evaluator 字段之外增加 `coordinate_frame` 和 `mission_status`：

```json
{
  "exploration_time": 98.76,
  "coordinate_frame": "world",
  "mission_status": "FINISHED",
  "detected_danger_sources": []
}
```

内部统一保存任务起点相对三维坐标。`result_coordinate_frame=auto` 时，只读取公开
`team_scene_info.json`：若声明 world，则执行 `Rz(yaw0)*p_relative+t_robot_start`；否则
输出 `start_relative`。非零起点、非零 yaw 和 z 变换均有回归测试。0 个危险源必须输出
空数组，不能残留上轮结果。

默认结果路径由 bringup 解析到同级 `SimEnv/results/detected_danger.json`，也可通过绝对
`result_file` 覆盖。

## 正式与隔离运行

正式模式要求 `competition_mode=true`、`multifloor_enabled=true`、GICP 后端、公开 scene
contract 和 preflight ready。隔离 smoke 可显式关闭这些守卫，但不能作为正式验收结果。

```bash
roslaunch danger_search_mission mission.launch autostart:=false
```

入口分段和返航参数见 `config/default.yaml`。600 秒是比赛硬门槛/评分口径；默认
`mission_timeout_s=0` 不主动中断。若赛事要求强制超时返航，应在正式 launch 明确设置该
参数，超时后仍走 RETURNING，不直接写成功结果。
