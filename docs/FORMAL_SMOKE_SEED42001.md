# 正式模式 smoke：seed 42001

日期：2026-08-27  
结论：`FAIL / entry_timeout`。本记录是问题证据，不是比赛验收通过记录。

## 场景与约束

- 使用真实 SimEnv/Gazebo、building control、Unitree 控制器以及官方强类型门/电梯服务。
- 固定 seed `42001`，1 层，0 个危险源，0 个干扰物。
- 公开起点为 world `(0.0, -3.2, 0.6)`、yaw `1.5708`。
- `ENABLE_REFEREE_ODOM=0`、`ENABLE_GROUND_TRUTH=0`、
  `POINTCLOUD_USE_GROUND_TRUTH_ODOM=0`。
- 算法只读取 `team_scene_info.json`；测试端生成和保存的真值不提供给算法。

临时场景、结果和真值保存在：

```text
/home/ruilinli/simenv_p1_smoke_seed42001.vpJjb2
```

ROS 日志保存在：

```text
/home/ruilinli/.ros/log/9ba7071c-a1dc-11f1-b67d-29a411bbf022
```

## 已证明可工作的部分

第四次启动的 preflight 输出 `competition runtime contract READY`。同一次运行中确认：

- `/scan` 的平台原始类型 `sensor_msgs/PointCloud` 被正确接受；
- 强类型 `/call_elevator`、`/set_door_state` 可解析；
- localization、mapping、navigation、RGB-D 和 mission 达到启动就绪条件；
- `/cmd_vel` 的唯一算法发布者为 `/control`；
- mission 能原子覆盖旧结果并在失败时写出明确 `ERROR`，没有伪报完成。

实测前修复的 preflight 问题及回归测试包括：

1. 内建禁止话题和公开 scene contract 的附加禁止话题取并集；
2. 兼容 `rosgraph.Master.getSystemState()` 的原始与已解包返回形状；
3. 区分平台节点和算法节点，平台自身订阅 truth 话题不会被误判为算法违规。

## 失败证据

结果文件内容为：

```json
{
  "exploration_time": 150.48,
  "coordinate_frame": "world",
  "mission_status": "ERROR",
  "detected_danger_sources": []
}
```

mission 日志记录 `reason=entry_timeout`。运行期间多次出现：

```text
GICP rejected (TRANSLATION_LIMIT): translation=0.136/0.080
GICP rejected (TRANSLATION_LIMIT): translation=0.133/0.081
GICP rejected (TRANSLATION_LIMIT): translation=0.123/0.080
```

测试端在运行末尾独立抽查到机器人 world 平面位置约为 `(10.69, -0.32)`，而算法定位约为
`(2.36, -0.23)`。该约 8.3 m 差异只用于测试诊断，算法未订阅或读取此真值。由于本轮未录制
同步 rosbag，这一抽查不能替代可复现的误差曲线，但足以否决当前 GICP/DWA 组合的正式
配置资格。不能通过放宽 translation gate 来掩盖该差异。

同轮还观察到 Mission 入场阶段的恢复事件被 exploration 记录为 trap。现已按
`active_goal_id`、事件时间和 exploration/transit 所有权隔离，并补充 Mission goal 与迟到
old-goal 回归测试。

## 下一轮整改和放行顺序

1. 用同一 seed 录制 `/scan`、IMU、GICP raw/validated pose、mapping pose、
   `/danger_search/cmd_vel_sent`、局部/全局规划和 navigation recovery；测试端另录 truth，
   两者不得接入算法图。
2. 对齐每帧点云时间、实际控制速度与 GICP 接受周期，定位 translation gate 拒绝的是
   单帧正常位移、点云畸变还是错误配准；修复原因后再用离线 bag 回放比较 XY/yaw 漂移。
3. 单独执行 Unitree 起步、窄门、近目标和原地旋转矩阵，检查 footprint/costmap 与 DWA
   加速度联合合同；四项全部通过后才把候选 DWA 参数标记为正式。
4. 单层 seed 42001 必须完成“入场→探索→返航→静止→FINISHED”，再进入 2/3 层和
   `0→1→2→0` 的真实电梯测试。
5. 最后运行 12-seed 与重复稳定性矩阵，并归档覆盖率、耗时、召回、虚警、三维误差、
   CPU/内存和失败原因。

如果同一 rosbag 上 GICP 的返航 XY 误差仍超过 0.5 m，再按项目升级条件对照 FAST-LIO2；
在误差改善至少 20%、实时率和任务成功率不下降、GPL-2.0 分发审查完成前，不切换正式默认。
