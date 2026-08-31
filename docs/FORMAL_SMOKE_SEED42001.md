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

## 同步 bag 复测与控制器标定

随后在相同 seed 上录制了同步诊断 bag：

```text
/home/ruilinli/simenv_p1_smoke_seed42001.vpJjb2/diagnostic_nonholonomic_entry.bag
```

测试端真值仅写入 bag 用于离线评分，没有接入算法 ROS 图。复测得到：

- Unitree RL 控制器在 `linear.y=0.20/0.25 m/s` 时没有可测平移，`1.0 m/s`
  才有明显横移；原 DWA 的 `[-0.20, 0.20]` 横移域会预测机器人实际不能执行的轨迹。
- 已据此把正式 DWA 改为 `max_vel_y=min_vel_y=0`、`vy_samples=1`，并关闭恢复插件的
  strafe 候选。全系统仍由 control mux 做每轴限速、加速度限制和过零保护。
- 非完整约束复测仍在 `150.48 s` 触发 `entry_timeout`。机器人已推进，但在第 6 段开始
  出现长时间旋转与恢复；这证明横移模型是一个真实缺陷，但不是唯一根因。
- 去除初始 world/map 刚体偏置后，第 5/6/7/8 个导航结果处的 GICP 平面误差分别约为
  `1.01/2.46/2.56/3.03 m`，而 yaw 误差仍约为 `0.09/0.13/0.04/0.02 rad`。
  退化主要发生在平面平移，不能通过放宽 jump gate 解决。
- 离线把 GICP 增量约束为非完整运动只能把末端误差从约 `3.03 m` 降到约 `2.36 m`，
  仍不合格。用本 seed 真值拟合的命令积分可得到较小误差，但它会在机器人受阻时虚增
  里程，且属于对单一场景的过拟合，因此没有写入正式定位。

FAST-LIO2 也在独立诊断链路上做了升级条件对照。当前 SimEnv 点云是每帧瞬时采样，点的
`offset_time` 全为零；后端持续报告 `No Effective Points`。约 `116.9 s` 的对照中，测试端
真值路径约 `19.85 m`，FAST-LIO2 输出路径约 `21.8 km`。它没有达到误差改善 20% 的门槛，
所以本轮不接入、不设为正式默认。

仓库已有的 Hector 受限修正也做了启动候选检查。正式参数同时启用
`multifloor_enabled=true` 与 `use_hector_correction=true` 时，定位 adapter 按设计拒绝启动：
当前 Hector 长期地图不支持独立楼层地图存取。该组合没有退化为单层运行，因此门禁行为
正确；在完成 Hector 分层地图实现前，它不能用于绕过本次 GICP 平移退化。

## 下一轮整改和放行顺序

1. 保留现有同步 bag 作为回归基线；新增 bag 继续由测试端单独录制 truth，且不得把 truth
   接入算法 ROS 图。
2. 用同一同步 bag 评估带退化检测的扫描匹配运动先验；先验只能用于候选生成和退化门控，
   不能在受阻时替代几何观测累加位移。至少补充直走、转弯、受阻和恢复四类 bag。
3. 继续执行 Unitree 窄门、近目标和原地旋转矩阵；目前只完成起步与横移响应标定，不能把
   DWA 候选参数标记为正式配置。
4. 单层 seed 42001 必须完成“入场→探索→返航→静止→FINISHED”，再进入 2/3 层和
   `0→1→2→0` 的真实电梯测试。
5. 最后运行 12-seed 与重复稳定性矩阵，并归档覆盖率、耗时、召回、虚警、三维误差、
   CPU/内存和失败原因。

如果同一 rosbag 上 GICP 的返航 XY 误差仍超过 0.5 m，再按项目升级条件对照 FAST-LIO2；
在误差改善至少 20%、实时率和任务成功率不下降、GPL-2.0 分发审查完成前，不切换正式默认。
