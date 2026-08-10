# danger_search_mission

单楼层 P0 任务总控。该包把定位、感知、探索、导航和控制模块连接成：

```text
IDLE -> ENTERING -> EXPLORING -> RETURNING -> FINISHED
                           \----> ERROR
```

## 实际职责

- 启动前等待位姿、建图、导航、相机/检测器和 exploration 服务就绪；
- 在任务开始时记录本次 `map` 中的起点位姿；
- 启动 exploration 并接收 `/exploration/complete`；
- 对红球检测做置信度过滤、空间融合和至少三帧确认；
- 在门外保存官方出生点，自动前进穿过入口后才启动 exploration；
- 探索完成后停止 exploration，独占发送返航 `/move_base` 目标；
- `600s` 仅是评分满分线，默认任务总超时为 `0`，不会据此中断探索；
- 返航成功后把坐标转换为以本次起点为原点的任务坐标；
- 原子写入 `detected_danger.json`，最后发布 `FINISHED`；
- 返航失败、返航超时或结果写入失败时发布 `ERROR`，并尽量保留已确认结果。

## 接口

订阅：

| 话题 | 类型 | 用途 |
|---|---|---|
| `/localization/pose` | `geometry_msgs/PoseWithCovarianceStamped` | 记录起点和检查位姿新鲜度 |
| `/mapping/status` | `danger_search_common/MappingStatus` | 建图就绪门和当前楼层 |
| `/navigation/health` | `danger_search_common/NavigationHealth` | 导航就绪与活动目标状态 |
| `/danger_detector/status` | `danger_search_common/DetectionStatus` | 相机、同步输入和 TF 就绪门 |
| `/danger_detector/detections` | `danger_search_common/DangerSourceArray` | 逐帧红球观测 |
| `/exploration/status` | `std_msgs/String` | 剩余前沿和覆盖诊断 |
| `/exploration/complete` | `std_msgs/Bool` | 自动结束探索并开始返航 |
| `/entrance/ready` | `std_msgs/Bool` | 主入口已经实际打开的就绪信号 |

发布：

| 话题 | 类型 | 用途 |
|---|---|---|
| `/mission/status` | `danger_search_common/MissionStatus` | 状态、耗时、楼层、前沿和结束原因 |
| `/mission/active` | `std_msgs/Bool` | 任务是否处于活动阶段 |

服务：

| 服务 | 类型 | 行为 |
|---|---|---|
| `/danger_search/start` | `std_srvs/Trigger` | 等待预检通过并开始探索 |
| `/danger_search/finish` | `std_srvs/Trigger` | 调试时提前停止探索并开始返航，不跳过返航 |
| `/danger_search/return_home` | `std_srvs/Trigger` | 调试时立即进入返航 |

mission 在 `ENTERING` 和 `RETURNING` 阶段作为 `/move_base` Action 客户端；只有入口
目标成功后才启动 exploration，返航前则先调用其 stop 服务，避免目标竞争。

## 结果路径

统一 launch 默认支持以下零配置布局：

```text
某个父目录/
├── SimEnv/
└── danger_search_ws/
```

此时自动输出到同级 `SimEnv/results/detected_danger.json`，没有写死用户名，也不依赖
节点当前工作目录。如果赛事部署布局不同，只需覆盖一次：

```bash
roslaunch danger_search_bringup competition.launch \
  simenv_root:=/absolute/path/to/SimEnv
```

也可以用 `result_file:=/absolute/path/to/SimEnv/results/detected_danger.json` 覆盖完整文件。

## 多帧确认

- `confidence < min_confidence` 的观测不进入候选；
- 相同 `detection_id` 只处理一次；
- 同楼层三维距离小于 `dedup_distance` 的观测合并为同一轨迹；
- 位置使用全部合并观测的运行均值；
- 轨迹观测数达到 `min_detections` 后才写入最终结果。

P0 默认值为 `0.8 m`、`3` 帧和置信度 `0.60`，需根据多随机场景结果继续标定。

## 启动

正式联调由 bringup 启动。本包独立调试也支持同样的路径与自动开始参数：

```bash
source /opt/ros/noetic/setup.bash
source ~/myProject/danger_search_ws/devel/setup.bash
roslaunch danger_search_mission mission.launch autostart:=false
```

独立启动仍要求外部已有 exploration 服务、`/move_base` Action 和四类健康输入。
