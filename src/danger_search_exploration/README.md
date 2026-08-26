# danger_search_exploration

单楼层 S0/P0 探索规划模块。当前目标是跑通合法选点、路径校验、导航执行、有限恢复、稳定完成判定和任务停止链路；这不是最终比赛探索算法。

## 当前算法

节点保留简单前沿聚类，但导航目标使用前沿内侧观察位，而不是未知边界本身：

1. 在当前二维占据地图中，把与未知栅格四邻接、占据概率低于 `free_threshold` 的已知栅格识别为前沿。
2. 按 `connectivity_occupied_threshold` 提取静态障碍并以圆形 `connectivity_clearance_radius` 膨胀，从机器人附近自由格进行 4 邻域搜索，只保留当前连通区域内的前沿。
3. 使用 8 邻域连通性聚类可达前沿，过滤长度小于 `min_frontier_length` 的噪声前沿。
4. 在每个前沿簇的已知可达侧搜索距边界 `0.30-0.70 m`（默认约 `0.45 m`）的观察位，检查完整落点净空并令朝向指向未知区域。
5. 调用 `/move_base/make_plan` 获取真实路径，拒绝穿越长期黑名单的候选，并按路径长度、整条路径最小净空和前沿信息量联合评分。
6. 发送得分最优的观察位；允许选择稍远但更宽、更有信息量的前沿。

默认连通性占据阈值为 `65`，圆形净空为 `0.30 m`，与 navigation 的静态地图规划配置一致。候选数量上限在可达区域筛选后生效；全图前沿数量仍用于区分“确实没有前沿”和“存在但当前区域不可达”。

该算法是确定性的 P0 基线。按团队追加验收要求，本包在简单前沿上实现了保守自动完成：输入新鲜且健康、无活动导航目标、地图超过稳定窗口且连续多轮无可达前沿时，才发布一次完成事件。它仍不包含 WFD、信息增益和定位修正版本等完整 P1 能力。

## 职责与边界

- 从 `/map` 提取、聚类并过滤简单前沿候选。
- 使用 `/localization/pose`、`/mapping/status` 和 `/navigation/health` 判断输入是否就绪。
- 调用 `/move_base/make_plan`，仅发送返回非空路径的候选。
- 通过 `/move_base` Action 发送、监控、超时取消导航目标。
- 对普通失败位置执行 15 s 短期空间冷却。只有收到 `/navigation/recovery_event` 的失败事件或最终 `CONTROL_FAILED` 时，才把卡死点周围 `0.70 m` 内低净空通道和失败终点写入长期黑名单；暂停、成功和短期冷却都不清除。只有区域连续两个显著地图版本恢复安全净空才失效。
- 通过 Trigger 服务幂等地启停；停止时取消全部活动目标。

模块不发布 `/cmd_vel`，不实现路径跟踪，不读取真值，不汇总危险源结果，也不负责调用任务级 `/danger_search/finish`。S0 允许人工结束任务。

### 为什么不输出 `cmd_vel`

探索模块只负责决定“去哪里”，输出的是带 `map` 坐标和朝向的 `/move_base` 导航目标。路径规划、路径跟踪和速度生成依赖局部障碍、机器人运动约束及控制频率，属于 navigation；navigation 输出 `/danger_search/nav_cmd_vel`。control 随后执行超时停车、加速度限制和安全仲裁，并作为唯一发布者输出最终 `/cmd_vel`：

```text
exploration --MoveBaseGoal--> navigation
            navigation --/danger_search/nav_cmd_vel--> control
                         control --/cmd_vel--> Unitree A1
```

如果 exploration 同时发布 `/cmd_vel`，会绕过路径跟踪和安全仲裁，并与 control 争抢同一话题，导致速度来源不唯一、停止语义不可靠。因此 exploration 在 stop 或目标超时时取消 Action，由 navigation 停止旧目标速度，再由 control 保证最终零速度。

## 接口

订阅：

| 默认名称 | 类型 |
|---|---|
| `/map` | `nav_msgs/OccupancyGrid` |
| `/localization/pose` | `geometry_msgs/PoseWithCovarianceStamped` |
| `/mapping/status` | `danger_search_common/MappingStatus` |
| `/navigation/health` | `danger_search_common/NavigationHealth` |
| `/navigation/recovery_event` | `danger_search_common/RecoveryEvent` |

发布：

| 默认名称 | 类型 | 语义 |
|---|---|---|
| `/exploration/status` | `std_msgs/String`（JSON） | `state/reason/remaining_frontier_count/known_grid_ratio/map_revision/has_active_goal` |
| `/exploration/complete` | `std_msgs/Bool`（latched） | 每个会话开始发布 `false`，满足收敛条件后只发布一次 `true` |
| `/exploration/observation_goals` | `geometry_msgs/PoseArray` | 当前前沿内侧观察位（RViz 诊断） |
| `/exploration/trap_blacklist` | `nav_msgs/GridCells` | 跨目标、跨暂停保留的卡死区域（RViz 诊断） |

调用：

| 默认名称 | 类型 |
|---|---|
| `/move_base/make_plan` | `nav_msgs/GetPlan` |
| `/move_base` | `move_base_msgs/MoveBaseAction` |

提供：

| 默认名称 | 类型 |
|---|---|
| `/danger_search/start_exploration` | `std_srvs/Trigger` |
| `/danger_search/stop_exploration` | `std_srvs/Trigger` |

所有接口名称、frame、超时和选点参数均从节点私有参数读取，默认值见 `config/default.yaml`。

## 运行

```bash
cd /home/langan/danger_search_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
roslaunch danger_search_exploration exploration.launch
```

独立 launch 会把 YAML 加载到节点私有命名空间。团队统一启动仍由 `danger_search_bringup competition.launch` 完成。

启动服务只让节点进入探索并等待输入，不要求依赖当时已经就绪。只有地图、位姿、建图健康、导航健康、`make_plan` 和 Action server 全部满足 S0 契约后才会发送目标。

## S0 验收

1. start 前不发送目标，重复 start 返回可预测成功。
2. 地图或位姿无效、建图未稳定/丢失、导航未就绪时不发送目标。
3. 候选来自有效前沿簇，必须在地图范围内且栅格值位于 `[0, free_threshold)`，并通过非空 `make_plan` 校验。
4. 成功后继续选择新目标；失败、取消和超时不会无限重试同一位置。
5. stop 取消全部目标，旧 Action 回调不能重新激活已停止的会话，重复 stop 返回可预测成功。
6. 输入过期、地图未初始化、导航服务不可用和定位丢失分别进入明确的 WAITING/FAILED 原因，不计入无可达前沿轮次。
7. `known_grid_ratio` 仅在已观测栅格的最小包围盒内统计，不把固定地图消息的全部未知边界当成真实可通行总面积。

后续 S1 才实现可靠前沿聚类、目标持久化、自动收敛和更完整恢复；S2 以后再实现房间可见性、多楼层与门梯能力。

## 多楼层探索（multifloor_enabled）

启用 `multifloor_enabled=true` 时，模块在当前层探索收敛（连续多轮无可达前沿且地图稳定）后，
自动执行电梯换层，而不是直接结束任务：

1. **电梯自主发现**：从当前层二维地图中找出大的实心连通区域（电梯井/楼梯井），
   在其周界检测"门缝"（墙上 0.8~2.5 m 的自由缺口），作为电梯厅候选。
   跳过贴地图边界或包围盒过大的区域，避免把密封楼外等伪影当作井道。
2. **换层状态机**：
   - 导航到电梯厅门缝前 → `/call_elevator` 呼梯到当前层并开门 → 进入轿厢
   - `/call_elevator` 呼叫目标楼层（`current_floor+1`）→ 电梯移动并开门
   - 出门 → 等待 `/mapping/current_floor` 变化且建图稳定 → 继续该层探索
3. **结束条件**：所有可达楼层探索完（目标楼层被电梯拒绝 `not served`），
   或换层连续失败达到 `elevator_max_retries` 后，才发布探索完成事件，交 mission 返航。
4. 电梯/门服务类型运行时动态发现；电梯每次操作带超时，换层失败自动退避重试，
   不无限循环。

相关参数见 `config/default.yaml` 的"多楼层探索"段：
`current_floor_topic`、`elevator_service`、`door_service`、`elevator_id`、
`shaft_min_area_m2`、`shaft_max_area_m2`、`door_gap_min_width_m`、`door_gap_max_width_m`、
`elevator_hall_approach_m`、`elevator_car_target_m`、`elevator_service_timeout_s`、
`elevator_max_retries`、`floor_change_timeout_s`、`floor_map_stable_time_s`。

单元测试 `test/test_multifloor.py` 覆盖电梯井门缝发现（含封闭无门缝井道不误检）。
