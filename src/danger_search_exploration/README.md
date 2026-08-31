# danger_search_exploration

单楼层 S0/P0 探索规划模块，并提供默认关闭的参数化电梯联调状态机。当前目标是跑通合法选点、路径校验、导航执行、有限恢复、稳定完成判定和任务停止链路；电梯状态机用于 S3/P3 接口和运动联调，不代表已经完成自主电梯语义发现。

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
- 对普通失败位置执行 15 s 短期空间冷却；成功目标至少冷却 45 s 且等待两个显著地图版本后才重新评估，避免地图噪声使同一点立即解禁。只有收到 `/navigation/recovery_event` 的失败事件或最终 `CONTROL_FAILED` 时，才把卡死点周围 `0.70 m` 内低净空通道和失败终点写入长期黑名单；暂停和短期冷却不清除长期黑名单，只有区域连续两个显著地图版本恢复安全净空才失效。
- 电梯联调启用后，显式触发“厅前目标 → 呼梯到当前层 → 开门 → navigation 限时门槛穿越进舱 → 关门 → 换层 → localization 楼层确认 → 开目标层门 → 等待地图稳定 → 限时门槛穿越出舱”的有界状态机。门梯和穿越服务在后台线程调用，不阻塞地图、位姿和导航状态回调。
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
| `/set_door_state` | `building_generator_interfaces/SetDoorState`，仅电梯联调启用时 |
| `/call_elevator` | `building_generator_interfaces/CallElevator`，仅电梯联调启用时 |
| `/navigation/traverse_portal` | `danger_search_common/TraversePortal`，进出轿厢的有界导航模式 |
| `/navigation/cancel_portal` | `std_srvs/Trigger` |
| `/localization/set_current_floor` | `danger_search_common/SetCurrentFloor`，仅在电梯服务确认目标层后调用 |

提供：

| 默认名称 | 类型 |
|---|---|
| `/danger_search/start_exploration` | `std_srvs/Trigger` |
| `/danger_search/stop_exploration` | `std_srvs/Trigger` |
| `/danger_search/start_elevator` | `std_srvs/Trigger`，使用参数目标楼层和入口位姿启动电梯任务 |
| `/danger_search/cancel_elevator` | `std_srvs/Trigger`，撤销导航目标并终止当前电梯任务 |

## 电梯联调

电梯功能默认关闭。完整系统启动时显式传入：

```bash
roslaunch danger_search_bringup competition.launch \
  elevator_enabled:=true \
  elevator_target_floor:=1 \
  simenv_root:=/home/langan/simenvnew
```

入口位姿使用 `[x, y, yaw]`，其中 yaw 从电梯厅指向轿厢。可以在启动前写入 YAML，也可以在运行中设置私有参数：

```bash
rosparam set /exploration/elevator/portal_pose '[-4.25, -0.80, 0.0]'
rosservice call /danger_search/start_elevator '{}'
```

若 `portal_pose` 为空，显式触发时使用最近一次成功前沿的 `(x, y, yaw)`。这只适合已人工确认该前沿为电梯门槛的隔离联调；正式自主流程仍需由允许传感器识别门槛和轿厢入口，不能把任意重复前沿当成电梯，也不能读取布局、电梯配置或世界真值。

当 `elevator/enabled=true` 且 `elevator/autonomous_when_floor_complete=true` 时，当前层满足既有“连续无前沿、地图稳定、无活动导航目标”条件后，不再立即发布全局完成，而是把当前层加入 `completed_floors`，从显式配置的 `elevator/served_floors` 中确定性选择距离最近的未完成楼层并启动同一电梯状态机。默认楼层列表为 `[0, 1]`；三层场景必须显式配置 `[0, 1, 2]`，不得从生成场景或真值文件推断。自主换层要求显式 `portal_pose`，不会退化使用任意最近成功前沿。仅当所有 `served_floors` 均收敛后才发布 `/exploration/complete=true`；剩余楼层换层失败达到上限时保持 `complete=false` 并报告 `autonomous_floor_transition_unavailable`。

状态会写入 `/exploration/status` 的 `elevator_enabled/elevator_active/elevator_state/elevator_reason/current_floor`，并附带 `autonomous_floor_change_enabled/served_floors/visited_floors/completed_floors/next_autonomous_floor/autonomous_transition_failures`。机器人已位于入口 `portal_near_distance` 范围内时会跳过不必要的厅前后退目标，直接呼梯开门。任何导航失败、门梯服务拒绝、操作超时、换层后地图未稳定或 exploration stop 都会有限退出并取消活动目标。换层成功后会清空当前实现中的楼层局部目标冷却和陷阱黑名单，避免不同楼层的同坐标记录互相污染；需要重访已完成楼层时仍应升级为分楼层运行时缓存。

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
