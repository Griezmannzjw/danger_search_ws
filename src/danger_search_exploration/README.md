# danger_search_exploration

单楼层 S0/P0 探索规划模块。当前目标是跑通合法选点、路径校验、导航执行、失败处理和任务停止链路；这不是最终比赛探索算法。

## S0 当前算法

S0 按 `SimEnv/AGENTS.md` 采用 **最近可达简单前沿（nearest reachable frontier）**：

1. 在当前二维占据地图中，把与未知栅格四邻接的已知自由栅格识别为前沿。
2. 使用 8 邻域连通性聚类前沿，过滤长度小于 `min_frontier_length` 的噪声前沿。
3. 每个前沿簇选择最接近簇质心的自由栅格作为候选，确保目标仍位于已知自由空间。
4. 按机器人到候选的欧氏距离由近到远排序，并调用 `/move_base/make_plan` 逐个验证。
5. 将首个返回非空路径的候选作为 `MoveBaseGoal` 发送给 navigation。

该算法是确定性的 S0 基线，用于完成至少一次地图驱动移动并跑通任务与评分链路。它不是最终比赛算法：S0 不要求信息增益、地图版本、自动收敛或完整恢复；S1 再加入可靠前沿收敛、目标冷却/黑名单、定位修正重验证和单楼层自动结束。当前代码中的有限失败冷却仅作为防止同一目标立即死循环的保护，不代表 S1 已完成。

## 职责与边界

- 从 `/map` 提取、聚类并过滤简单前沿候选。
- 使用 `/localization/pose`、`/mapping/status` 和 `/navigation/health` 判断输入是否就绪。
- 调用 `/move_base/make_plan`，仅发送返回非空路径的候选。
- 通过 `/move_base` Action 发送、监控、超时取消导航目标。
- 对失败位置执行短期冷却，连续失败达到上限后退避，不无限重试同一候选。
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
3. 候选来自有效前沿簇，必须在地图范围内且栅格值为 `0`，并通过非空 `make_plan` 校验。
4. 成功后继续选择新目标；失败、取消和超时不会无限重试同一位置。
5. stop 取消全部目标，旧 Action 回调不能重新激活已停止的会话，重复 stop 返回可预测成功。

后续 S1 才实现可靠前沿聚类、目标持久化、自动收敛和更完整恢复；S2 以后再实现房间可见性、多楼层与门梯能力。
