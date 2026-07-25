# danger_search_mission

任务总控模块。

## 职责

1. **任务状态机管理**：IDLE → EXPLORING → RETURNING → FINISHED
2. **危险源融合去重**：维护全局危险源列表，多次检测确认后入库
3. **任务结束判断**：超时 / 覆盖率达标 / 手动结束
4. **返航触发**：通知探索模块返航
5. **结果输出**：按比赛格式写入 `results/detected_danger.json`

## 节点

### mission_manager.py
任务管理器主节点。

#### 订阅话题
| 话题 | 类型 | 说明 |
|------|------|------|
| `/danger_search/detections` | `danger_search_common/DangerSourceArray` | 危险源检测结果 |

#### 发布话题
| 话题 | 类型 | 说明 |
|------|------|------|
| `/mission/state` | `danger_search_common/MissionState` | 任务状态（latch） |
| `/mission/active` | `std_msgs/Bool` | 任务是否激活（latch） |

#### 服务
| 服务 | 类型 | 说明 |
|------|------|------|
| `/danger_search/start` | `std_srvs/Trigger` | 开始任务 |
| `/danger_search/finish` | `std_srvs/Trigger` | 结束任务并输出结果 |
| `/danger_search/return_home` | `std_srvs/Trigger` | 触发返航 |

## 融合去重策略

1. 每次检测到来时，与已有危险源做距离匹配
2. 距离 < `dedup_distance_m` 视为同一个危险源
3. 同一源检测次数 ≥ `min_detections` 才确认为正式结果
4. 位置取多次检测的平均值，提高精度
5. 候选阶段的源不计入最终结果

## 配置参数

见 `config/default.yaml`，主要包括：
- 去重距离阈值
- 最少确认检测次数
- 任务超时时长
- 结果输出路径
- 返航触发条件

## 输出格式

```json
{
  "exploration_time": 98.76,
  "detected_danger_sources": [
    {"position": [2.34, -1.56, 0.25]}
  ]
}
```

完全匹配比赛要求的 `detected_danger.json` 格式。
