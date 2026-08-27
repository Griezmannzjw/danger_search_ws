# danger_search_common

公共消息、服务定义与工具函数包，所有其他功能包均依赖此包。

## 功能

- 自定义 ROS 消息（msg）定义
- 自定义 ROS 服务（srv）定义
- 通用工具函数（后续补充）

## 消息定义

### MappingStatus.msg
- `transitioning`：显式换层尚未取得稳定目标层地图
- `map_epoch`：活动地图身份，切层或地图重置时单调递增
- `floor_z_m`：当前楼层相对任务起点的配置标高

### TransitFloor.action
- Goal：`target_floor`, `exit_to_hall`
- Result：`success`, `reached_floor`, `map_epoch`, `failure_code`, `message`
- Feedback：`phase`, `current_floor`, `progress`, `map_epoch`

### DangerSource.msg
单个危险源检测结果
- `position`：带时间戳的 map 三维位置
- `floor_id`、`map_epoch`：采集时活动楼层地图身份
- `localization_correction_version`：采集时定位修正版本
- `confidence`、`confirmed`：检测置信度和跨帧确认状态

### DangerSourceArray.msg
一批次危险源检测结果数组
- `header` (std_msgs/Header)
- `dangers` (DangerSource[]): 危险源列表

### MissionState.msg
任务状态消息
- `state` (string): IDLE / ENTERING / EXPLORING / RETURNING / FINISHED / ERROR
- `detail` (string): 状态详情
- `progress` (float32): 任务进度 0~1

## 服务定义

### SwitchFloor.srv
- Request：`transition_id`, `target_floor`
- Response：`success`, `map_epoch`, `message`
- 相同 `transition_id` 与目标楼层的重试必须返回同一 epoch，不重复切图

### StartMission.srv
开始探索任务
- Request: 空
- Response: success, message

### FinishMission.srv
结束任务并输出结果
- Request: 空
- Response: success, message, detected_count, exploration_time

### ReturnHome.srv
触发返航
- Request: 空
- Response: success, message

## 依赖

- std_msgs
- geometry_msgs
- sensor_msgs
- nav_msgs
- message_generation / message_runtime
- actionlib_msgs
