# danger_search_common

公共消息、服务定义与工具函数包，所有其他功能包均依赖此包。

## 功能

- 自定义 ROS 消息（msg）定义
- 自定义 ROS 服务（srv）定义
- 通用工具函数（后续补充）

## 消息定义

### DangerSource.msg
单个危险源检测结果
- `pose` (geometry_msgs/Pose): map 坐标系下位姿
- `confidence` (float32): 检测置信度 0~1
- `radius` (float32): 球体半径估计

### DangerSourceArray.msg
一批次危险源检测结果数组
- `header` (std_msgs/Header)
- `dangers` (DangerSource[]): 危险源列表

### MissionState.msg
任务状态消息
- `state` (string): IDLE / EXPLORING / RETURNING / FINISHED / ERROR
- `detail` (string): 状态详情
- `progress` (float32): 任务进度 0~1

## 服务定义

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
