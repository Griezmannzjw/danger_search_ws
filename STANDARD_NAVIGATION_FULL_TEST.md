# Seed 42 固定电梯坐标完整多楼层探索测试

本文档测试以下完整流程：

```text
Seed 42 标准出生点
→ 探索 0 层
→ 使用固定电梯坐标到达并验证电梯
→ 乘梯到 1 层并探索
→ 乘梯到 2 层并探索
→ 自动返回 0 层和任务出生点
→ mission FINISHED
```

本测试使用 `simulation_truth`。固定电梯模式只绕过“发现电梯位置”，不会绕过门外目标可达性检查、开—关—开激光验证、进出梯避障、呼梯、楼层地图切换或稳定等待。

## 0. 首次测试前编译

最新代码尚未编译时执行一次：

```bash
cd /home/ruilinli/danger_search_ws
source /opt/ros/noetic/setup.bash
source /home/ruilinli/SimEnv/devel/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release
```

## 1. 终端一：启动 Seed 42 仿真

先在旧 Gazebo 和 danger_search 终端中按 `Ctrl+C`。然后打开新终端执行：

```bash
cd /home/ruilinli/SimEnv
source /opt/ros/noetic/setup.bash

GUI=true \
SEED=42 \
ROBOT_X=0.0 \
ROBOT_Y=5.0 \
ROBOT_Z=0.6 \
ROBOT_YAW=1.5708 \
ENABLE_SENSORS=1 \
ENABLE_REFEREE_ODOM=0 \
ENABLE_GROUND_TRUTH=0 \
POINTCLOUD_USE_GROUND_TRUTH_ODOM=0 \
START_CONTROLLER=1 \
CONTROLLER_FOREGROUND=1 \
./auto.sh
```

控制器加载完成后，在该终端中：

1. 按一次数字 `2`。
2. 等待机器人站稳。
3. 按一次数字 `6`。

必须看到类似输出：

```text
Switched from passive to fixed stand
Switched from fixed stand to RL
[INFO] Entered RL /cmd_vel mode.
```

注意：

- 不要按 `4`，模式 4 是键盘速度模式。
- 终端一必须保持运行。
- 每次重启 SimEnv 后都必须重新按 `2`、`6`。
- 未进入模式 `6` 时，机器人即使收到 `/cmd_vel` 也不会实际行走。

## 2. 终端二：启动完整 danger_search 系统

确认终端一已进入模式 `6` 后，打开新终端：

```bash
cd /home/ruilinli/danger_search_ws
source /opt/ros/noetic/setup.bash
source /home/ruilinli/SimEnv/devel/setup.bash
source devel/setup.bash --extend

roslaunch danger_search_bringup simulation_truth.launch \
  autostart:=false \
  entry_enabled:=false \
  fixed_elevator_hall_enabled:=true \
  fixed_elevator_hall_x:=-2.40 \
  fixed_elevator_hall_y:=-1.65 \
  fixed_elevator_hall_into_yaw:=-1.5707963 \
  simenv_root:=/home/ruilinli/SimEnv
```

等待并确认出现：

```text
[preflight] simulation_truth runtime contract READY
[posture_safety_monitor] safety stop cleared
```

终端二保持运行。

### 固定电梯坐标说明

标准出生点为：

```text
world: x=0.0, y=5.0, yaw=1.5708
```

对应的固定电梯门中心和进入方向为：

```text
map: x=-2.40, y=-1.65, into_yaw=-1.5707963
```

启用固定模式后，机器人直接使用该入口候选，但每次换层仍执行：

```text
导航到门外
→ 检查目标是否可达
→ 开门扫描
→ 关门扫描
→ 重新开门
→ 验证电梯
→ 进入轿厢
→ 呼梯
→ 切换楼层地图
→ 退出轿厢
```

## 3. 终端三：启动前检查

打开新终端并加载环境：

```bash
cd /home/ruilinli/danger_search_ws
source /opt/ros/noetic/setup.bash
source /home/ruilinli/SimEnv/devel/setup.bash
source devel/setup.bash --extend
```

确认关键节点：

```bash
rosnode list | sort
```

至少应该存在：

```text
/competition_preflight
/control
/exploration
/localization_adapter
/mission
/move_base
/navigation_monitor
/posture_safety_monitor
```

检查 preflight：

```bash
rostopic echo -n 1 /danger_search/preflight_ready
```

预期：

```text
data: True
```

检查安全状态：

```bash
rostopic echo -n 1 /danger_search/safety_stop
```

预期：

```text
data: False
```

检查初始楼层：

```bash
rostopic echo -n 1 /mapping/current_floor
```

预期：

```text
data: 0
```

检查固定电梯参数：

```bash
rosparam get /exploration/fixed_elevator_hall_enabled
rosparam get /exploration/fixed_elevator_hall_x
rosparam get /exploration/fixed_elevator_hall_y
rosparam get /exploration/fixed_elevator_hall_into_yaw
```

预期：

```text
true
-2.4
-1.65
-1.5707963
```

检查仿真安全和进出梯超时：

```bash
rosparam get /posture_safety_monitor/posture_imu_timeout_s
rosparam get /exploration/elevator_crossing_timeout_s
```

预期：

```text
1.5
40.0
```

## 4. 启动完整自主任务

上述检查全部正确后，在终端三执行：

```bash
rosservice call /danger_search/start "{}"
```

预期返回：

```text
success: True
message: "Mission started"
```

此后不要发送手动导航目标，也不要运行单独的 `TransitFloorAction` 测试脚本。系统将自动执行：

```text
开始任务
→ 探索 0 层
→ 判断 0 层无剩余可达前沿
→ 标记 0 层完成
→ 自动前往已知电梯并到达 1 层
→ 探索 1 层
→ 自动到达 2 层
→ 探索 2 层
→ 全部楼层探索完成
→ 自动返回 0 层
→ 返回任务出生点
→ 连续静止验证
→ 写入结果文件
→ mission FINISHED
```

## 5. 监控完整流程

监控终端先加载环境：

```bash
cd /home/ruilinli/danger_search_ws
source /opt/ros/noetic/setup.bash
source /home/ruilinli/SimEnv/devel/setup.bash
source devel/setup.bash --extend
```

### 5.1 探索状态

```bash
rostopic echo /exploration/status
```

0 层探索完成时，终端二日志应出现：

```text
[exploration] floor 0 complete
```

随后系统应自动进入换层流程。

### 5.2 电梯换层阶段

在另一个已加载环境的终端执行：

```bash
rostopic echo /danger_search/transit_floor/feedback
```

每次换层应依次经过：

```text
TO_HALL
OPEN_CURRENT_START
OPEN_CURRENT_WAIT
CAPTURE_OPEN_SCAN
VALIDATE_CLOSE_START
VALIDATE_CLOSE_WAIT
CAPTURE_CLOSED_SCAN
REOPEN_CURRENT_START
REOPEN_CURRENT_WAIT
ENTER
CLOSE_CURRENT_START
CLOSE_CURRENT_WAIT
CALL_TARGET_START
CALL_TARGET_WAIT
SWITCH_FLOOR_START
EXIT
WAIT_STABLE
DONE
```

### 5.3 当前楼层

```bash
rostopic echo /mapping/current_floor
```

探索阶段预期依次出现 `0`、`1`、`2`；任务返航时最终回到 `0`。

### 5.4 安全状态

```bash
rostopic echo /danger_search/safety_stop
```

正常运行应保持：

```text
data: False
```

### 5.5 实际速度

```bash
rostopic echo /danger_search/cmd_vel_sent
```

进入电梯时，探索节点的电梯速度约为 `+0.40 m/s`；退出电梯时方向相反。经过控制仲裁后，`cmd_vel_sent` 可能经过缩放，不一定仍显示 `0.40`。

## 6. 可选：记录完整测试日志

任务开始前，在另一个终端执行：

```bash
cd /home/ruilinli/danger_search_ws
source /opt/ros/noetic/setup.bash
source /home/ruilinli/SimEnv/devel/setup.bash
source devel/setup.bash --extend

mkdir -p test_logs

rosbag record \
  -O test_logs/seed42_full_multifloor \
  /mission/status \
  /mission/active \
  /exploration/status \
  /exploration/complete \
  /mapping/status \
  /mapping/current_floor \
  /navigation/health \
  /danger_search/transit_floor/goal \
  /danger_search/transit_floor/feedback \
  /danger_search/transit_floor/result \
  /danger_search/elevator_cmd_vel \
  /danger_search/cmd_vel_sent \
  /danger_search/safety_stop
```

任务结束后按 `Ctrl+C` 停止 rosbag。

## 7. 判断任务是否完成

查看 mission 状态：

```bash
rostopic echo -n 1 /mission/status
```

查看探索完成信号：

```bash
rostopic echo -n 1 /exploration/complete
```

检查最终楼层：

```bash
rostopic echo -n 1 /mapping/current_floor
```

最终应为：

```text
data: 0
```

检查速度归零：

```bash
rostopic echo -n 1 /danger_search/cmd_vel_sent
```

预期：

```text
linear:
  x: 0.0
  y: 0.0
angular:
  z: 0.0
```

检查安全状态：

```bash
rostopic echo -n 1 /danger_search/safety_stop
```

预期：

```text
data: False
```

## 8. 查看最终结果文件

直接查看：

```bash
cat /home/ruilinli/SimEnv/results/detected_danger.simulation_truth.json
```

格式化查看：

```bash
python3 -m json.tool \
  /home/ruilinli/SimEnv/results/detected_danger.simulation_truth.json
```

结果应包含类似字段：

```json
{
  "mission_status": "FINISHED",
  "run_profile": "simulation_truth",
  "localization_backend": "gazebo_truth",
  "official_eligible": false
}
```

`official_eligible=false` 是正常结果，因为这是 `simulation_truth` 测试，不是正式比赛运行。

## 9. 失败时记录的信息

任一换层失败后，不要继续手动发送换层目标。记录：

```bash
rostopic echo -n 1 /danger_search/transit_floor/result
rostopic echo -n 1 /danger_search/safety_stop
rostopic echo -n 1 /mapping/status
rostopic echo -n 1 /navigation/health
rostopic echo -n 1 /danger_search/cmd_vel_sent
```

同时保存终端二中的：

- Action `failure_code` 和 `message`。
- 当前换层 phase。
- 当前楼层和 map epoch。
- 门扫描验证结果。
- swept-footprint 障碍检查结果。
- 是否出现 `imu_stale`、跌倒或导航超时。

## 10. 停止测试

按以下顺序停止：

1. rosbag 终端按 `Ctrl+C`。
2. danger_search 的终端二按 `Ctrl+C`。
3. SimEnv 的终端一按 `Ctrl+C`。
4. 如果第一次只停止控制器，再按一次 `Ctrl+C` 停止 Gazebo。

## 11. 坐标模式不要混用

本文档使用标准出生点：

```text
ROBOT_X=0.0
ROBOT_Y=5.0
ROBOT_YAW=1.5708
```

因此必须使用：

```text
fixed_elevator_hall_x=-2.40
fixed_elevator_hall_y=-1.65
fixed_elevator_hall_into_yaw=-1.5707963
```

不要在该出生点下使用 `0.80, 0.00, 0.00`。该坐标只适用于机器人出生在电梯正前方 `(0.85, 2.60, 0.0)` 的直接进梯测试。
