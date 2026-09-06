# danger_search_control

控制执行层。`cmd_mux.py` 是最终 `/cmd_vel` 的唯一发布者；`posture_safety_monitor.py` 将全局 IMU 姿态安全状态锁存到最高优先级急停。

## 全局姿态安全

系统订阅 `/trunk_imu`。roll 或 pitch 超过 30°并持续 0.18 秒会锁存 `/danger_search/safety_stop=true`，可滤掉短暂门槛冲击，同时覆盖实测 39°与 70°翻倒。IMU 超过 0.25 秒未更新或四元数非法也会立即 fail-safe；只有姿态超限才锁存 `/danger_search/posture_fallen=true`，不会把所有急停伪报为翻倒。

自动恢复要求倾角不超过 15°并连续稳定 2 秒。显式服务 `/danger_search/reset_posture_safety` 也只在 IMU 新鲜且当前姿态安全时解除，原因发布到 `/danger_search/posture_safety_reason`。Mission 在 ENTERING、EXPLORING、RETURNING 任一阶段收到急停后取消导航和换层、停止探索，并原子写入 `mission_status=ERROR`，原因是 `posture_safety_stop`。

`/danger_search/imu_diagnostic` 提供 IMU 接收计数、到达间隔、header 时间戳、样本合法性和当前安全原因，仅用于定位传感器链路问题，不参与安全判定。

## 数据流

```text
/danger_search/nav_cmd_vel
            ↓             /danger_search/entry_cmd_vel
            ↓             /danger_search/elevator_cmd_vel
            └───────────────┐
        cmd_mux
            ↓
/cmd_vel 与 /danger_search/cmd_vel_sent
```

## 正式职责

- 处理导航的 `linear.x`、`linear.y`、`angular.z`，其余 Twist 分量始终保持为零。
- 电梯命令采用 0.25 秒短租约，租约内优先于导航，超时后才回退到仍新鲜的导航命令。
- 进场命令采用独立 0.25 秒租约，硬限制 `|vx|<=0.40`、`vy=0`、`|wz|<=0.30`，纵向/角加速度分别为 0.75/1.5。
- 进场与电梯租约同时有效表示控制权冲突，立即 fail-closed 输出零速度。
- 拒绝 NaN、Inf 或无法转换为有限数值的导航速度。
- 先做三轴最大速度限幅，再做线速度和角速度加速度限制。
- 未收到有效命令、命令超时或外部急停时立即输出三轴零速度。
- 急停解除不恢复旧命令，必须收到新鲜、有效的导航命令后才从零重新加速。
- 同时发布实际输出和完全相同的诊断回显。

固定优先级如下：

```text
safety_stop
  > entry/elevator 租约冲突停车
  > entry 短租约
  > elevator 短租约
  > 非法输入 / 超时 / 未收到有效命令
  > 限幅和加速度限制后的导航命令
```

## 接口

### 输入

| 话题 | 类型 | 说明 |
|------|------|------|
| `/danger_search/nav_cmd_vel` | `geometry_msgs/Twist` | 导航速度输入 |
| `/danger_search/entry_cmd_vel` | `geometry_msgs/Twist` | 门槛进场短程速度输入，租约 0.25 秒 |
| `/danger_search/elevator_cmd_vel` | `geometry_msgs/Twist` | 电梯进出控制速度输入，租约 0.25 秒 |
| `/danger_search/safety_stop` | `std_msgs/Bool` | 由外部安全模块发布的急停输入，`true` 时立即停车 |

### 输出

| 话题 | 类型 | 说明 |
|------|------|------|
| `/cmd_vel` | `geometry_msgs/Twist` | 给机器人控制器的最终速度，control 唯一发布 |
| `/danger_search/cmd_vel_sent` | `geometry_msgs/Twist` | 实际输出回显，仅用于诊断和核对，不能作为正式里程计输入 |

### 私有参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `nav_cmd_topic` | `/danger_search/nav_cmd_vel` | 导航输入话题 |
| `entry_cmd_topic` | `/danger_search/entry_cmd_vel` | 进场短程控制输入话题 |
| `elevator_cmd_topic` | `/danger_search/elevator_cmd_vel` | 电梯控制输入话题 |
| `output_cmd_topic` | `/cmd_vel` | 最终输出话题 |
| `sent_cmd_topic` | `/danger_search/cmd_vel_sent` | 输出回显话题 |
| `safety_stop_topic` | `/danger_search/safety_stop` | 外部急停输入话题 |
| `enable_safety` | `true` | 是否启用命令超时停车 |
| `cmd_timeout_s` | `0.5` | 有效导航命令最大允许间隔，单位秒 |
| `elevator_timeout_s` | `0.25` | 电梯输入短租约，单位秒 |
| `entry_timeout_s` | `0.25` | 进场输入短租约，单位秒 |
| `entry_max_linear_speed` | `0.40` | 进场纵向速度硬上限 |
| `entry_max_angular_speed` | `0.30` | 进场角速度硬上限 |
| `entry_max_linear_accel` | `4.0` | 进场纵向加速度上限；在 0.10 s 内进入经烟测可执行的 0.40 m/s 步态，仍受硬速度上限约束 |
| `entry_max_angular_accel` | `1.50` | 进场角加速度上限 |
| `max_linear_speed` | `0.40` | `linear.x` 最大绝对速度，单位米每秒 |
| `max_lateral_speed` | `0.25` | `linear.y` 最大绝对速度，单位米每秒 |
| `max_angular_speed` | `0.80` | `angular.z` 最大绝对速度，单位弧度每秒 |
| `max_linear_accel` | `3.0` | `linear.x` 纵向加速度上限，与 10 Hz DWA 动态窗口一致 |
| `max_lateral_accel` | `2.0` | `linear.y` 横向加速度上限 |
| `max_angular_accel` | `8.0` | `angular.z` 角加速度上限，使首周期可达 0.8 rad/s |
| `max_dt_s` | `0.10` | 加速度计算使用的最大时间步长，单位秒 |
| `output_rate` | `50` | 输出频率，单位赫兹 |

所有频率、超时、速度上限、加速度上限和 `max_dt_s` 必须是正有限数值；非法配置会记录错误并拒绝启动。仿真时钟倒退、停住或一次跳跃过大时，控制器不会产生异常速度变化。

## 启动和手动测试

正式仿真前应先让 Unitree 进入 `/cmd_vel` 控制模式。系统入口会启动姿态安全发布者；其他安全模块仍可通过同一话题请求急停。

下面的隔离测试把四个话题都改到 `/test` 命名空间，不启动 Gazebo，也不会向真实 `/cmd_vel` 发运动命令：

```bash
source /opt/ros/noetic/setup.bash
source /home/ruilinli/SimEnv/danger_search_ws/devel/setup.bash
roscore
```

另开终端启动控制节点：

```bash
source /opt/ros/noetic/setup.bash
source /home/ruilinli/SimEnv/danger_search_ws/devel/setup.bash
rosrun danger_search_control cmd_mux.py \
  _nav_cmd_topic:=/test/nav_cmd_vel \
  _elevator_cmd_topic:=/test/elevator_cmd_vel \
  _output_cmd_topic:=/test/cmd_vel \
  _sent_cmd_topic:=/test/cmd_vel_sent \
  _safety_stop_topic:=/test/safety_stop
```

再开终端观察最终输出和回显：

```bash
rostopic echo /test/cmd_vel
rostopic echo /test/cmd_vel_sent
```

发布正常命令和超过 `cmd_timeout_s` 的静默，验证平滑输出和超时停车：

```bash
rostopic pub -r 10 /test/nav_cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.20, y: 0.10}, angular: {z: 0.40}}'
```

发布大速度验证三轴截断，再发布外部急停验证立即停车：

```bash
rostopic pub -1 /test/nav_cmd_vel geometry_msgs/Twist \
  '{linear: {x: 9.0, y: -9.0}, angular: {z: 9.0}}'
rostopic pub -1 /test/safety_stop std_msgs/Bool '{data: true}'
rostopic pub -1 /test/safety_stop std_msgs/Bool '{data: false}'
```

解除急停后需重新发布有效导航命令；`/test/cmd_vel` 和 `/test/cmd_vel_sent` 在每条路径上应保持内容一致。

也可以在控制节点运行期间，另开终端执行自动隔离 smoke test。它会验证三轴正常输出、速度截断、超时停车、急停、解除急停后的重新平滑启动和指令回显：

```bash
source /opt/ros/noetic/setup.bash
source /home/ruilinli/SimEnv/danger_search_ws/devel/setup.bash
python3 /home/ruilinli/SimEnv/danger_search_ws/src/danger_search_control/test/cmd_mux_smoke_test.py
```

该脚本只发布和订阅 `/test/*`，不会启动 Gazebo，也不会向真实 `/cmd_vel` 发送命令。

## 明确不在本实现内

- 除导航、电梯短租约和急停外的其他速度仲裁或手动遥控。
- 自动障碍急停和碰撞检测（姿态翻倒检测已经实现）。
- `/control/status` 或任何新的 ROS msg、srv、action、急停话题。
- Unitree 控制器侧的硬件命令看门狗。

上述能力由后续阶段或系统集成方负责，不能从本节点当前的急停订阅推断为已实现。
