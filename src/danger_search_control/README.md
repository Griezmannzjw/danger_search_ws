# danger_search_control

控制执行层。`cmd_mux.py` 是最终 `/cmd_vel` 的唯一发布者，当前 P0 只有一条导航速度通道和一个最高优先级的外部急停门。

## 数据流

```text
/danger_search/nav_cmd_vel
            ↓
        cmd_mux
            ↓
/cmd_vel 与 /danger_search/cmd_vel_sent
```

## P0 职责

- 处理导航的 `linear.x`、`linear.y`、`angular.z`，其余 Twist 分量始终保持为零。
- 拒绝 NaN、Inf 或无法转换为有限数值的导航速度。
- 先做三轴最大速度限幅，再做线速度和角速度加速度限制。
- 未收到有效命令、命令超时或外部急停时立即输出三轴零速度。
- 急停解除不恢复旧命令，必须收到新鲜、有效的导航命令后才从零重新加速。
- 同时发布实际输出和完全相同的诊断回显。

固定优先级如下：

```text
safety_stop
  > 非法输入 / 超时 / 未收到有效命令
  > 限幅和加速度限制后的导航命令
```

## 接口

### 输入

| 话题 | 类型 | 说明 |
|------|------|------|
| `/danger_search/nav_cmd_vel` | `geometry_msgs/Twist` | 导航速度输入 |
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
| `output_cmd_topic` | `/cmd_vel` | 最终输出话题 |
| `sent_cmd_topic` | `/danger_search/cmd_vel_sent` | 输出回显话题 |
| `safety_stop_topic` | `/danger_search/safety_stop` | 外部急停输入话题 |
| `enable_safety` | `true` | 是否启用命令超时停车 |
| `cmd_timeout_s` | `0.5` | 有效导航命令最大允许间隔，单位秒 |
| `max_linear_speed` | `0.30` | `linear.x` 最大绝对速度，单位米每秒 |
| `max_lateral_speed` | `0.25` | `linear.y` 最大绝对速度，单位米每秒 |
| `max_angular_speed` | `0.80` | `angular.z` 最大绝对速度，单位弧度每秒 |
| `max_linear_accel` | `1.0` | `linear.x`、`linear.y` 每次变化的最大加速度 |
| `max_angular_accel` | `2.0` | `angular.z` 每次变化的最大加速度 |
| `max_dt_s` | `0.10` | 加速度计算使用的最大时间步长，单位秒 |
| `output_rate` | `50` | 输出频率，单位赫兹 |

所有频率、超时、速度上限、加速度上限和 `max_dt_s` 必须是正有限数值；非法配置会记录错误并拒绝启动。仿真时钟倒退、停住或一次跳跃过大时，控制器不会产生异常速度变化。

## 启动和手动测试

正式仿真前应先让 Unitree 进入 `/cmd_vel` 控制模式。当前工作区没有 `/danger_search/safety_stop` 的发布者，它需要由系统集成方提供；订阅该话题不等于已经实现自动障碍急停。

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

## 明确不在本 P0 实现内

- 多路速度仲裁和手动遥控。
- 自动障碍急停、摔倒检测和碰撞检测。
- `/control/status` 或任何新的 ROS msg、srv、action、急停话题。
- Unitree 控制器侧的硬件命令看门狗。

上述能力由后续阶段或系统集成方负责，不能从本节点当前的急停订阅推断为已实现。
