# danger_search_control

控制执行层模块。

## 职责

1. **速度仲裁**：多路速度指令按优先级合并输出
2. **安全监控**：指令超时自动停车、急停
3. **加速度限制**：平滑速度输出，避免冲击
4. **指令回显**：发布已发送的速度指令，供定位模块使用

## 节点

### cmd_mux.py
速度多路复用器。

#### 订阅话题
| 话题 | 类型 | 优先级 | 说明 |
|------|------|--------|------|
| `/danger_search/nav_cmd_vel` | `geometry_msgs/Twist` | 10 | 导航速度指令 |
| `/danger_search/safety_stop` | `std_msgs/Bool` | 100 | 安全停车信号 |

#### 发布话题
| 话题 | 类型 | 说明 |
|------|------|------|
| `/cmd_vel` | `geometry_msgs/Twist` | 最终输出给机器人控制器 |
| `/danger_search/cmd_vel_sent` | `geometry_msgs/Twist` | 已发送指令回显 |

## 安全机制

1. **指令超时保护**：超过 `cmd_timeout_s` 未收到导航指令，自动停车
2. **加速度限制**：线加速度和角加速度限制，输出平滑
3. **安全停车通道**：`/danger_search/safety_stop` 最高优先级，触发立即停车

## 配置参数

见 `config/default.yaml`。

## 扩展方向

- 增加手动遥控输入通道（优先级 50）
- 增加摔倒检测自动停车
- 增加碰撞检测急停
- 增加速度指令可视化调试
