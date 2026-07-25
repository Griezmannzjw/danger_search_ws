# 导航健康状态
std_msgs/Header header

bool ready                              # 导航就绪
bool controller_active                  # 控制器活跃
bool stuck                              # 机器人卡住
bool fallen                             # 机器人摔倒
bool has_active_goal                    # 有活动导航目标

string active_goal_id                   # 当前目标ID
float32 progress                        # 执行进度 0.0~1.0
time last_cmd_time                      # 最后发送速度指令时间

string failure_code                     # 失败码（SUCCEEDED/UNREACHABLE/CANCELED/TIMEOUT/CONTROL_FAILED/ROBOT_FALLEN/LOCALIZATION_LOST）
string failure_detail                   # 失败详情
