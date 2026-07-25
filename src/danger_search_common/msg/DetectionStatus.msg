# 检测器健康状态
std_msgs/Header header

bool ready                              # 检测器初始化完成
bool input_fresh                        # 输入数据新鲜
float32 input_latency_ms                # 输入延迟（毫秒）

uint32 total_detections                 # 当前帧检测总数
uint32 confirmed_count                  # 已确认危险源数量
uint32 pending_verification             # 待复核数量

uint32 capability_version               # 能力参数版本
string status_reason                    # 状态原因说明
