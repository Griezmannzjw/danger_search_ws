# 定位健康状态
std_msgs/Header header

# 跟踪状态枚举
uint8 STATE_INITIALIZING = 0
uint8 STATE_TRACKING = 1
uint8 STATE_DEGRADED = 2
uint8 STATE_RELOCALIZING = 3
uint8 STATE_LOST = 4

uint8 tracking_state                    # 当前跟踪状态
float64 pose_covariance_trace           # 协方差迹（位置+姿态）
bool drift_warning                      # 漂移告警
float32 drift_rate_linear               # 线漂移率估计 (m/s)
float32 drift_rate_angular              # 角漂移率估计 (rad/s)
bool pose_jump_detected                 # 位姿跳变检测

float32 last_correction_translation     # 上次修正平移量 (m)
float32 last_correction_rotation        # 上次修正旋转量 (rad)
uint64 correction_version               # 修正版本号（回环/重定位时递增）
string relocalization_event_id          # 重定位事件ID
time last_stable_time                   # 上次稳定时间
string status_reason                    # 状态原因说明
