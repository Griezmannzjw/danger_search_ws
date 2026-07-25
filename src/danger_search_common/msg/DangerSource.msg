# 单个危险源/干扰源检测结果
# 探索规划接口规范 v1.0

# 分类常量
uint8 CLASS_UNKNOWN = 0
uint8 CLASS_DANGER_RED_SPHERE = 1       # 危险源：红色球体
uint8 CLASS_DISTRACTOR_RED_CUBE = 2     # 干扰源：红色方块
uint8 CLASS_DISTRACTOR_GREEN_SPHERE = 3 # 干扰源：绿色球体

string detection_id                     # 单次检测唯一ID
string track_id                         # 跟踪轨迹ID
uint8 class_id                          # 分类

geometry_msgs/PointStamped position     # 位置（map坐标系，带时间戳）
float64[9] position_covariance          # 3x3位置协方差矩阵（行优先）

int32 floor_id                          # 所在楼层（从0开始）
float32 confidence                      # 置信度 0.0~1.0
bool confirmed                          # 是否已确认入库
bool verification_required              # 是否需要异视角复核

string[] possible_duplicate_track_ids   # 疑似重复轨迹ID列表
uint64 localization_correction_version  # 对应定位修正版本
time source_time                        # 原始检测时间
