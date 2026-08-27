# 建图状态
std_msgs/Header header

bool ready                              # 初始化完成
bool stable                             # 地图稳定
bool lost                               # 定位/建图丢失
int32 current_floor                     # 当前楼层（从0开始）
bool transitioning                      # 正在执行显式楼层切换，期间地图不可用于规划
uint64 map_epoch                        # 当前活动地图身份；切层或重置时单调递增
float64 floor_z_m                       # 当前楼层相对任务起点的标高（米）

FloorMapInfo[] floor_maps               # 各楼层地图版本信息
string status_reason                    # 状态原因说明
