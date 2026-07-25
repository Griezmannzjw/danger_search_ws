# 建图状态
std_msgs/Header header

bool ready                              # 初始化完成
bool stable                             # 地图稳定
bool lost                               # 定位/建图丢失
int32 current_floor                     # 当前楼层（从0开始）

FloorMapInfo[] floor_maps               # 各楼层地图版本信息
string status_reason                    # 状态原因说明
