# 任务状态
std_msgs/Header header

string mission_state                    # IDLE/EXPLORING/RETURNING/FINISHED/ERROR 等
int32 current_floor                     # 当前楼层
time start_time                         # 任务开始时间
duration elapsed_time                   # 已耗时
duration scored_exploration_time        # 计分有效探索时间

string active_goal_id                   # 当前活动目标ID
string map_coverage_summary             # 地图覆盖率摘要
string topology_debt_summary            # 拓扑债务摘要
string room_visibility_summary          # 房间可见域摘要

uint32 remaining_frontier_count         # 剩余前沿数量
uint64 localization_correction_version  # 当前定位修正版本
string finish_reason                    # 结束原因
