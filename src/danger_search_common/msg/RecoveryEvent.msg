# 导航卡死恢复事件。header.frame_id 固定为 map。
uint8 PHASE_TRIGGERED=1
uint8 PHASE_SUCCEEDED=2
uint8 PHASE_FAILED=3

uint8 MANEUVER_NONE=0
uint8 MANEUVER_BACKUP=1
uint8 MANEUVER_STRAFE_LEFT=2
uint8 MANEUVER_STRAFE_RIGHT=3
uint8 MANEUVER_ROTATE=4

std_msgs/Header header
uint32 event_id
string active_goal_id
uint8 phase
uint8 maneuver
geometry_msgs/Pose stuck_pose
geometry_msgs/Pose goal_pose
uint32 attempt
float32 requested_distance
float32 achieved_distance
float32 min_clearance
