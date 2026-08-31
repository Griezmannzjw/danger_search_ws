# Atomic internal/public envelope for one floor's occupancy grid.
std_msgs/Header header
int32 floor_id
uint64 map_epoch
uint64 map_version
nav_msgs/OccupancyGrid occupancy_grid
