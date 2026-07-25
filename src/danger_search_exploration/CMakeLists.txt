cmake_minimum_required(VERSION 3.0.2)
project(danger_search_exploration)

find_package(catkin REQUIRED COMPONENTS
  rospy
  nav_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf2_ros
  actionlib
  actionlib_msgs
  move_base_msgs
  danger_search_common
)

catkin_package(
  CATKIN_DEPENDS
  rospy
  nav_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf2_ros
  actionlib
  actionlib_msgs
  move_base_msgs
  danger_search_common
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/exploration_planner.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
