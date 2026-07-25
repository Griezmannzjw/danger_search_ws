cmake_minimum_required(VERSION 3.0.2)
project(danger_search_navigation)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  nav_msgs
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
  geometry_msgs
  nav_msgs
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
  scripts/nav_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
