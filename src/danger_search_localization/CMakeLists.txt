cmake_minimum_required(VERSION 3.0.2)
project(danger_search_localization)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  danger_search_common
)

catkin_package(
  CATKIN_DEPENDS
  rospy
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  danger_search_common
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/pose_estimator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
