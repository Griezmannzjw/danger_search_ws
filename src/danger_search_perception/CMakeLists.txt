cmake_minimum_required(VERSION 3.0.2)
project(danger_search_perception)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  geometry_msgs
  cv_bridge
  tf2_ros
  danger_search_common
)

catkin_package(
  CATKIN_DEPENDS
  rospy
  sensor_msgs
  geometry_msgs
  cv_bridge
  tf2_ros
  danger_search_common
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/danger_detector.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
