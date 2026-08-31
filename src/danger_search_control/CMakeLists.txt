cmake_minimum_required(VERSION 3.0.2)
project(danger_search_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  std_msgs
  sensor_msgs
  std_srvs
  danger_search_common
)

catkin_package(
  CATKIN_DEPENDS
  rospy
  geometry_msgs
  std_msgs
  sensor_msgs
  std_srvs
  danger_search_common
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/cmd_mux.py
  scripts/posture_safety_monitor.py
  scripts/posture_safety_core.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_cmd_mux.py)
  catkin_add_nosetests(test/test_posture_safety.py)
endif()
