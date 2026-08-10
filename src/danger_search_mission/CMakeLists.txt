cmake_minimum_required(VERSION 3.0.2)
project(danger_search_mission)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  std_srvs
  actionlib
  actionlib_msgs
  move_base_msgs
  danger_search_common
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
  rospy
  std_msgs
  geometry_msgs
  std_srvs
  actionlib
  actionlib_msgs
  move_base_msgs
  danger_search_common
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/mission_manager.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_nosetests(test/test_mission_core.py)
  catkin_add_nosetests(test/test_mission_contract.py)
  add_rostest(test/mission_smoke.test)
endif()
