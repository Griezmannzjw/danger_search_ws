cmake_minimum_required(VERSION 3.0.2)
project(danger_search_navigation)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf
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
  tf
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
  scripts/navigation_core.py
  scripts/navigation_monitor.py
  scripts/navigation_monitor_core.py
  scripts/navigation_config_guard.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_navigation_core.py)
  catkin_add_nosetests(test/test_navigation_monitor.py)
  catkin_add_nosetests(test/test_standard_config.py)
endif()
