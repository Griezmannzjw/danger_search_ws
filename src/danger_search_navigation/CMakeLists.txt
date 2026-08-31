cmake_minimum_required(VERSION 3.0.2)
project(danger_search_navigation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf
  tf2
  tf2_ros
  actionlib
  actionlib_msgs
  move_base_msgs
  nav_core
  pluginlib
  base_local_planner
  costmap_2d
  danger_search_common
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES unitree_escape_recovery
  CATKIN_DEPENDS
  roscpp
  rospy
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf
  tf2
  tf2_ros
  actionlib
  actionlib_msgs
  move_base_msgs
  nav_core
  pluginlib
  base_local_planner
  costmap_2d
  danger_search_common
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(unitree_escape_recovery
  src/escape_recovery_core.cpp
  src/unitree_escape_recovery.cpp
)
add_dependencies(unitree_escape_recovery ${catkin_EXPORTED_TARGETS})
target_link_libraries(unitree_escape_recovery ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS
  scripts/nav_controller.py
  scripts/navigation_core.py
  scripts/navigation_monitor.py
  scripts/navigation_monitor_core.py
  scripts/navigation_config_guard.py
  scripts/navigation_command_mux.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS unitree_escape_recovery
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES recovery_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_navigation_core.py)
  catkin_add_nosetests(test/test_navigation_monitor.py)
  catkin_add_nosetests(test/test_navigation_monitor_callbacks.py)
  catkin_add_nosetests(test/test_navigation_config_guard.py)
  catkin_add_nosetests(test/test_standard_config.py)
  catkin_add_gtest(test_escape_recovery
    test/test_escape_recovery.cpp
    src/escape_recovery_core.cpp
  )
  if(TARGET test_escape_recovery)
    target_link_libraries(test_escape_recovery ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_dwa_velocity_samples
    test/test_dwa_velocity_samples.cpp
  )
  if(TARGET test_dwa_velocity_samples)
    target_link_libraries(test_dwa_velocity_samples ${catkin_LIBRARIES})
  endif()
endif()
