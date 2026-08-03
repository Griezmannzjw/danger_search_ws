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
  pcl_ros
  pcl_conversions
)

find_package(PCL REQUIRED COMPONENTS common filters registration)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
  rospy
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  danger_search_common
  pcl_ros
  pcl_conversions
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

add_executable(lidar_odometry_node src/lidar_odometry_node.cpp)
target_link_libraries(lidar_odometry_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(lidar_odometry_node ${catkin_EXPORTED_TARGETS})

install(TARGETS lidar_odometry_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/pose_estimator.py
  scripts/scan_projector.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()
