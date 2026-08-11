cmake_minimum_required(VERSION 3.0.2)
project(danger_search_localization)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf
  tf2_ros
  tf2_geometry_msgs
  message_generation
  danger_search_common
  pcl_ros
  pcl_conversions
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters registration)
find_package(OpenMP QUIET)

catkin_python_setup()

add_message_files(FILES Pose6D.msg)
generate_messages(DEPENDENCIES geometry_msgs)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf
  tf2_ros
  tf2_geometry_msgs
  danger_search_common
  pcl_ros
  pcl_conversions
  message_runtime
  DEPENDS EIGEN3 PCL
)

include_directories(
  include
  third_party/fast_lio/include
  third_party/fast_lio/src
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

add_executable(lidar_odometry_node src/lidar_odometry_node.cpp)
target_link_libraries(lidar_odometry_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(lidar_odometry_node ${catkin_EXPORTED_TARGETS})

add_executable(sim_sensor_adapter_node src/sim_sensor_adapter_node.cpp)
target_link_libraries(sim_sensor_adapter_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(sim_sensor_adapter_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(lio_occupancy_mapper_node src/lio_occupancy_mapper_node.cpp)
target_link_libraries(lio_occupancy_mapper_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(lio_occupancy_mapper_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(fast_lio_mapping
  third_party/fast_lio/src/laserMapping.cpp
  third_party/fast_lio/src/preprocess.cpp
  third_party/fast_lio/include/ikd-Tree/ikd_Tree.cpp
)
target_compile_features(fast_lio_mapping PRIVATE cxx_std_14)
target_compile_definitions(fast_lio_mapping PRIVATE
  ROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/third_party/fast_lio/"
  MP_PROC_NUM=2
)
if(OpenMP_CXX_FOUND)
  target_link_libraries(fast_lio_mapping OpenMP::OpenMP_CXX)
endif()
target_link_libraries(fast_lio_mapping ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(fast_lio_mapping ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS lidar_odometry_node sim_sensor_adapter_node
  lio_occupancy_mapper_node fast_lio_mapping
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/pose_estimator.py
  scripts/scan_projector.py
  scripts/lio_interface.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_lidar_odometry_core test/test_lidar_odometry_core.cpp)
  if(TARGET test_lidar_odometry_core)
    target_link_libraries(test_lidar_odometry_core ${catkin_LIBRARIES} ${PCL_LIBRARIES})
  endif()
  catkin_add_nosetests(test/test_adapter.py)
  catkin_add_nosetests(test/test_config.py)
  catkin_add_nosetests(test/test_pose_filter.py)
  catkin_add_nosetests(test/test_pose_fusion.py)
  catkin_add_nosetests(test/test_scan_projection.py)
  catkin_add_nosetests(test/test_vertical_estimation.py)
endif()
