cmake_minimum_required(VERSION 3.0.2)
project(danger_search_common)

catkin_python_setup()

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  message_generation
  std_msgs
  geometry_msgs
  nav_msgs
)

add_action_files(
  FILES
  TransitFloor.action
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  DangerSource.msg
  DangerSourceArray.msg
  MissionState.msg
  MissionStatus.msg
  DetectionStatus.msg
  LocalizationStatus.msg
  FloorMapInfo.msg
  FloorOccupancyGrid.msg
  MappingStatus.msg
  NavigationHealth.msg
  RecoveryEvent.msg
)

add_service_files(
  FILES
  SwitchFloor.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
  geometry_msgs
  nav_msgs
)

catkin_package(
  CATKIN_DEPENDS
  actionlib_msgs
  message_runtime
  std_msgs
  geometry_msgs
  nav_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_message_contract.py)
  catkin_add_nosetests(test/test_short_range_safety.py)
endif()
