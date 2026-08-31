cmake_minimum_required(VERSION 3.0.2)
project(danger_search_common)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
  nav_msgs
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
  SetCurrentFloor.srv
  TraversePortal.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
  geometry_msgs
  nav_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
