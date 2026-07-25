cmake_minimum_required(VERSION 3.0.2)
project(danger_search_common)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
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
  MappingStatus.msg
  NavigationHealth.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  StartMission.srv
  FinishMission.srv
  ReturnHome.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
  geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
