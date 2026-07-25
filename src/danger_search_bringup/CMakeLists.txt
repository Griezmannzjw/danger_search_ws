cmake_minimum_required(VERSION 3.0.2)
project(danger_search_bringup)

find_package(catkin REQUIRED)

catkin_package()

# 安装 launch 和 config 文件
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
