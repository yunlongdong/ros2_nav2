cmake_minimum_required(VERSION 3.5)
project(nav2_bringup_custom)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch params map behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
