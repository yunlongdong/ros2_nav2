cmake_minimum_required(VERSION 3.5)
project(nav2_bringup_custom)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch params map behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/nav_goal_status_monitor.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
