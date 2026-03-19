cmake_minimum_required(VERSION 3.5)
project(fake_diff_drive)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(fake_diff_drive_node src/fake_diff_drive_node.cpp)
ament_target_dependencies(fake_diff_drive_node
  rclcpp geometry_msgs nav_msgs sensor_msgs tf2 tf2_ros tf2_geometry_msgs)

add_executable(fake_scan_node src/fake_scan_node.cpp)
ament_target_dependencies(fake_scan_node
  rclcpp sensor_msgs)

install(TARGETS fake_diff_drive_node fake_scan_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME})

ament_package()
