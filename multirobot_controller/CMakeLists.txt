cmake_minimum_required(VERSION 3.8)
project(multirobot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(multi_robot_interface REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(square_controller_server src/square_controller_server.cpp)
ament_target_dependencies(square_controller_server
  rclcpp
  rclcpp_action
  nav_msgs
  std_msgs
  geometry_msgs
  multi_robot_interface
)

add_executable(square_controller_client src/square_controller_client.cpp)
ament_target_dependencies(square_controller_client
  rclcpp
  rclcpp_action
  nav_msgs
  std_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  multi_robot_interface
)

install(TARGETS
  square_controller_server
  square_controller_client
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
