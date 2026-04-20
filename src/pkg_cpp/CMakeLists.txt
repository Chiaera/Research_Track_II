cmake_minimum_required(VERSION 3.8)
project(pkg_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include)

add_library(move_robot_server SHARED src/move_robot_server.cpp)
ament_target_dependencies(move_robot_server rclcpp rclcpp_components rclcpp_action robot_interfaces tf2_msgs geometry_msgs tf2 tf2_ros)
rclcpp_components_register_nodes(move_robot_server "robot_namespace::MoveRobotServerNode")

add_library(move_robot_client SHARED src/move_robot_client.cpp)
ament_target_dependencies(move_robot_client rclcpp rclcpp_components rclcpp_action robot_interfaces)
rclcpp_components_register_nodes(move_robot_client "robot_namespace::MoveRobotClientNode")

install(TARGETS
  move_robot_server
  move_robot_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

ament_package()
