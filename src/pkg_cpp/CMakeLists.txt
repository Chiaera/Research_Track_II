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
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(move_robot_server src/move_robot_server.cpp)
ament_target_dependencies(move_robot_server rclcpp rclcpp_action robot_interfaces nav_msgs geometry_msgs)

install(TARGETS
  move_robot_server
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
