cmake_minimum_required(VERSION 3.8)
project(mission_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate custom manipulation actions
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Pick.action"
  "action/Place.action"
)

include_directories(include)

add_executable(mission_controller
  src/mission_controller.cpp
)

ament_target_dependencies(mission_controller
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
  sensor_msgs
  std_srvs
)

install(
  TARGETS mission_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  DIRECTORY launch config action
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()