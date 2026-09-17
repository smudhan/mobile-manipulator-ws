cmake_minimum_required(VERSION 3.8)

project(mission_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ============================================================
# Dependencies
# ============================================================

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(control_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)

# ============================================================
# ROS2 Actions
# ============================================================

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Pick.action"
  "action/Place.action"
)

# ============================================================
# Include directories
# ============================================================

include_directories(include)

# ============================================================
# Mission Controller
# ============================================================

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

# ============================================================
# Arm Controller
# ============================================================

add_executable(arm_controller
  src/arm_controller.cpp
)

ament_target_dependencies(arm_controller
  rclcpp
  rclcpp_action
  control_msgs
)

# ============================================================
# Generated ROSIDL typesupport
# ============================================================

rosidl_get_typesupport_target(
  cpp_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

target_link_libraries(
  mission_controller
  "${cpp_typesupport_target}"
)

target_link_libraries(
  arm_controller
  "${cpp_typesupport_target}"
)

# ============================================================
# Install executables
# ============================================================

install(
  TARGETS
    mission_controller
    arm_controller
  DESTINATION lib/${PROJECT_NAME}
)

# ============================================================
# Install headers
# ============================================================

install(
  DIRECTORY include/
  DESTINATION include/
)

# ============================================================
# Install package resources
# ============================================================

install(
  DIRECTORY
    launch
    config
    action
  DESTINATION share/${PROJECT_NAME}
)

# ============================================================
# ROSIDL runtime
# ============================================================

ament_export_dependencies(rosidl_default_runtime)

ament_package()