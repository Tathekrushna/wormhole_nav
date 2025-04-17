cmake_minimum_required(VERSION 3.8)
project(wormhole_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(SQLite3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate interfaces for actions
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveRobot.action"
  DEPENDENCIES builtin_interfaces
)

# Include directories
include_directories(include)

# Add executable
add_executable(wormhole_action_server src/wormhole_action_server.cpp)

# Link dependencies
ament_target_dependencies(wormhole_action_server
  rclcpp
  nav2_msgs
  nav2_util
  geometry_msgs
  tf2
  tf2_geometry_msgs
  rclcpp_action
)

# Link SQLite3 library
target_link_libraries(wormhole_action_server SQLite::SQLite3)

# Install executable
install(TARGETS wormhole_action_server DESTINATION lib/${PROJECT_NAME})

# Testing dependencies
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

