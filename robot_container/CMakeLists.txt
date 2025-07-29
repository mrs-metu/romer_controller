cmake_minimum_required(VERSION 3.8)
project(robot_container)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(romer_node_utils REQUIRED)
find_package(Eigen3 REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/State.msg"
  "srv/SetState.srv"
  "action/ControllerTrajectory.action"
  DEPENDENCIES
    std_msgs
    trajectory_msgs
    action_msgs
    builtin_interfaces
)

# Export dependencies
ament_export_include_directories(include)

# Install header files
install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
