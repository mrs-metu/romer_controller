cmake_minimum_required(VERSION 3.8)
project(ros_custom_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(romer_node_utils REQUIRED)
find_package(robot_container REQUIRED)
find_package(romer_custom_estimator REQUIRED)
find_package(romer_custom_hardware_adapter REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# Install
install(
  DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  std_srvs
  romer_node_utils
  robot_container
  romer_custom_estimator
  romer_custom_hardware_adapter
)

ament_package()