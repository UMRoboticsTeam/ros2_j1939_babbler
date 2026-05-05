cmake_minimum_required(VERSION 3.5)
project(generic_can_driver)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Fixes intellisense not finding ros2 headers https://youtrack.jetbrains.com/issue/CPP-29747/Certain-ROS2-package-headers-missing-from-Intellisense-when-using-a-Docker-toolchain
include_directories(SYSTEM /opt/ros/$ENV{ROS_DISTRO}/include)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(can_dbc_parser REQUIRED)
find_package(ros_babel_fish)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(
  ${PROJECT_NAME} SHARED
  src/generic_can_driver.cpp
)

ament_export_dependencies(can_dbc_parser)

target_compile_options(${PROJECT_NAME} PRIVATE -Wno-unused-function)

rclcpp_components_register_node(generic_can_driver
  PLUGIN "ros2_j1939::GenericCanDriver"
  EXECUTABLE generic_can_driver_exe
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)