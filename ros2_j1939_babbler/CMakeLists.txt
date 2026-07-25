cmake_minimum_required(VERSION 3.5)
project(ros2_j1939_babbler)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Fixes intellisense not finding ros2 headers https://youtrack.jetbrains.com/issue/CPP-29747/Certain-ROS2-package-headers-missing-from-Intellisense-when-using-a-Docker-toolchain
include_directories(SYSTEM /opt/ros/$ENV{ROS_DISTRO}/include)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(ros_babel_fish REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ros2_j1939_babbler_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

# Build third party libraries
add_subdirectory(third-party SYSTEM)

ament_auto_find_build_dependencies()

ament_auto_add_library(
  ${PROJECT_NAME} SHARED
        src/babel_bridge.cpp
        src/babel_bridge_impl.cpp
        src/static_bridge.cpp
        src/static_bridge_impl.cpp
)

ament_export_dependencies(ros2_j1939_babbler_msgs)

target_link_libraries(${PROJECT_NAME}
        ${ros2_j1939_babbler_msgs_LIBRARIES}
        mireo_can_utils
)

# Mark mireo directories as system so compiler shuts up about warnings in them
target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE $<TARGET_PROPERTY:mireo_can_utils,INTERFACE_INCLUDE_DIRECTORIES>)

target_compile_options(${PROJECT_NAME} PRIVATE -Wno-unused-function)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "ros2_j1939_babbler::BabelBridge"
  EXECUTABLE babel_bridge
)

rclcpp_components_register_node(${PROJECT_NAME}
        PLUGIN "ros2_j1939_babbler::StaticBridge"
        EXECUTABLE static_bridge
)

target_include_directories(${PROJECT_NAME}
        PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src/internal
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
