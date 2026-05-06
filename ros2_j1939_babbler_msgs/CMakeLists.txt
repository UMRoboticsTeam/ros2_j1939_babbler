cmake_minimum_required(VERSION 3.8)
project(ros2_j1939_babbler_msgs)

set(Python3_EXECUTABLE python)
set(DBC_PATH "${CMAKE_CURRENT_SOURCE_DIR}/Messages.dbc")

set(MSGS_DIR "${CMAKE_CURRENT_BUILD_DIR}/msg/")

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Run our python script to generate the msg definitions
execute_process(
        COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/MessageGenerator.py
        "${DBC_PATH}" "${MSGS_DIR}")

# Tell CMake that it needs to reconfigure if the DBC file changes
# This is because the msgs list (and the rosidl target list) is generated at configure time not build time
# So if a build notices that the DBC has changed, then that means we need new msgs, and thus we need to reconfigure
set_property(DIRECTORY APPEND PROPERTY
        CMAKE_CONFIGURE_DEPENDS "${DBC_PATH}"
)

# Make a list of our msg files and create the rosidl target with them
file(GLOB msg_files RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" CONFIGURE_DEPENDS "${MSGS_DIR}/*.msg")
rosidl_generate_interfaces(${PROJECT_NAME}
        "${msg_files}"
        DEPENDENCIES std_msgs builtin_interfaces
)

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

install(
        FILES ${DBC_PATH}
        DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
