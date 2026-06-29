cmake_minimum_required(VERSION 3.8)
project(ros2_j1939_babbler_msgs)

# This can be overridden by specifying "-DDBC_PATH=..." when calling cmake
set(DBC_PATH "${CMAKE_CURRENT_SOURCE_DIR}/Messages.dbc" CACHE STRING "Path to symbol file to generate ROS messages for")

set(Python3_EXECUTABLE python)

set(MSGS_DIR "${CMAKE_CURRENT_BINARY_DIR}/msg/")
set(GENERATED_HEADER_OUTPUT_FOLDER "${CMAKE_CURRENT_BINARY_DIR}/include/${PROJECT_NAME}")
set(GENERATED_SOURCE_OUTPUT_FOLDER "${CMAKE_CURRENT_BINARY_DIR}/src")

set(TYPE_CONVERSION_HEADER_PATH "${GENERATED_HEADER_OUTPUT_FOLDER}/type_conversion.hpp")
set(C_TYPE_DEFINITIONS_HEADER_PATH "${GENERATED_HEADER_OUTPUT_FOLDER}/can_message_types.h")
set(TYPE_CONVERSION_HELPERS_HEADER_PATH "${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/type_conversion_helpers.hpp")
set(C_TYPE_DEFINITIONS_SOURCE_PATH "${GENERATED_SOURCE_OUTPUT_FOLDER}/can_message_types.c")

set(TYPE_CONVERSION_LIBRARY_NAME ros2_j1939_babbler_msgs_type_conversions)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Run our python script to generate the ROS message definitions and conversion functions
execute_process(
        COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/MessageGenerator.py
        "${DBC_PATH}" "${MSGS_DIR}" "${TYPE_CONVERSION_HEADER_PATH}"
        "${GENERATED_HEADER_OUTPUT_FOLDER}" "${GENERATED_SOURCE_OUTPUT_FOLDER}")

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

# Compile our C type definitions in with a new library in the package
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
ament_auto_add_library(${TYPE_CONVERSION_LIBRARY_NAME} SHARED
        ${C_TYPE_DEFINITIONS_SOURCE_PATH}
)
target_include_directories(${TYPE_CONVERSION_LIBRARY_NAME} PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(${TYPE_CONVERSION_LIBRARY_NAME} PUBLIC "${cpp_typesupport_target}")
ament_target_dependencies(${TYPE_CONVERSION_LIBRARY_NAME} PUBLIC ${DEPENDENCIES})

# Export dependencies downstream to client packages.
ament_export_dependencies(
        rosidl_default_runtime
        ${DEPENDENCIES}
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
        FILES "${DBC_PATH}"
        DESTINATION share/${PROJECT_NAME}/
)
install(
        FILES "${TYPE_CONVERSION_HEADER_PATH}" "${TYPE_CONVERSION_HELPERS_HEADER_PATH}" "${C_TYPE_DEFINITIONS_HEADER_PATH}"
        DESTINATION include/${PROJECT_NAME}/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_auto_package()
