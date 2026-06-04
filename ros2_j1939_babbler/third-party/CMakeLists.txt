cmake_minimum_required(VERSION 3.22.1)
project(mireo_can_utils)

# ********** Setup CMake **********

# Require C++20
if(CMAKE_CXX_STANDARD LESS 20)
    message(SEND_ERROR "${PROJECT_NAME} requires C++20 features")
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    # enables building a static library but later link it into a dynamic library
    add_compile_options(-fPIC)
endif()
if(NOT WIN32)
    # About -Wno-sign-conversion: With Clang, -Wconversion implies -Wsign-conversion. There are a number of
    # implicit sign conversions in gtest.cc, see https://ci.ros2.org/job/ci_osx/9381/clang/.
    # Hence disabling -Wsign-conversion for now until all those have eventually been fixed.
    # (from https://github.com/ros2/rcutils/pull/263#issuecomment-663252537)
    add_compile_options(-Wall -Wextra -Wconversion -Wno-sign-conversion -Wpedantic)
endif()

find_package(Boost REQUIRED)

add_library(${PROJECT_NAME})

target_sources(${PROJECT_NAME} PRIVATE
        mireo_can_utils/dbc/dbc_parser.cpp
)

set(public_headers # These files will be installed with the library
        mireo_can_utils/dbc/dbc_parser.h
        mireo_can_utils/any/any.h
        mireo_can_utils/any/tag_invoke.h
        mireo_can_utils/can/can_codec.h
        mireo_can_utils/can/can_kernel.h
        mireo_can_utils/v2c/v2c_transcoder.h
)
target_include_directories(${PROJECT_NAME} PUBLIC # Everything in this folder will be available during building
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/mireo_can_utils/>"
        "$<BUILD_INTERFACE:${Boost_INCLUDE_DIRS}>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
set_property(TARGET ${lib_target} PROPERTY PUBLIC_HEADER ${public_headers})
