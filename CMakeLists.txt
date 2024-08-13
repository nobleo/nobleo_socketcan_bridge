# Copyright (C) 2024 Nobleo Technology B.V.
#
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.8)
project(nobleo_socketcan_bridge)

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(can_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(fmt REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

add_library(socketcan_bridge_component
  src/socketcan_bridge.cpp
  src/socketcan_bridge_node.cpp
)
target_include_directories(socketcan_bridge_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(socketcan_bridge_component
  can_msgs
  diagnostic_msgs
  diagnostic_updater
  rclcpp
  rclcpp_components
)
target_link_libraries(socketcan_bridge_component fmt::fmt)

rclcpp_components_register_node(
    socketcan_bridge_component
    PLUGIN "nobleo_socketcan_bridge::SocketCanBridgeNode"
    EXECUTABLE socketcan_bridge
)

install(
  TARGETS socketcan_bridge_component
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# EventsExecutor is supported from rclcpp version 21 or greater (iron)
if(rclcpp_VERSION_MAJOR GREATER_EQUAL 21)
  message(STATUS "EventsExecutor support enabled")
  add_executable(socketcan_bridge_ee src/socketcan_bridge_ee.cpp)
  target_link_libraries(socketcan_bridge_ee socketcan_bridge_component)

  install(TARGETS socketcan_bridge_ee
    DESTINATION lib/${PROJECT_NAME}
  )
else()
  message(STATUS "EventsExecutor support disabled")
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest)
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(unittests test/unittests.cpp)
  target_link_libraries(unittests socketcan_bridge_component)
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  socketcan_bridge_component
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
