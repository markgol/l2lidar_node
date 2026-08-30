cmake_minimum_required(VERSION 3.22)

project(l2lidar_node
        VERSION 2.0.0
        LANGUAGES CXX
  )

# ============================
# Compiler settings
# ============================
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Enable Qt MOC/UIC/RCC automatically
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# ============================
# ROS2
# ============================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

# ============================
# Qt 6
# ============================
find_package(Qt6 REQUIRED COMPONENTS Core Network)
message(STATUS "Qt6_VERSION=${Qt6_VERSION}")
message(STATUS "Qt6_DIR=${Qt6_DIR}")

get_filename_component(
    QT6_CMAKE_DIR
    "${Qt6_DIR}"
    ABSOLUTE
)

get_filename_component(
    QT_RUNTIME_DIR
    "${QT6_CMAKE_DIR}/../.."
    ABSOLUTE
)

message(STATUS "QT_RUNTIME_DIR=${QT_RUNTIME_DIR}")

# ============================
# Include directories
# ============================
include_directories(
  include
  L2lidarClass/include
)

# ============================
# Sources
# ============================
set(SOURCES
  src/l2lidar_node.cpp
  L2lidarClass/src/L2lidar.cpp
  L2lidarClass/src/L2calibration.cpp
)

set(HEADERS
  L2lidarClass/include/L2lidar.h
  L2lidarClass/include/L2calibration.h
  L2lidarClass/include/RangeCalSegments.h
  L2lidarClass/include/PCpoint.h
  L2lidarClass/include/unitree_lidar_protocolL2.h
  L2lidarClass/include/unitree_lidar_utilitiesL2.h
  L2lidarClass/include/quaternion.h
)

# ============================
# Executable
# ============================
add_executable(l2lidar_node
  ${SOURCES}
  ${HEADERS}
  src/main.cpp
  include/l2lidar_node.hpp
  config/l2lidar_node.yaml
  launch/l2lidar.launch.py
  launch/l2lidar_with_rviz.launch.py
  "config/Herman1 cal overrides.csv"
)

target_link_libraries(l2lidar_node
  Qt6::Core
  Qt6::Network
)

ament_target_dependencies(l2lidar_node
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
  geometry_msgs
)

target_include_directories(l2lidar_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

set_target_properties(l2lidar_node PROPERTIES
  BUILD_RPATH "\$ORIGIN/lib"
  BUILD_RPATH_USE_ORIGIN TRUE
  INSTALL_RPATH "\$ORIGIN/lib"
)

# ============================
# Distribution
#
# Note: This is specific to Qt 6.11.2 build
# It uses libicu v73 which is bundled with the Qt 6.11.2 distro
# This is in conflict with the Ubuntu libicu v74
# The v73 shared libraries must be part of the distro
# Files are copied to the dist folder inside the project build folder
# for the specific architecture being built
# ============================
set(DIST_DIR ${CMAKE_BINARY_DIR}/dist)
message(STATUS "DIST_DIR=${DIST_DIR}")
add_custom_target(dist ALL
  COMMAND ${CMAKE_COMMAND} -E rm -rf ${DIST_DIR}
  COMMAND ${CMAKE_COMMAND} -E make_directory ${DIST_DIR}
  COMMAND ${CMAKE_COMMAND} -E make_directory ${DIST_DIR}/lib
  COMMAND ${CMAKE_COMMAND} -E make_directory ${DIST_DIR}/config
  COMMAND ${CMAKE_COMMAND} -E make_directory ${DIST_DIR}/launch
  COMMAND ${CMAKE_COMMAND} -E make_directory ${DIST_DIR}/rviz
  COMMAND ${CMAKE_COMMAND} -E make_directory ${DIST_DIR}/docs
  COMMAND ${CMAKE_COMMAND} -E make_directory ${DIST_DIR}/urdf
  COMMAND ${CMAKE_COMMAND} -E make_directory ${DIST_DIR}/meshes

  COMMAND ${CMAKE_COMMAND} -E copy
          $<TARGET_FILE:l2lidar_node>
          ${DIST_DIR}/

  COMMAND ${CMAKE_COMMAND} -E copy
          $<TARGET_FILE:Qt6::Core>
          ${DIST_DIR}/lib/libQt6Core.so.6

  COMMAND ${CMAKE_COMMAND} -E copy
          $<TARGET_FILE:Qt6::Network>
          ${DIST_DIR}/lib/libQt6Network.so.6

  COMMAND ${CMAKE_COMMAND} -E copy
          ${QT_RUNTIME_DIR}/libicui18n.so.73
          ${DIST_DIR}/lib/libicui18n.so.73

  COMMAND ${CMAKE_COMMAND} -E copy
          ${QT_RUNTIME_DIR}/libicuuc.so.73
          ${DIST_DIR}/lib/libicuuc.so.73

  COMMAND ${CMAKE_COMMAND} -E copy
          ${QT_RUNTIME_DIR}/libicudata.so.73
          ${DIST_DIR}/lib/libicudata.so.73

  COMMAND ${CMAKE_COMMAND} -E copy_directory
          ${CMAKE_SOURCE_DIR}/config
          ${DIST_DIR}/config

  COMMAND ${CMAKE_COMMAND} -E copy_directory
          ${CMAKE_SOURCE_DIR}/launch
          ${DIST_DIR}/launch

  COMMAND ${CMAKE_COMMAND} -E copy_directory
          ${CMAKE_SOURCE_DIR}/rviz
          ${DIST_DIR}/rviz

  COMMAND ${CMAKE_COMMAND} -E copy_directory
          ${CMAKE_SOURCE_DIR}/licenses
          ${DIST_DIR}

  COMMAND ${CMAKE_COMMAND} -E copy_directory
          ${CMAKE_SOURCE_DIR}/docs
          ${DIST_DIR}/docs

  COMMAND ${CMAKE_COMMAND} -E copy_directory
          ${CMAKE_SOURCE_DIR}/urdf
          ${DIST_DIR}/urdf

  COMMAND ${CMAKE_COMMAND} -E copy_directory
          ${CMAKE_SOURCE_DIR}/meshes
          ${DIST_DIR}/meshes

  DEPENDS l2lidar_node
)

# ============================
# Install
# ============================
install(TARGETS l2lidar_node
  DESTINATION lib/${PROJECT_NAME}
)

install(IMPORTED_RUNTIME_ARTIFACTS
  Qt6::Core
  Qt6::Network
  DESTINATION lib/${PROJECT_NAME}/lib
)

install(DIRECTORY
  launch
  config
  rviz
  meshes
  urdf
  DESTINATION share/${PROJECT_NAME}
)

# ============================
# Package export
# ============================
ament_package()
