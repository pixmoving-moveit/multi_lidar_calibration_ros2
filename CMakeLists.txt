cmake_minimum_required(VERSION 3.5)
project(multi_lidar_calibration)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# Declare a C++ executable

ament_auto_add_library(multi_lidar_calibration_icp SHARED
  src/multi_lidar_calibration_icp.cpp
)

ament_auto_add_executable(
  multi_lidar_calibration_icp_node
  src/multi_lidar_calibration_icp_node.cpp
)

target_link_libraries(
  multi_lidar_calibration_icp_node
  multi_lidar_calibration_icp
)

ament_auto_add_library(multi_lidar_calibration_ndt SHARED
  src/multi_lidar_calibration_ndt.cpp
)

ament_auto_add_executable(
  multi_lidar_calibration_ndt_node
  src/multi_lidar_calibration_ndt_node.cpp
)

target_link_libraries(
  multi_lidar_calibration_ndt_node
  multi_lidar_calibration_ndt
)

# install
ament_auto_package(
  INSTALL_TO_SHARE
  launch
  # config
)
