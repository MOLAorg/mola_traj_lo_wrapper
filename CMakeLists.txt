# ------------------------------------------------------------------------------
#                       mola_traj_lo_wrapper
#
# Copyright (C) 2026, Jose Luis Blanco-Claraco
# All rights reserved.
# Released under GNU GPL v3. See LICENSE file.
# See THIRD_PARTY_NOTICES.md for the licensing terms of the vendored
# Traj-LO / Sophus / robin-map code under third_party/.
# ------------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.22...3.31)  # Ubuntu 22.04+

if("$ENV{ROS_VERSION}" STREQUAL "2")
    set(DETECTED_ROS2 TRUE)
endif()

project(mola_traj_lo_wrapper LANGUAGES CXX)

# MOLA CMake scripts: "mola_xxx()"
find_package(mola_common REQUIRED)

# find dependencies:
find_package(Eigen3 REQUIRED)
find_package(TBB REQUIRED)
find_package(mrpt_maps REQUIRED)
find_package(mrpt_obs REQUIRED)
find_package(CLI11 REQUIRED)
find_package(mola_kernel REQUIRED)
find_package(mola_yaml REQUIRED)

# ament_cmake / BUILD_TESTING must be set up before add_subdirectory() below,
# since traj_lo_core's own CMakeLists.txt uses ament_add_gtest():
if(DETECTED_ROS2)
  find_package(ament_cmake REQUIRED)

  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    find_package(ament_cmake_gtest REQUIRED)
    ament_lint_auto_find_test_dependencies()
  endif()
endif()

# -----------------------------------------------------------------------------
#  traj_lo_core: the ROS-free, MOLA-free Traj-LO algorithm library.
#  Unlike the other wrappers in this suite there is no PCL anywhere: Traj-LO
#  works on plain Eigen vectors and its own voxel map.
# -----------------------------------------------------------------------------
add_subdirectory(third_party/traj_lo_core)

# -----------------------------------------------------------------------------
#  mola_traj_lo_wrapper: the online mola-cli module
# -----------------------------------------------------------------------------
mola_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES
    module/src/TrajLoOdometry.cpp
    module/src/TrajLoOdometry_Adapters.cpp
    # Last:
    module/src/register.cpp
  PUBLIC_LINK_LIBRARIES
    Eigen3::Eigen
    mrpt::mrpt_maps
    mrpt::mrpt_obs
    mola::mola_kernel
    mola::mola_yaml
  PRIVATE_LINK_LIBRARIES
    # traj_lo_core is a plain (non ament/non exported) static lib: link it
    # PRIVATE so `install(EXPORT ...)` doesn't need to export it too (its
    # symbols get fully absorbed into this .so). The public header only needs
    # the light Types.hpp, whose include dir is added PUBLIC below.
    traj_lo_core
  CMAKE_DEPENDENCIES
    Eigen3
    mola_kernel
    mrpt_maps
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/module/include>
  # module/include/.../TrajLoOdometry.h (installed, public) only needs
  # traj_lo_core/Types.hpp, which is plain Eigen structs and pulls in none of
  # the vendored Sophus/robin-map/TBB headers.
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/traj_lo_core/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE
    module/src
)
install(
  DIRECTORY module/include/
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include
)
install(
  DIRECTORY third_party/traj_lo_core/include/
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include
)

# Install pipeline / mola-cli launch YAMLs:
install(
  DIRECTORY
    mola-cli-launchs
    pipelines
  DESTINATION
    share/${PROJECT_NAME}
)

# Install gui-* convenience scripts:
install(
  PROGRAMS
    scripts/mola-trajlo-gui-oxford-spires
  DESTINATION
    bin
)

# -----------------------------------------------------------------------------
#  Tests for the wrapper module itself (traj_lo_core has its own, next to its
#  sources). Only the per-point timing conversion so far: it is the classic
#  failure of this port (seconds vs. nanoseconds, and the azimuth fallback),
#  and getting it wrong is invisible except as a worse APE.
# -----------------------------------------------------------------------------
if(DETECTED_ROS2 AND BUILD_TESTING AND ament_cmake_gtest_FOUND)
  ament_add_gtest(test_scan_conversion module/test/test_scan_conversion.cpp)
  target_include_directories(test_scan_conversion PRIVATE module/src)
  target_link_libraries(test_scan_conversion ${PROJECT_NAME} traj_lo_core)
endif()

# -----------------------------------------------------------------------------
#  mola-traj-lo-cli: the offline, lossless CLI app
# -----------------------------------------------------------------------------
find_package(mola_input_mulran_dataset) # optional
find_package(mola_input_rawlog) # optional
find_package(mola_input_rosbag2) # optional
find_package(mola_input_rosbag1) # optional
find_package(mola_input_kitti_dataset) # optional

mola_add_executable(
  TARGET  mola-traj-lo-cli
  SOURCES apps/mola-traj-lo-cli.cpp
  LINK_LIBRARIES
    ${PROJECT_NAME}
    mrpt::mrpt_maps
    CLI11::CLI11
)

if(mola_input_mulran_dataset_FOUND)
  message(STATUS "mola-traj-lo-cli: With Mulran = YES")
  target_link_libraries(mola-traj-lo-cli mola::mola_input_mulran_dataset)
  target_compile_definitions(mola-traj-lo-cli PRIVATE HAVE_MOLA_INPUT_MULRAN)
else()
  message(STATUS "mola-traj-lo-cli: With Mulran = NO")
endif()

if(mola_input_rawlog_FOUND)
  message(STATUS "mola-traj-lo-cli: With Rawlog = YES")
  target_link_libraries(mola-traj-lo-cli mola::mola_input_rawlog)
  target_compile_definitions(mola-traj-lo-cli PRIVATE HAVE_MOLA_INPUT_RAWLOG)
else()
  message(STATUS "mola-traj-lo-cli: With Rawlog = NO")
endif()

if(mola_input_rosbag2_FOUND)
  message(STATUS "mola-traj-lo-cli: With rosbag2 = YES")
  target_link_libraries(mola-traj-lo-cli mola::mola_input_rosbag2)
  target_compile_definitions(mola-traj-lo-cli PRIVATE HAVE_MOLA_INPUT_ROSBAG2)
else()
  message(STATUS "mola-traj-lo-cli: With rosbag2 = NO")
endif()

if(mola_input_rosbag1_FOUND)
  message(STATUS "mola-traj-lo-cli: With rosbag1 = YES")
  target_link_libraries(mola-traj-lo-cli mola::mola_input_rosbag1)
  target_compile_definitions(mola-traj-lo-cli PRIVATE HAVE_MOLA_INPUT_ROSBAG1)
else()
  message(STATUS "mola-traj-lo-cli: With rosbag1 = NO")
endif()

if(mola_input_kitti_dataset_FOUND)
  message(STATUS "mola-traj-lo-cli: With KITTI = YES")
  target_link_libraries(mola-traj-lo-cli mola::mola_input_kitti_dataset)
  target_compile_definitions(mola-traj-lo-cli PRIVATE HAVE_MOLA_INPUT_KITTI)
else()
  message(STATUS "mola-traj-lo-cli: With KITTI = NO")
endif()

# -----------------------------------------------------------------------------
#  ROS2
# -----------------------------------------------------------------------------
if(DETECTED_ROS2)
  ament_package()
endif()
