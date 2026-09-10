# traj_lo_core: the Traj-LO algorithm, stripped of its bag readers and its
# OpenGL GUI, behind a plain-data API boundary. See ../../THIRD_PARTY_NOTICES.md.
find_package(TBB REQUIRED)

add_library(traj_lo_core STATIC
  src/TrajLoCore.cpp
  upstream/src/core/odometry.cpp
  upstream/src/core/map_manager.cpp
)

# Only include/ is PUBLIC: it holds nothing but the plain-data boundary
# (Types.hpp) and TrajLoCore.h, neither of which includes a vendored header.
# The Traj-LO sources, Sophus and robin-map stay PRIVATE so that consumers of
# this library need none of them.
target_include_directories(traj_lo_core
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/upstream/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../Sophus
    ${CMAKE_CURRENT_SOURCE_DIR}/../robin-map/include
)

target_link_libraries(traj_lo_core PUBLIC Eigen3::Eigen PRIVATE TBB::tbb)

# Upstream sets this; Sophus otherwise wants fmt for its error messages.
target_compile_definitions(traj_lo_core PRIVATE SOPHUS_USE_BASIC_LOGGING)

# Traj-LO accumulates each segment's Hessian and gradient with
# tbb::parallel_reduce, whose split points and reduction order depend on task
# stealing, so the same input does not give the same trajectory twice. The
# deterministic variant fixes both and makes a run bit-exact and independent
# of the thread count, at a small cost in speed. On by default: the offline
# CLI is used for benchmark comparisons, where reproducibility is the point.
option(TRAJLO_DETERMINISTIC
  "Use tbb::parallel_deterministic_reduce for the point registration (reproducible runs)"
  ON)
if(TRAJLO_DETERMINISTIC)
  target_compile_definitions(traj_lo_core PRIVATE TRAJLO_DETERMINISTIC)
endif()

set_target_properties(traj_lo_core PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED ON
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest QUIET)
  if(ament_cmake_gtest_FOUND)
    ament_add_gtest(test_traj_lo_core test/test_traj_lo_core.cpp)
    target_link_libraries(test_traj_lo_core traj_lo_core)
  endif()
endif()
