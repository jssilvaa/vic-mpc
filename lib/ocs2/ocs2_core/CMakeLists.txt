# Minimal transplant of ocs2_core's foundational substrate (Types, LoadData,
# LinearInterpolation, Lookup, Numerics, NumericTraits) from the reference's
# now-initialized OCS2 submodule. The upstream package is ament/ROS2-based and
# depends on ocs2_thirdparty (CppAD); we deliberately vendor only the ROS-free,
# CppAD-free subset and compile it with plain CMake + C++20. See
# reference_ocs2_source_available + reference_macos_deviations memories.
find_package(Boost REQUIRED)

add_library(ocs2_core STATIC
  src/Types.cpp
  src/reference/ModeSchedule.cpp
  src/reference/TargetTrajectories.cpp)

target_include_directories(ocs2_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

# PUBLIC so consumers of LoadData.h (boost property_tree) and Types.h (Eigen)
# inherit the include paths.
target_link_libraries(ocs2_core PUBLIC
  Eigen3::Eigen
  Boost::boost)

add_library(ocs2::core ALIAS ocs2_core)
