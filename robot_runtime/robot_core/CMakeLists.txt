add_library(robot_core INTERFACE)
target_include_directories(robot_core INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(robot_core INTERFACE Eigen3::Eigen)
add_library(robot::core ALIAS robot_core)
