# humanoid_centroidal_mpc — the centroidal MPC formulation package. So far header-only
# (CentroidalMpcRobotModel); grows with the dynamics/cost/constraint terms (D3 #30+).
add_library(humanoid_centroidal_mpc INTERFACE)

target_include_directories(humanoid_centroidal_mpc INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_link_libraries(humanoid_centroidal_mpc INTERFACE
  humanoid::common_mpc
  ocs2::centroidal_model
  ocs2::pinocchio_interface
  ocs2::robotic_tools)

add_library(humanoid::centroidal_mpc ALIAS humanoid_centroidal_mpc)

# D3 #29d verify: config -> custom pinocchio interface -> CentroidalModelInfo ->
# CentroidalMpcRobotModel round-trip + A_b^-1 generalized-velocity solve.
add_executable(centroidalModelDemo exe/centroidalModelDemo.cpp)
target_link_libraries(centroidalModelDemo PRIVATE humanoid::centroidal_mpc)
set_target_properties(centroidalModelDemo PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
