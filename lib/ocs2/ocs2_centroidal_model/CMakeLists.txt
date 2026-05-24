# Transplanted ocs2_centroidal_model (CppAD-stripped): CentroidalModelInfo + the
# createCentroidalModelInfo / loadCentroidalType / loadDefaultJointState factory
# functions. OCS2's createPinocchioInterface builders + the toCppAd()/ad_scalar_t
# machinery are intentionally omitted. Depends on ocs2_pinocchio_interface (for the
# relocated PinocchioInterface) — clean base-layer dependency, no inversion.
find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED IMPORTED_TARGET pinocchio)

add_library(ocs2_centroidal_model STATIC
  src/CentroidalModelInfo.cpp
  src/FactoryFunctions.cpp
  src/CentroidalModelPinocchioMapping.cpp
  src/ModelHelperFunctions.cpp
  src/PinocchioCentroidalDynamics.cpp)

target_include_directories(ocs2_centroidal_model PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_link_libraries(ocs2_centroidal_model PUBLIC
  ocs2::core
  ocs2::pinocchio_interface
  ocs2::robotic_tools
  PkgConfig::pinocchio
  Eigen3::Eigen)

add_library(ocs2::centroidal_model ALIAS ocs2_centroidal_model)
