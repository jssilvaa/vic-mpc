# Transplanted ocs2_pinocchio_interface: our PinocchioInterface (Model+Data+urdfTree
# bundle), relocated out of humanoid_common_mpc so lower-level packages
# (ocs2_centroidal_model) can depend on it without a circular dependency. Matches the
# reference's package layout + the <ocs2_pinocchio_interface/PinocchioInterface.h>
# include path. Base layer: depends only on ocs2_core + pinocchio + urdfdom + Eigen.
find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED IMPORTED_TARGET pinocchio)

# urdfdom (for <urdf_model/model.h>) — same brew-layout derivation as robot_model.
find_package(urdfdom CONFIG REQUIRED
  HINTS /opt/homebrew/Cellar/urdfdom/6.0.0/lib/urdfdom/cmake
        /opt/homebrew/opt/urdfdom/lib/urdfdom/cmake)
find_package(urdfdom_headers CONFIG REQUIRED
  HINTS /opt/homebrew/Cellar/urdfdom_headers/3.0.0/lib/urdfdom_headers/cmake
        /opt/homebrew/opt/urdfdom_headers/lib/urdfdom_headers/cmake)
set(URDFDOM_PREFIX_INCLUDE "${urdfdom_DIR}/../../../include")
set(URDFDOM_HEADERS_PREFIX_INCLUDE "${urdfdom_headers_DIR}/../../../include")

add_library(ocs2_pinocchio_interface STATIC
  src/PinocchioInterface.cpp)

target_include_directories(ocs2_pinocchio_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${URDFDOM_PREFIX_INCLUDE}
  ${URDFDOM_PREFIX_INCLUDE}/urdfdom
  ${URDFDOM_HEADERS_PREFIX_INCLUDE}
  ${URDFDOM_HEADERS_PREFIX_INCLUDE}/urdfdom_headers)

target_link_libraries(ocs2_pinocchio_interface PUBLIC
  ocs2::core
  PkgConfig::pinocchio
  Eigen3::Eigen
  ${urdfdom_LIBRARIES})

add_library(ocs2::pinocchio_interface ALIAS ocs2_pinocchio_interface)
