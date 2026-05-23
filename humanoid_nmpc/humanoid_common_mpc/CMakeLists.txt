# pinocchio (brewed): uses pkg-config rather than CMake config on Homebrew
find_package(PkgConfig REQUIRED)
# GLOBAL so PkgConfig::pinocchio is nameable from other subdirectories
# (e.g. mujoco_sim_interface's pinocchioCrossCheck exe links it transitively).
pkg_check_modules(pinocchio REQUIRED IMPORTED_TARGET GLOBAL pinocchio)

# Boost (brewed): pinocchio's .pc on Homebrew does NOT propagate boost's include
# dir, so #include <boost/type_traits/...> from pinocchio/utils/check.hpp fails
# unless we add /opt/homebrew/include ourselves. Header-only, no link needed.
find_package(Boost REQUIRED)

# urdfdom: reuse the brew-layout include-dir derivation from robot_model. 
find_package(urdfdom CONFIG REQUIRED
  HINTS /opt/homebrew/Cellar/urdfdom/6.0.0/lib/urdfdom/cmake
        /opt/homebrew/opt/urdfdom/lib/urdfdom/cmake)
find_package(urdfdom_headers CONFIG REQUIRED
  HINTS /opt/homebrew/Cellar/urdfdom/3.0.0/lib/urdfdom_headers/cmake
        /opt/homebrew/opt/urdfdom/lib/urdfdom_headers/cmake)
set (URDFDOM_PREFIX_INCLUDE "${urdfdom_DIR}/../../../include")
set (URDFDOM_HEADERS_PREFIX_INCLUDE "${urdfdom_headers_DIR}/../../../include")

add_library(humanoid_common_mpc STATIC
  src/pinocchio_model/PinocchioInterface.cpp
  src/pinocchio_model/createPinocchioModel.cpp
  src/pinocchio_model/DynamicsHelperFunctions.cpp
)

target_include_directories(humanoid_common_mpc PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${URDFDOM_PREFIX_INCLUDE}
  ${URDFDOM_PREFIX_INCLUDE}/urdfdom
  ${URDFDOM_HEADERS_PREFIX_INCLUDE} 
  ${URDFDOM_HEADERS_PREFIX_INCLUDE}/urdfdom_headers)

target_link_libraries(humanoid_common_mpc PUBLIC
  robot::model
  PkgConfig::pinocchio
  Boost::boost
  Eigen3::Eigen
  ${urdfdom_LIBRARIES})

add_library(humanoid::common_mpc ALIAS humanoid_common_mpc)

