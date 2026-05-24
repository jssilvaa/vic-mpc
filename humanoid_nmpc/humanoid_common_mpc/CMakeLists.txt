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
  src/pinocchio_model/createPinocchioModel.cpp
  src/pinocchio_model/DynamicsHelperFunctions.cpp
  src/pinocchio_model/pinocchioUtils.cpp
  src/common/ModelSettings.cpp
  src/gait/Gait.cpp
  src/gait/ModeSequenceTemplate.cpp
  src/gait/GaitSchedule.cpp
  src/swing_foot_planner/CubicSpline.cpp
  src/swing_foot_planner/SplineCpg.cpp
  src/swing_foot_planner/SwingTrajectoryPlanner.cpp
  src/contact/ContactCenterPoint.cpp
  src/contact/ContactPolygon.cpp
  src/contact/ContactRectangle.cpp
)

target_include_directories(humanoid_common_mpc PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${URDFDOM_PREFIX_INCLUDE}
  ${URDFDOM_PREFIX_INCLUDE}/urdfdom
  ${URDFDOM_HEADERS_PREFIX_INCLUDE} 
  ${URDFDOM_HEADERS_PREFIX_INCLUDE}/urdfdom_headers)

target_link_libraries(humanoid_common_mpc PUBLIC
  ocs2::core
  ocs2::robotic_tools
  ocs2::pinocchio_interface
  robot::model
  PkgConfig::pinocchio
  Boost::boost
  Eigen3::Eigen
  ${urdfdom_LIBRARIES})

add_library(humanoid::common_mpc ALIAS humanoid_common_mpc)

# D2 behavior check: config -> joint partition -> gait schedule.
add_executable(gaitScheduleDemo exe/gaitScheduleDemo.cpp)
target_link_libraries(gaitScheduleDemo PRIVATE humanoid::common_mpc)
set_target_properties(gaitScheduleDemo PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# D2 behavior check: swing-foot planner over a hand-built walking gait.
add_executable(swingPlannerDemo exe/swingPlannerDemo.cpp)
target_link_libraries(swingPlannerDemo PRIVATE humanoid::common_mpc)
set_target_properties(swingPlannerDemo PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

