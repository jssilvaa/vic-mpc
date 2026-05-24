cmake_minimum_required(VERSION 3.20)
project(vic_mpc CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Default RPATH so binaries find FetchContent'd MuJoCo at runtime without
# the user setting DYLD_LIBRARY_PATH.
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH ON)
if(APPLE)
  set(CMAKE_MACOSX_RPATH ON)
  set(CMAKE_BUILD_RPATH_USE_ORIGIN ON)
  list(APPEND CMAKE_INSTALL_RPATH "@loader_path/../lib")
endif()

# ----------------------------------------------------------------------------
# Eigen3: when FETCH_MUJOCO=ON, MuJoCo's CMake fetches Eigen3 itself and
# creates the Eigen3::Eigen imported target. Declaring it here too triggers
# a duplicate-target error, so we let MuJoCo own it in that case. When using
# a prebuilt MuJoCo, we have to find Eigen ourselves.
# ----------------------------------------------------------------------------

# ----------------------------------------------------------------------------
# MuJoCo
#   1) -DMUJOCO_DIR=$HOME/mujoco-3.x.x  uses a prebuilt release.
#   2) -DFETCH_MUJOCO=ON                builds from source (and provides
#                                       libmujoco_simulate for the viewer).
# Default for vic-mpc: FETCH_MUJOCO=ON because we want the simulate viewer.
# ----------------------------------------------------------------------------
option(FETCH_MUJOCO "Build MuJoCo from source via FetchContent" ON)

if(FETCH_MUJOCO)
  include(FetchContent)
  FetchContent_Declare(
    mujoco
    GIT_REPOSITORY https://github.com/google-deepmind/mujoco.git
    GIT_TAG        3.8.0
  )
  set(MUJOCO_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(MUJOCO_BUILD_SIMULATE ON  CACHE BOOL "" FORCE)
  set(MUJOCO_BUILD_TESTS    OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(mujoco)
else()
  if(NOT DEFINED MUJOCO_DIR)
    message(FATAL_ERROR
      "Set -DMUJOCO_DIR=/path/to/mujoco or pass -DFETCH_MUJOCO=ON.")
  endif()
  add_library(mujoco::mujoco SHARED IMPORTED)
  if(APPLE)
    set_target_properties(mujoco::mujoco PROPERTIES
      IMPORTED_LOCATION             "${MUJOCO_DIR}/lib/libmujoco.3.8.0.dylib"
      INTERFACE_INCLUDE_DIRECTORIES "${MUJOCO_DIR}/include")
  else()
    set_target_properties(mujoco::mujoco PROPERTIES
      IMPORTED_LOCATION             "${MUJOCO_DIR}/lib/libmujoco.so.3.8.0"
      INTERFACE_INCLUDE_DIRECTORIES "${MUJOCO_DIR}/include")
  endif()
endif()

# After MuJoCo loads, Eigen3::Eigen may already be defined (MuJoCo's
# FetchContent path creates it). If not (prebuilt MuJoCo, or MuJoCo's
# find_package found Eigen but didn't re-export the target), find it ourselves.
if(NOT TARGET Eigen3::Eigen)
  find_package(Eigen3 REQUIRED NO_MODULE)
endif()

# ----------------------------------------------------------------------------
# Subpackages (mirror examples/wb_humanoid_mpc layout).
# ----------------------------------------------------------------------------
add_subdirectory(robot_runtime/robot_core)
add_subdirectory(robot_runtime/robot_model)
add_subdirectory(humanoid_nmpc/remote_control)
# ocs2_core (transplanted substrate) before humanoid_common_mpc, which includes
# <ocs2_core/Types.h> and links ocs2::core.
# Transplanted OCS2 substrate (CppAD/ROS-free subset), grouped under lib/ to keep
# the top level our-code-only (mirrors the reference's lib/ocs2_ros2/ grouping).
add_subdirectory(lib/ocs2/ocs2_core)
add_subdirectory(lib/ocs2/ocs2_robotic_tools)
add_subdirectory(lib/ocs2/ocs2_pinocchio_interface)
add_subdirectory(lib/ocs2/ocs2_centroidal_model)
# humanoid_common_mpc before mujoco_sim_interface: the pinocchioCrossCheck exe in
# the latter links humanoid::common_mpc, so the alias must already exist.
add_subdirectory(humanoid_nmpc/humanoid_common_mpc)
add_subdirectory(humanoid_nmpc/humanoid_centroidal_mpc)
add_subdirectory(robot_runtime/mujoco_sim_interface)
