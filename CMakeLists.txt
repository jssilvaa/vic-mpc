cmake_minimum_required(VERSION 3.20)
project(vic_mpc CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ----------------------------------------------------------------------------
# MuJoCo
#
# Two ways to point CMake at MuJoCo:
#   1) Install the official prebuilt release into ~/mujoco-3.x.x and pass
#        -DMUJOCO_DIR=$HOME/mujoco-3.x.x
#      to cmake. This is the fastest path on macOS.
#   2) Let FetchContent build it from source (slow first build, ~5 min).
#      Run cmake with -DFETCH_MUJOCO=ON.
# ----------------------------------------------------------------------------
option(FETCH_MUJOCO "Build MuJoCo from source via FetchContent" OFF)

if(FETCH_MUJOCO)
  include(FetchContent)
  FetchContent_Declare(
    mujoco
    GIT_REPOSITORY https://github.com/google-deepmind/mujoco.git
    GIT_TAG        3.8.0
  )
  set(MUJOCO_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  # ON: build libmujoco_simulate so we can embed the official viewer UI.
  set(MUJOCO_BUILD_SIMULATE ON  CACHE BOOL "" FORCE)
  set(MUJOCO_BUILD_TESTS    OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(mujoco)
else()
  if(NOT DEFINED MUJOCO_DIR)
    message(FATAL_ERROR
      "Set -DMUJOCO_DIR=/path/to/mujoco (the unpacked release) "
      "or pass -DFETCH_MUJOCO=ON to build from source.")
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
  add_library(mujoco ALIAS mujoco::mujoco)
endif()

# GLFW for the viewer example. On macOS: brew install glfw
# When FETCH_MUJOCO is ON the simulate sub-build will fetch its own GLFW too,
# so the local find_package is only consulted in the prebuilt path.
find_package(glfw3 QUIET)
find_package(Threads REQUIRED)

# ----------------------------------------------------------------------------
# Examples
# ----------------------------------------------------------------------------
# Bake the absolute models dir into the binaries so the default path works
# regardless of which directory the user launches them from.
set(DEFAULT_MODEL "${CMAKE_SOURCE_DIR}/models/cartpole.xml")

add_executable(01_load_step src/01_load_step.cpp)
target_link_libraries(01_load_step PRIVATE mujoco::mujoco)
target_compile_definitions(01_load_step PRIVATE DEFAULT_MODEL="${DEFAULT_MODEL}")

add_executable(03_pd_control src/03_pd_control.cpp)
target_link_libraries(03_pd_control PRIVATE mujoco::mujoco)
target_compile_definitions(03_pd_control PRIVATE DEFAULT_MODEL="${DEFAULT_MODEL}")

# 02_viewer embeds MuJoCo's official `simulate` viewer (passive mode), so it
# needs libmujoco_simulate. That target only exists in the FETCH_MUJOCO build,
# since the prebuilt release ships it as a separate component we don't wire up.
if(TARGET mujoco::libmujoco_simulate)
  add_executable(02_viewer src/02_viewer.cpp)
  target_link_libraries(02_viewer PRIVATE
    mujoco::mujoco
    mujoco::libmujoco_simulate
    glfw
    Threads::Threads)
  target_compile_definitions(02_viewer PRIVATE DEFAULT_MODEL="${DEFAULT_MODEL}")
  if(APPLE)
    # CoreVideo is needed by simulate's GlfwCoreVideo; the framework is a
    # PUBLIC link of platform_ui_adapter (an OBJECT lib) so it doesn't
    # propagate transitively through libmujoco_simulate. Add it directly.
    target_link_libraries(02_viewer PRIVATE
      "-framework OpenGL" "-framework Cocoa" "-framework IOKit"
      "-framework CoreVideo")
  endif()
else()
  message(STATUS "libmujoco_simulate not available — skipping 02_viewer "
                 "(rebuild with -DFETCH_MUJOCO=ON)")
endif()
