find_package(glfw3 REQUIRED)
find_package(Threads REQUIRED)

# GLEW is only needed on non-Apple platforms (macOS uses the system OpenGL framework).
if(NOT APPLE)
  find_package(GLEW REQUIRED)
endif()

add_library(mujoco_sim_interface STATIC
  src/MujocoSimInterface.cpp
  src/MujocoRenderer.cpp)

target_include_directories(mujoco_sim_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_link_libraries(mujoco_sim_interface PUBLIC
  robot::model
  robot::core
  mujoco::mujoco
  glfw
  Threads::Threads)

if(APPLE)
  target_link_libraries(mujoco_sim_interface PUBLIC
    "-framework OpenGL" "-framework Cocoa" "-framework IOKit" "-framework CoreVideo")
else()
  target_link_libraries(mujoco_sim_interface PUBLIC GLEW::GLEW)
endif()

add_library(robot::mujoco_sim_interface ALIAS mujoco_sim_interface)

# D0 deliverable executable
add_executable(mujocoSimPDStand exe/mujocoSimPDStand.cpp)
target_link_libraries(mujocoSimPDStand PRIVATE
  robot::mujoco_sim_interface
  remote_control)
set_target_properties(mujocoSimPDStand PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
