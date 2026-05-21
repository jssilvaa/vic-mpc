# SDL2 (brewed). Brew's sdl2-config.cmake exports an SDL2::SDL2 target.
find_package(SDL2 REQUIRED CONFIG)

add_library(remote_control STATIC
  src/XboxControllerInterface.cpp)

target_include_directories(remote_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

# Older SDL2 config files use SDL2::SDL2; newer split also provides SDL2::SDL2-static.
if(TARGET SDL2::SDL2)
  target_link_libraries(remote_control PUBLIC SDL2::SDL2)
else()
  target_include_directories(remote_control PUBLIC ${SDL2_INCLUDE_DIRS})
  target_link_libraries(remote_control PUBLIC ${SDL2_LIBRARIES})
endif()
