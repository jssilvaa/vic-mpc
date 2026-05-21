# urdfdom (brewed) is config-file based but the brew install does not expose its
# config via CMAKE_PREFIX_PATH by default. Hint it explicitly.
find_package(urdfdom CONFIG REQUIRED
  HINTS /opt/homebrew/Cellar/urdfdom/6.0.0/lib/urdfdom/cmake
        /opt/homebrew/opt/urdfdom/lib/urdfdom/cmake)
find_package(urdfdom_headers CONFIG REQUIRED
  HINTS /opt/homebrew/Cellar/urdfdom_headers/3.0.0/lib/urdfdom_headers/cmake
        /opt/homebrew/opt/urdfdom_headers/lib/urdfdom_headers/cmake)

add_library(robot_model STATIC
  src/RobotDescription.cpp
  src/RobotState.cpp)

# urdfdom's brew layout puts the parser headers under <prefix>/include/urdfdom/
# but the cmake config exports include dir as <prefix>/include/urdfdom (so
# the in-tree headers include each other as <urdf_parser/...>). The reference
# impl includes them with the package prefix as <urdfdom/urdf_parser/...>, so
# we also need <prefix>/include on the path. Same story for urdfdom_headers.
# Derive both paths from the *_DIR cache variables.
set(URDFDOM_PREFIX_INCLUDE "${urdfdom_DIR}/../../../include")
set(URDFDOM_HEADERS_PREFIX_INCLUDE "${urdfdom_headers_DIR}/../../../include")

target_include_directories(robot_model PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${URDFDOM_PREFIX_INCLUDE}
  ${URDFDOM_PREFIX_INCLUDE}/urdfdom
  ${URDFDOM_HEADERS_PREFIX_INCLUDE}
  ${URDFDOM_HEADERS_PREFIX_INCLUDE}/urdfdom_headers)

target_link_libraries(robot_model PUBLIC
  robot::core
  ${urdfdom_LIBRARIES})

add_library(robot::model ALIAS robot_model)
