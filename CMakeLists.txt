cmake_minimum_required(VERSION 3.0.0)
get_filename_component(PROJECT ${CMAKE_SOURCE_DIR} NAME)
project(${PROJECT})

# Link with the Webots controller library.
set(WEBOTS_HOME "/usr/local/webots")
link_directories(${WEBOTS_HOME}/lib/controller)
set(WEBOTSLIBRARIES m ${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX} ${CMAKE_SHARED_LIBRARY_PREFIX}CppController${CMAKE_SHARED_LIBRARY_SUFFIX})
include_directories(${WEBOTS_HOME}/include/controller/c ${WEBOTS_HOME}/include/controller/cpp)

# pinocchio
find_package(pinocchio REQUIRED)
find_package(qpOASES REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(${PINOCCHIO_INCLUDE_DIRS})
include_directories(${YAML_CPP_INCLUDE_DIR})

# Get C++ sources
file(GLOB_RECURSE SRC_LIST 
    ./src/*.cpp
    ./src/model/*.cpp
)
# Get C++ head files
include_directories(include/common include/model include)

# Setup the target executable.
add_executable(${PROJECT} ${SRC_LIST})
target_link_libraries(${PROJECT} ${WEBOTSLIBRARIES})
target_link_libraries(${PROJECT} pinocchio::pinocchio)
target_link_libraries(${PROJECT} qpOASES)
target_link_libraries(${PROJECT} yaml-cpp::yaml-cpp)


# Resolve for the package path at compile time.
configure_file (
  "${PROJECT_SOURCE_DIR}/include/common/projectPath.h.in"
  "${PROJECT_SOURCE_DIR}/include/common/projectPath.h" @ONLY
)