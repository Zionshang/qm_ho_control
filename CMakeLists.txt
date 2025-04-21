cmake_minimum_required(VERSION 3.0.0)
project(qm_ho_control)

# Link with the Webots controller library.
set(WEBOTS_HOME "/usr/local/webots")
link_directories(${WEBOTS_HOME}/lib/controller)
set(WEBOTSLIBRARIES m ${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX} ${CMAKE_SHARED_LIBRARY_PREFIX}CppController${CMAKE_SHARED_LIBRARY_SUFFIX})
include_directories(${WEBOTS_HOME}/include/controller/c ${WEBOTS_HOME}/include/controller/cpp)

# pinocchio
find_package(pinocchio REQUIRED)
find_package(qpOASES REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(aligator REQUIRED)

include_directories(${PINOCCHIO_INCLUDE_DIRS})
include_directories(${YAML_CPP_INCLUDE_DIR})

# Get C++ sources
file(GLOB_RECURSE SRC_LIST 
    ./src/*.cpp
    ./src/model/*.cpp
)
# Get C++ head files
include_directories(include)

# Setup the target executable.
add_executable(${PROJECT_NAME} ${SRC_LIST})
target_link_libraries(${PROJECT_NAME} ${WEBOTSLIBRARIES})
target_link_libraries(${PROJECT_NAME} pinocchio::pinocchio)
target_link_libraries(${PROJECT_NAME} qpOASES)
target_link_libraries(${PROJECT_NAME} yaml-cpp::yaml-cpp)
target_link_libraries(${PROJECT_NAME} aligator::aligator)


# Resolve for the package path at compile time.
configure_file (
  "${PROJECT_SOURCE_DIR}/include/utils/project_path.hpp.in"
  "${PROJECT_SOURCE_DIR}/include/utils/project_path.hpp" @ONLY
)