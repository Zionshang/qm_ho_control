cmake_minimum_required(VERSION 3.10.0)
project(qm_ho_control)

# pinocchio
find_package(pinocchio REQUIRED)
find_package(qpOASES REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(custom_msgs REQUIRED)

include_directories(
  include
  ${PINOCCHIO_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
  # ${rclcpp_INCLUDE_DIRS}
)

file(GLOB_RECURSE SRC_LIST
  ./src/*.cpp
  ./src/model/*.cpp
)

add_executable(qm_ho_control ${SRC_LIST})

target_link_libraries(qm_ho_control
  pinocchio::pinocchio
  qpOASES
  yaml-cpp::yaml-cpp
)

ament_target_dependencies(qm_ho_control
  rclcpp
  sensor_msgs
  custom_msgs
)

install(TARGETS
  qm_ho_control
  DESTINATION lib/${PROJECT_NAME}
)

configure_file(
  "${PROJECT_SOURCE_DIR}/include/utils/project_path.hpp.in"
  "${PROJECT_SOURCE_DIR}/include/utils/project_path.hpp" @ONLY
)

ament_package()
