cmake_minimum_required(VERSION 3.5)

project(arm7dof_nac_controller)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(arm7dof_fk_ik REQUIRED)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra -Wpedantic")
endif()

# https://stackoverflow.com/a/12258855
set(EIGEN3_INCLUDE_DIR "/usr/include/eigen3/")
include_directories("${EIGEN3_INCLUDE_DIR}")

function(custom_executable target)
  add_executable(${target} src/${target}.cpp)
  ament_target_dependencies(${target} "rclcpp" "std_msgs" "sensor_msgs" "geometry_msgs" "arm7dof_fk_ik" ${PROJECT_NAME})
  install(TARGETS ${target} DESTINATION bin)
endfunction()

custom_executable(arm7dof_nac_controller)

ament_package()
