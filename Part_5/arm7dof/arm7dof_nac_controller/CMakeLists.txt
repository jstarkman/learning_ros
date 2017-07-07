cmake_minimum_required(VERSION 3.5)

project(arm7dof_nac_controller)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(arm7dof_fk_ik REQUIRED)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# https://stackoverflow.com/a/12258855
set(EIGEN3_INCLUDE_DIR "$ENV{EIGEN3_INCLUDE_DIR}")
if(NOT EIGEN3_INCLUDE_DIR)
    message(FATAL_ERROR "Please point the environment variable EIGEN3_INCLUDE_DIR to the include directory of your Eigen3 installation.")
endif()
include_directories("${EIGEN3_INCLUDE_DIR}")

function(custom_executable target)
  add_executable(${target} src/${target}.cpp)
  ament_target_dependencies(${target} "rclcpp" "std_msgs" "sensor_msgs" "geometry_msgs" "arm7dof_fk_ik" ${PROJECT_NAME})
  install(TARGETS ${target} DESTINATION lib/${PROJECT_NAME})
endfunction()

custom_executable(arm7dof_nac_controller)

ament_package()
