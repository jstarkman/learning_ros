cmake_minimum_required(VERSION 3.5)
project(localization_w_gps)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(rmw REQUIRED)

find_package(std_msgs)
find_package(nav_msgs)
find_package(geometry_msgs)
find_package(xform_utils)
find_package(sensor_msgs)

# https://stackoverflow.com/a/12258855
set(EIGEN3_INCLUDE_DIR "$ENV{EIGEN3_INCLUDE_DIR}")
if(NOT EIGEN3_INCLUDE_DIR)
    message(FATAL_ERROR "Please point the environment variable EIGEN3_INCLUDE_DIR to the include directory of your Eigen3 installation.")
endif()
include_directories("${EIGEN3_INCLUDE_DIR}")

set(target localization_w_gps)

add_executable(${target} src/${target}.cpp)
ament_target_dependencies(${target} "rclcpp" "rcutils" "std_msgs" "nav_msgs" "geometry_msgs" "xform_utils" "sensor_msgs")
install(TARGETS ${target} DESTINATION lib/${PROJECT_NAME})

ament_package()
