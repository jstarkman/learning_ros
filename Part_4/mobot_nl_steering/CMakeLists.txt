cmake_minimum_required(VERSION 3.5)
project(mobot_nl_steering)

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

set(target mobot_nl_steering)

add_executable(${target} src/${target}.cpp)
ament_target_dependencies(${target} "rclcpp" "rcutils" "std_msgs" "nav_msgs" "geometry_msgs")
install(TARGETS ${target} DESTINATION bin)

ament_package()
