cmake_minimum_required(VERSION 3.5)
project(mobot_nl_steering)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)

find_package(std_msgs)
find_package(nav_msgs)
find_package(geometry_msgs)

set(target mobot_nl_steering)

add_executable(${target} src/${target}.cpp)
ament_target_dependencies(${target} "rclcpp" "std_msgs" "nav_msgs" "geometry_msgs")
install(TARGETS ${target} DESTINATION bin)

ament_package()
