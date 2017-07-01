cmake_minimum_required(VERSION 3.5)
project(lin_steering)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)

find_package(std_msgs)
find_package(nav_msgs)
find_package(geometry_msgs)

function(custom_executable target)
  add_executable(${target} src/${target}.cpp)
  ament_target_dependencies(${target}
    "rclcpp" "std_msgs")
  install(TARGETS ${target}
    DESTINATION bin)
endfunction()

custom_executable(lin_steering_wrt_odom)
custom_executable(lin_steering_wrt_amcl)

ament_package()
