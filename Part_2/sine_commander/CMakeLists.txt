cmake_minimum_required(VERSION 3.5)

project(sine_commander)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra -Wpedantic")
endif()

function(custom_executable target)
  add_executable(${target} src/${target}.cpp)
  ament_target_dependencies(${target}
    "rclcpp" "std_msgs" "sensor_msgs")
  install(TARGETS ${target}
    DESTINATION bin)
endfunction()

custom_executable(sine_commander)
custom_executable(test_sine_commander)

ament_package()
