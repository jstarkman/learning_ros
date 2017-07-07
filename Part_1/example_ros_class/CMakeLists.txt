cmake_minimum_required(VERSION 3.5)
project(example_ros_class)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(rmw REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)


function(add_install_exe target files)
  add_executable(${target} ${files})
  ament_target_dependencies(${target}
    "rclcpp" "rcutils" "std_msgs" "std_srvs")
  install(TARGETS ${target} DESTINATION bin)
endfunction()


add_install_exe(example_ros_class src/example_ros_class.cpp)

ament_package()