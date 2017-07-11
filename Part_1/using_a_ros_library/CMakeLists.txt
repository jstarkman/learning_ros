cmake_minimum_required(VERSION 3.5)
project(using_a_ros_library)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(creating_a_ros_library REQUIRED)

find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

function(add_link_install_exe target files)
  add_executable(${target} ${files})
  ament_target_dependencies(${target}
    "rclcpp" "creating_a_ros_library" "std_msgs" "std_srvs")
  install(TARGETS ${target} DESTINATION bin)
endfunction()

add_link_install_exe(example_ros_class_test_main
                     src/example_ros_class_test_main.cpp)

ament_package()
