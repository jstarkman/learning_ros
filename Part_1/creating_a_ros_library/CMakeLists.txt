cmake_minimum_required(VERSION 2.8.3)
project(creating_a_ros_library)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(include)


function(add_install_dll target files)
  add_library(${target} SHARED ${files})
  ament_target_dependencies(${target}
    rclcpp std_msgs std_srvs class_loader)
  install(TARGETS ${target} DESTINATION lib)
endfunction()


function(add_link_install_exe target link_to_whom files)
  add_executable(${target} ${files})
  target_link_libraries(${target} ${link_to_whom})
  ament_target_dependencies(${target}
    "rclcpp" "std_msgs" "std_srvs" ${link_to_whom})
  install(TARGETS ${target} DESTINATION bin)
endfunction()


set(shlib example_ros_library)
add_install_dll(${shlib} src/example_ros_class.cpp)
add_link_install_exe(ros_library_test_main ${shlib}
  src/example_ros_class_test_main.cpp)

ament_package()