cmake_minimum_required(VERSION 3.5)
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
ament_export_include_directories(include)

add_library(${PROJECT_NAME} SHARED src/example_ros_class.cpp)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp std_msgs std_srvs class_loader)
ament_export_libraries(${PROJECT_NAME})
# install(TARGETS ${PROJECT_NAME} DESTINATION lib)


function(add_link_install_exe target link_to_whom files)
  add_executable(${target} ${files})
  target_link_libraries(${target} ${link_to_whom})
  ament_target_dependencies(${target}
    "rclcpp" "std_msgs" "std_srvs" ${link_to_whom})
  install(TARGETS ${target} DESTINATION bin)
endfunction()

add_link_install_exe(ros_library_test_main ${PROJECT_NAME}
  src/example_ros_class_test_main.cpp)

ament_package()

install(DIRECTORY include/
  DESTINATION include
)
