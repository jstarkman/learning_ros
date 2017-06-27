cmake_minimum_required(VERSION 3.5)

project(traj_builder)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra -Wpedantic")
endif()

include_directories(include)
ament_export_include_directories(include)

add_library(${PROJECT_NAME} SHARED src/traj_builder.cpp)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp std_msgs nav_msgs geometry_msgs
  )
ament_export_libraries(${PROJECT_NAME})

set(target traj_builder_example_main)

add_executable(${target} src/${target}.cpp)
target_link_libraries(${target} ${PROJECT_NAME})
ament_target_dependencies(${target} "rclcpp" "std_msgs" "nav_msgs" "geometry_msgs" ${PROJECT_NAME})
install(TARGETS ${target} DESTINATION bin)

ament_package()

install(DIRECTORY include/
  DESTINATION include
)
