cmake_minimum_required(VERSION 3.5)

project(xform_utils)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra -Wpedantic")
endif()

# https://stackoverflow.com/a/12258855
set(EIGEN3_INCLUDE_DIR "/usr/include/eigen3/")
include_directories("${EIGEN3_INCLUDE_DIR}")

include_directories(include)
ament_export_include_directories(include)

add_library(${PROJECT_NAME} SHARED src/xform_utils.cpp)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp std_msgs nav_msgs geometry_msgs tf2)
# ament_export_libraries(${PROJECT_NAME})
target_compile_definitions(${PROJECT_NAME}
  PRIVATE "XFORM_UTILS_BUILDING_LIBRARY")
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

set(target example_xform_utils)

add_executable(${target} src/${target}.cpp)
target_link_libraries(${target} ${PROJECT_NAME})
ament_target_dependencies(${target} "rclcpp" "std_msgs" "nav_msgs" "geometry_msgs" "tf2" ${PROJECT_NAME})
install(TARGETS ${target} DESTINATION bin)

ament_package()

install(DIRECTORY include/
  DESTINATION include
)
