cmake_minimum_required(VERSION 3.5)

project(odom_tf)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(xform_utils REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra -Wpedantic")
endif()

# https://stackoverflow.com/a/12258855
set(EIGEN3_INCLUDE_DIR "$ENV{EIGEN3_INCLUDE_DIR}")
if(NOT EIGEN3_INCLUDE_DIR)
    message(FATAL_ERROR "Please point the environment variable EIGEN3_INCLUDE_DIR to the include directory of your Eigen3 installation.")
endif()
include_directories("${EIGEN3_INCLUDE_DIR}")

include_directories(include)
ament_export_include_directories(include)

add_library(${PROJECT_NAME} SHARED src/OdomTf.cpp)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp std_msgs nav_msgs geometry_msgs tf2 tf2_ros tf2_msgs xform_utils
  )
ament_export_libraries(${PROJECT_NAME})

set(target odom_tf_demo)

add_executable(${target} src/odom_tf_main.cpp)
target_link_libraries(${target} ${PROJECT_NAME})
ament_target_dependencies(${target} "rclcpp" "std_msgs" "nav_msgs" "geometry_msgs" "tf2" "tf2_ros" "tf2_msgs" "xform_utils" ${PROJECT_NAME})
install(TARGETS ${target} DESTINATION bin)

ament_package()

install(DIRECTORY include/
  DESTINATION include
)
