cmake_minimum_required(VERSION 3.5)

project(mobot_pub_des_state)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(traj_builder REQUIRED)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra -Wpedantic")
endif()

# https://stackoverflow.com/a/12258855
set(EIGEN3_INCLUDE_DIR "/usr/include/eigen3/")
include_directories("${EIGEN3_INCLUDE_DIR}")

function(custom_executable target)
  add_executable(${target} src/${target}.cpp)
  ament_target_dependencies(${target}
    "rclcpp" "std_msgs" "nav_msgs" "geometry_msgs" "traj_builder")
  install(TARGETS ${target}
    DESTINATION bin)
endfunction()

custom_executable(pub_des_state_main)
custom_executable(pub_des_state_startup_main)
custom_executable(open_loop_controller)
custom_executable(pub_des_state_path_client)
custom_executable(pub_des_state_path_client_3mx3m_sqr)
# custom_executable(pub_des_state_path_client_10mx10m_sqr)
custom_executable(starting_pen_pub_des_state_path_client)

ament_package()

install(DIRECTORY include/
  DESTINATION include
)
