cmake_minimum_required(VERSION 3.5)
project(simple_baxter_gripper_interface)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(rmw REQUIRED)
find_package(baxter_core_msgs REQUIRED)

include_directories(include)
ament_export_include_directories(include)

add_library(${PROJECT_NAME} SHARED src/${PROJECT_NAME}.cpp)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp baxter_core_msgs)
target_compile_definitions(${PROJECT_NAME}
  PRIVATE "simple_baxter_gripper_interface_BUILDING_LIBRARY")
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_libraries(${PROJECT_NAME})

function(custom_executable target)
  add_executable(${target} src/${target}.cpp)
  target_link_libraries(${target} ${PROJECT_NAME})
  ament_target_dependencies(${target}
    "rclcpp" "rcutils" "baxter_core_msgs" ${PROJECT_NAME})
  install(TARGETS ${target} DESTINATION bin)  
endfunction()

custom_executable(test_baxter_gripper)
custom_executable(baxter_gripper_lib_test_main)

ament_package()

install(DIRECTORY include/
  DESTINATION include
)
