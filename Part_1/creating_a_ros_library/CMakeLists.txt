cmake_minimum_required(VERSION 3.5)
project(creating_a_ros_library)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(rmw REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(include)
ament_export_include_directories(include)

add_library(${PROJECT_NAME} SHARED src/example_ros_class.cpp)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp std_msgs std_srvs class_loader)
target_compile_definitions(${PROJECT_NAME}
  PRIVATE "CREATING_A_ROS_LIBRARY_BUILDING_LIBRARY")
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
ament_export_libraries(${PROJECT_NAME})

function(add_link_install_exe target link_to_whom files)
  add_executable(${target} ${files})
  target_link_libraries(${target} ${link_to_whom})
  ament_target_dependencies(${target}
    "rclcpp" "rcutils" "std_msgs" "std_srvs" ${link_to_whom})
  install(TARGETS ${target} DESTINATION lib/${PROJECT_NAME})
endfunction()

add_link_install_exe(ros_library_test_main ${PROJECT_NAME}
  src/example_ros_class_test_main.cpp)

ament_package()

install(DIRECTORY include/
  DESTINATION include
)
