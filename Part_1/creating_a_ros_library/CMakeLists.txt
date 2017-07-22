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

# Include own include/ directory and export under ros2_ws/install/
include_directories(include)
ament_export_include_directories(include)

# Add new shared library (libfoo.so) named libcreating_a_ros_library.so
add_library(${PROJECT_NAME} SHARED src/example_ros_class.cpp)
# Library depends on these other packages
ament_target_dependencies(${PROJECT_NAME}
  rclcpp std_msgs std_srvs class_loader)
# Makes DLL visible on Windows
target_compile_definitions(${PROJECT_NAME}
  PRIVATE "CREATING_A_ROS_LIBRARY_BUILDING_LIBRARY")
# Copy under install/lib/
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
# Tell ament to effect the copy
ament_export_libraries(${PROJECT_NAME})

function(add_link_install_exe target files)
  add_executable(${target} ${files})
  # Link against own library
  target_link_libraries(${target} ${PROJECT_NAME})
  # Depend on self
  ament_target_dependencies(${target}
    "rclcpp" "rcutils" "std_msgs" "std_srvs" ${PROJECT_NAME})
  install(TARGETS ${target} DESTINATION lib/${PROJECT_NAME})
endfunction()

add_link_install_exe(ros_library_test_main src/example_ros_class_test_main.cpp)

ament_package()

# Ensure that header files are copied/symlinked where they should be
install(DIRECTORY include/
  DESTINATION include
)
