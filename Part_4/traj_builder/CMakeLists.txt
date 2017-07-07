cmake_minimum_required(VERSION 3.5)

project(traj_builder)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


include_directories(include)
ament_export_include_directories(include)

add_library(${PROJECT_NAME} SHARED src/traj_builder.cpp)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp std_msgs nav_msgs geometry_msgs
  )
# ament_export_libraries(${PROJECT_NAME})
target_compile_definitions(${PROJECT_NAME}
  PRIVATE "TRAJ_BUILDER_BUILDING_LIBRARY")
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_libraries(${PROJECT_NAME})

set(target traj_builder_example_main)

add_executable(${target} src/${target}.cpp)
target_link_libraries(${target} ${PROJECT_NAME})
ament_target_dependencies(${target} "rclcpp" "rcutils" "std_msgs" "nav_msgs" "geometry_msgs" ${PROJECT_NAME})
install(TARGETS ${target} DESTINATION bin)

ament_package()

install(DIRECTORY include/
  DESTINATION include
)
