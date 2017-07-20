cmake_minimum_required(VERSION 3.5)

# Package name goes here; should match directory name
project(minimal_nodes)

# Packages from package.xml.  Always need ament_cmake.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(std_msgs REQUIRED)

## Boilerplate
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
## end boilerplate

# define function named `custom_executable` to call for each executable ("target")
function(custom_executable target)
  # Create new executable `foo` from src/foo.cpp
  add_executable(${target} src/${target}.cpp)
  # Tell ament that foo needs rclcpp, std_msgs.
  # Easiest to list all packages here.
  ament_target_dependencies(${target}
    "rclcpp"
    "std_msgs")
  # In general, "install" means "copy to installation directory";
  # here, building with `ament build -s` will create a symlink instead.
  # This saves disk space and saves the need to rebuild when editing scripts
  install(TARGETS ${target}
    DESTINATION lib/${PROJECT_NAME})
  # Note that the lib/${PROJECT_NAME} is needed to interact properly with `ros2 run`
endfunction()

## how to extend the above function for special cases
# custom_executable(imu_listener)
# ament_target_dependencies(imu_listener "sensor_msgs")
## make sure to find_package(sensor_msgs REQUIRED) first

# Call the above function for each source file.  We could have copied
# out the contents of the function each time, but that is harder to
# maintain and not worth the effort here, where each source file is a
# separate executable.
custom_executable(minimal_publisher)
custom_executable(sleepy_minimal_publisher)
custom_executable(minimal_subscriber)
custom_executable(minimal_subscriber_lambda)
custom_executable(minimal_simulator)
custom_executable(minimal_controller)

# Run ament end stuff (needed, should be last)
ament_package()
