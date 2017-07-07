cmake_minimum_required(VERSION 3.5)

project(baxter_core_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AnalogIOState.msg"
  "msg/AnalogIOStates.msg"
  "msg/AnalogOutputCommand.msg"
  "msg/AssemblyState.msg"
  "msg/AssemblyStates.msg"
  "msg/CameraControl.msg"
  "msg/CameraSettings.msg"
  "msg/CollisionAvoidanceState.msg"
  "msg/CollisionDetectionState.msg"
  "msg/DigitalIOState.msg"
  "msg/DigitalIOStates.msg"
  "msg/DigitalOutputCommand.msg"
  "msg/EndEffectorCommand.msg"
  "msg/EndEffectorProperties.msg"
  "msg/EndEffectorState.msg"
  "msg/EndpointState.msg"
  "msg/EndpointStates.msg"
  "msg/HeadPanCommand.msg"
  "msg/HeadState.msg"
  "msg/JointCommand.msg"
  "msg/NavigatorState.msg"
  "msg/NavigatorStates.msg"
  "msg/RobustControllerStatus.msg"
  "msg/SEAJointState.msg"
  "msg/URDFConfiguration.msg"
  "srv/CloseCamera.srv"
  "srv/ListCameras.srv"
  "srv/OpenCamera.srv"
  "srv/SolvePositionIK.srv"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs builtin_interfaces
)

ament_package()
