#include <creating_a_ros_library/example_ros_class.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::node::Node::make_shared("example_lib_test_main");

  ROS_INFO("main: instantiating an object of type ExampleRosClass");
  ExampleRosClass exampleRosClass(node);

  ROS_INFO("main: going into spin; let the callbacks do all the work");
  rclcpp::spin(node);
  return 0;
}