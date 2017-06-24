#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float64.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::node::Node::make_shared("minimal_publisher2");
  auto pub = node->create_publisher<std_msgs::msg::Float64>("vel_cmd", rmw_qos_profile_default);

  auto input_float = std::make_shared<std_msgs::msg::Float64>();
  input_float->data = 0.0;

  rclcpp::WallRate naptime(1.0);
  while (rclcpp::ok())
  {
    input_float->data = input_float->data + 0.001;
    pub->publish(input_float);
    naptime.sleep();
  }
}
