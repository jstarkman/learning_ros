#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float64.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("minimal_subscriber");

  auto sub = node->create_subscription<std_msgs::msg::Float64>(
      "topic1",
      [](const std_msgs::msg::Float64::SharedPtr msg) { std::cout << "received value is: " << msg->data << std::endl; },
      rmw_qos_profile_default);

  rclcpp::spin(node);

  return 0;
}
