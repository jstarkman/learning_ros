#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float64.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::node::Node::make_shared("minimal_publisher");
  auto pub = node->create_publisher<std_msgs::msg::Float64>(
      "topic1", rmw_qos_profile_default);

  auto msg = std::make_shared<std_msgs::msg::Float64>();
  double count = 0.0;

  while (rclcpp::ok()) {
    msg->data = count;
    count += 0.001;
    printf("Publishing: %f\n", msg->data);
    pub->publish(msg);
    rclcpp::spin_some(node);
  }

  return 0;
}
