#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float64.hpp"

/**
 * Callback run when a new Float64 comes in.  This is the function
 * referred to by the `create_subscription` line.  Simple callbacks
 * like this one should be implemented as lambda expressions (see
 * minimal_subscriber_lambda.cpp for details).
 */
void cbTopic1(const std_msgs::msg::Float64::SharedPtr msg)
{
  printf("received value is: %f\n", msg->data);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("minimal_subscriber");

  auto sub = node->create_subscription<std_msgs::msg::Float64>("topic1", cbTopic1, rmw_qos_profile_default);

  rclcpp::spin(node);

  return 0;
}
