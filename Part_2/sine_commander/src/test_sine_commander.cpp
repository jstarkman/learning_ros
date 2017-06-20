#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float64.hpp"

void cbTopic1(const std_msgs::msg::Float64::SharedPtr msg)
{
  printf("received value is: %f\n", msg->data);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("minimal_subscriber");

  auto sub = node->create_subscription<std_msgs::msg::Float64>("/one_DOF_robot/joint1_velocity_controller/command",
                                                               cbTopic1, rmw_qos_profile_default);

  rclcpp::spin(node);

  return 0;
}
