#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "baxter_core_msgs/msg/head_pan_command.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::node::Node::make_shared("baxter_head_pan_zero");
  auto head_pan_pub = node->create_publisher<baxter_core_msgs::msg::HeadPanCommand>("/robot/head/command_head_pan",
                                                                                    rmw_qos_profile_default);

  auto headPanCommand = std::make_shared<baxter_core_msgs::msg::HeadPanCommand>();
  headPanCommand->target = 0.0;
  headPanCommand->enable_pan_request = 1;

  rclcpp::WallRate timer(4);
  for (int i = 0; i < 4; i++)
  {
    head_pan_pub->publish(headPanCommand);
    timer.sleep();
    rclcpp::spin_some(node);
  }
}
