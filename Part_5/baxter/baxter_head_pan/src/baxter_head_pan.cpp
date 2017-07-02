#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "baxter_core_msgs/msg/head_pan_command.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::node::Node::make_shared("baxter_head_pan");
  auto head_pan_pub = node->create_publisher<baxter_core_msgs::msg::HeadPanCommand>("/robot/head/command_head_pan",
                                                                                    rmw_qos_profile_default);

  auto headPanCommand = std::make_shared<baxter_core_msgs::msg::HeadPanCommand>();
  headPanCommand->target = 0.0;
  headPanCommand->enable_pan_request = 1;

  double amp, freq;
  std::cout << "enter pan amplitude (rad): ";
  std::cin >> amp;
  std::cout << "enter pan freq (Hz): ";
  std::cin >> freq;
  double dt = 0.01;
  rclcpp::WallRate timer(1 / dt);
  double phase = 0.0;
  double theta;

  while (rclcpp::ok())
  {
    phase += M_PI * 2.0 * freq * dt;
    if (phase > 2.0 * M_PI)
      phase -= 2.0 * M_PI;
    theta = amp * sin(phase);
    headPanCommand->target = theta;
    head_pan_pub->publish(headPanCommand);
    timer.sleep();
    rclcpp::spin_some(node);
  }
}
