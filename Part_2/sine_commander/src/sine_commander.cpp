#include <math.h>
#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float64.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::node::Node::make_shared("sine_commander");
  auto cmd_publisher = node->create_publisher<std_msgs::msg::Float64>(
      "/one_DOF_robot/joint1_velocity_controller/command", rmw_qos_profile_default);
  auto v_cmd_float64 = std::make_shared<std_msgs::msg::Float64>();

  double v_cmd = 0.0;
  double x_cmd = 0.0;
  double x_amp = 0.0;
  double freq, omega;
  std::cout << "enter displacement amplitude: ";
  std::cin >> x_amp;
  std::cout << "enter freq (in Hz): ";
  std::cin >> freq;
  omega = freq * 2.0 * M_PI;
  double phase = 0;
  double dt = 0.01;

  rclcpp::WallRate sample_rate(1 / dt);

  while (rclcpp::ok())
  {
    phase += omega * dt;
    if (phase > 2.0 * M_PI)
    {
      phase -= 2.0 * M_PI;
    }
    x_cmd = x_amp * sin(phase);
    v_cmd = omega * x_amp * cos(phase);
    v_cmd_float64->data = v_cmd;
    cmd_publisher->publish(v_cmd_float64);
    sample_rate.sleep();
    rclcpp::spin_some(node);
  }
  return 0;
}
