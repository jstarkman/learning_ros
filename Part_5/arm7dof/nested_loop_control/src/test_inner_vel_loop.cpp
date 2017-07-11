#define _USE_MATH_DEFINES
#include <math.h>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::node::Node::make_shared("test_inner_vel_loop");

  int jnum;
  double omega, amp;
  auto jnt_cmd_publisher =
      node->create_publisher<std_msgs::msg::Float64MultiArray>("qdes_attractor_vec", rmw_qos_profile_default);
  std_msgs::msg::Float64MultiArray qdes_msg;
  for (int i = 0; i < 7; i++)
  {
    qdes_msg.data.push_back(0.0);
  }
  std::cout << " node to command sinusoidal motions to individual joints to test inner_vel_loop controller"
            << std::endl;
  std::cout << " you will be prompted for joint number, amplitude and frequency" << std::endl;
  std::cout << "enter jnt num (0-6): ";
  std::cin >> jnum;
  if ((jnum > 6) || (jnum < 0))
  {
    std::cout << "wise guy!" << std::endl;
    return 1;
  }

  std::cout << "enter amplitude, rad: ";
  std::cin >> amp;

  std::cout << "enter omega, rad/sec: ";
  std::cin >> omega;

  double dt = 0.01;
  double phase = 0.0;
  while (rclcpp::ok())
  {
    phase += omega * dt;
    if (phase > 2.0 * M_PI)
    {
      phase -= 2.0 * M_PI;
    }
    qdes_msg.data[jnum] = amp * sin(phase);
    jnt_cmd_publisher->publish(qdes_msg);
    rclcpp::WallRate(1 / dt).sleep();
  }
}
