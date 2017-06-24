#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

auto g_velocity = std::make_shared<std_msgs::msg::Float64>();
auto g_vel_cmd = std::make_shared<std_msgs::msg::Float64>();
auto g_force = std::make_shared<std_msgs::msg::Float64>();

void myCallbackVelocity(const std_msgs::msg::Float64::SharedPtr message_holder)
{
  printf("received velocity value is: %f\n", message_holder->data);
  g_velocity->data = message_holder->data;
}

void myCallbackVelCmd(const std_msgs::msg::Float64::SharedPtr message_holder)
{
  printf("received velocity command value is: %f\n", message_holder->data);
  g_vel_cmd->data = message_holder->data;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("minimal_controller");
  auto my_subscriber_1 =
      node->create_subscription<std_msgs::msg::Float64>("velocity", myCallbackVelocity, rmw_qos_profile_default);
  auto my_subscriber_2 =
      node->create_subscription<std_msgs::msg::Float64>("vel_cmd", myCallbackVelCmd, rmw_qos_profile_default);
  auto my_publisher = node->create_publisher<std_msgs::msg::Float64>("force_cmd", rmw_qos_profile_default);

  double Kv = 1.0;
  double dt_controller = 0.1;

  double sample_rate = 1.0 / dt_controller;
  // from src/ros2/rclcpp/rclcpp/include/rclcpp/rate.hpp:
  // using Rate = GenericRate<std::chrono::system_clock>;
  // using WallRate = GenericRate<std::chrono::steady_clock>;
  rclcpp::WallRate naptime(sample_rate);
  g_velocity->data = 0.0;
  g_force->data = 0.0;
  g_vel_cmd->data = 0.0;
  double vel_err = 0.0;

  while (rclcpp::ok())
  {
    vel_err = g_vel_cmd->data - g_velocity->data;

    g_force->data = Kv * vel_err;

    my_publisher->publish(g_force);

    printf("force command = %f\n", g_force->data);
    rclcpp::spin_some(node);
    naptime.sleep();
  }
  return 0;
}
