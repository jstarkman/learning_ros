#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

auto g_velocity = std::make_shared<std_msgs::msg::Float64>();
auto g_force = std::make_shared<std_msgs::msg::Float64>();

void myCallback(const std_msgs::msg::Float64::SharedPtr message_holder)
{
  printf("received force value is: %f\n", message_holder->data);
  g_force->data = message_holder->data;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::node::Node::make_shared("minimal_simulator");
  auto my_subscriber =
      node->create_subscription<std_msgs::msg::Float64>("force_cmd", myCallback, rmw_qos_profile_default);
  auto my_publisher = node->create_publisher<std_msgs::msg::Float64>("velocity", rmw_qos_profile_default);

  double mass = 1.0;
  double dt = 0.01;
  double sample_rate = 1.0 / dt;
  rclcpp::WallRate naptime(sample_rate);
  g_velocity->data = 0.0;
  g_force->data = 0.0;
  while (rclcpp::ok())
  {
    g_velocity->data = g_velocity->data + (g_force->data / mass) * dt;

    my_publisher->publish(g_velocity);
    printf("velocity = %f\n", g_velocity->data);
    rclcpp::spin(node);
    naptime.sleep();
  }
  return 0;
}
