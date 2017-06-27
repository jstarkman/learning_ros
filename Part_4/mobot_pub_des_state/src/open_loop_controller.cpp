#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

rclcpp::publisher::Publisher<geometry_msgs::msg::Twist>::SharedPtr g_twist_republisher;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::node::Node::make_shared("open_loop_controller");
  g_twist_republisher = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", rmw_qos_profile_default);

  auto des_state_subscriber = node->create_subscription<nav_msgs::msg::Odometry>(
      "/desState", [](const nav_msgs::msg::Odometry::SharedPtr msg) { g_twist_republisher->publish(msg->twist.twist); },
      rmw_qos_profile_default);

  rclcpp::spin(node);
}
