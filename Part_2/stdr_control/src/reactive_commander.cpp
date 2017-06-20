#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

bool g_lidar_alarm = false;

void alarmCallback(const std_msgs::msg::Bool::SharedPtr alarm_msg)
{
  g_lidar_alarm = alarm_msg->data;
  if (g_lidar_alarm)
  {
    std::cout << "LIDAR alarm received!" << std::endl;
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::node::Node::make_shared("commander");
  auto twist_commander = node->create_publisher<geometry_msgs::msg::Twist>("/robot0/cmd_vel", rmw_qos_profile_default);
  auto alarm_subscriber =
      node->create_subscription<std_msgs::msg::Bool>("lidar_alarm", alarmCallback, rmw_qos_profile_default);

  double sample_dt = 0.01;
  double speed = 0.5;
  double yaw_rate = 0.5;
  double time_3_sec = 3.0;

  geometry_msgs::msg::Twist twist_cmd;

  twist_cmd.linear.x = 0.0;
  twist_cmd.linear.y = 0.0;
  twist_cmd.linear.z = 0.0;
  twist_cmd.angular.x = 0.0;
  twist_cmd.angular.y = 0.0;
  twist_cmd.angular.z = 0.0;

  rclcpp::WallRate loop_timer(1 / sample_dt);
  double timer = 0.0;

  for (int i = 0; i < 10; i++)
  {
    twist_commander->publish(twist_cmd);
    rclcpp::spin_some(node);
    loop_timer.sleep();
  }
  while (rclcpp::ok())
  {
    twist_cmd.angular.z = 0.0;
    twist_cmd.linear.x = speed;
    while (!g_lidar_alarm)
    {
      twist_commander->publish(twist_cmd);
      timer += sample_dt;
      rclcpp::spin_some(node);
      loop_timer.sleep();
    }

    twist_cmd.linear.x = 0.0;
    twist_cmd.angular.z = yaw_rate;
    timer = 0.0;
    while (g_lidar_alarm)
    {
      twist_commander->publish(twist_cmd);
      timer += sample_dt;
      rclcpp::spin_some(node);
      loop_timer.sleep();
    }
  }
}
