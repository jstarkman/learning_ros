#define _USE_MATH_DEFINES
#include <math.h>
#include "example_ros_msg/msg/example_message.hpp"
#include "rclcpp/rclcpp.hpp"

#include "rcutils/logging_macros.h"
#define ROS_INFO RCUTILS_LOG_INFO

/* Ripped from ~/ros2_ws/src/ros2/demos/intra_process_demo/src/image_pipeline/common.hpp */
void set_now(builtin_interfaces::msg::Time& time)
{
  std::chrono::nanoseconds now = std::chrono::high_resolution_clock::now().time_since_epoch();
  if (now <= std::chrono::nanoseconds(0))
  {
    time.sec = time.nanosec = 0;
  }
  else
  {
    time.sec = static_cast<builtin_interfaces::msg::Time::_sec_type>(now.count() / 1000000000);
    time.nanosec = now.count() % 1000000000;
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::node::Node::make_shared("example_ros_message_publisher");
  auto pub = node->create_publisher<example_ros_msg::msg::ExampleMessage>("example_topic", rmw_qos_profile_default);

  double counter = 0.0;
  double sqrt_arg;
  rclcpp::rate::WallRate naptime(1.0);

  auto my_new_message = std::make_shared<example_ros_msg::msg::ExampleMessage>();
  set_now(my_new_message->header.stamp);
  my_new_message->header.frame_id = "base_frame";
  my_new_message->demo_double = 100.0;

  while (rclcpp::ok())
  {
    set_now(my_new_message->header.stamp);
    my_new_message->demo_int *= 2.0;
    sqrt_arg = my_new_message->demo_double;
    my_new_message->demo_double = sqrt(sqrt_arg);

    pub->publish(my_new_message);
    naptime.sleep();
  }
}
