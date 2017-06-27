#include "odom_tf/odom_tf.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("OdomTf_node");

  ROS_INFO("main: instantiating an object of type OdomTf");
  OdomTf odomTf(node);

  ROS_INFO("starting main loop");
  rclcpp::WallRate sleep_timer(50.0);
  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);
    sleep_timer.sleep();
  }
  return 0;
}
