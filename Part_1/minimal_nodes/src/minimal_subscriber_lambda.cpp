#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float64.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("minimal_subscriber");

  auto sub = node->create_subscription<std_msgs::msg::Float64>(
      "topic1",
      /* Example of a closure, introduced in C++11.  Outer variables
       * to be enclosed within the lambda's scope go in the [] (here
       * there are none, hence the empty brackets).  Function
       * arguments go between the () and code between the {} as usual.
       * For more examples, see other packages, e.g.,
       * Part_4/mobot_pub_des_state/src/pub_des_state.cpp
       * (those are for services, but services take callbacks too,
       * and thus also take lambda expressions).
       */
      [](const std_msgs::msg::Float64::SharedPtr msg) { std::cout << "received value is: " << msg->data << std::endl; },
      rmw_qos_profile_default);

  rclcpp::spin(node);

  return 0;
}
