#include "custom_msgs/msg/vec_of_doubles.hpp"
#include "rclcpp/rclcpp.hpp"

#define ROS_INFO printf

#include <vector>

void myCallback(const custom_msgs::msg::VecOfDoubles::SharedPtr msg)
{
  std::vector<double> vec_of_doubles = msg->dbl_vec;

  int nvals = vec_of_doubles.size();
  for (int i = 0; i < nvals; i++)
  {
    ROS_INFO("vec[%d] = %f\n", i, vec_of_doubles[i]);
  }
  ROS_INFO("\n");
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("vector_subscriber");

  auto sub =
      node->create_subscription<custom_msgs::msg::VecOfDoubles>("vec_topic", myCallback, rmw_qos_profile_default);

  rclcpp::spin(node);

  return 0;
}
