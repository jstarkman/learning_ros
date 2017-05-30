#include "custom_msgs/msg/vec_of_doubles.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::node::Node::make_shared("vector_publisher");
  auto pub = node->create_publisher<custom_msgs::msg::VecOfDoubles>("vec_topic", rmw_qos_profile_default);

  auto vec_msg = std::make_shared<custom_msgs::msg::VecOfDoubles>();
  double counter = 0.0;
  rclcpp::rate::WallRate naptime(1.0);

  vec_msg->dbl_vec.resize(3);
  vec_msg->dbl_vec[0] = 1.414;
  vec_msg->dbl_vec[1] = 2.71828;
  vec_msg->dbl_vec[2] = 3.1416;
  vec_msg->dbl_vec.push_back(counter);

  while (rclcpp::ok())
  {
    counter += 1.0;
    vec_msg->dbl_vec.push_back(counter);
    pub->publish(vec_msg);
    naptime.sleep();
  }

  return 0;
}
