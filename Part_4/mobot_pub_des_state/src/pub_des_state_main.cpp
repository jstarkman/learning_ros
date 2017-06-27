#include "pub_des_state.cpp"
#include "pub_des_state.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::node::Node::make_shared("des_state_publisher");

  DesStatePublisher desStatePublisher(node);

  rclcpp::WallRate looprate(1 / dt);
  desStatePublisher.set_init_pose(0, 0, 0);

  desStatePublisher.append_path_queue(5.0, 0.0, 0.0);
  desStatePublisher.append_path_queue(0.0, 0.0, 0.0);

  while (rclcpp::ok())
  {
    desStatePublisher.pub_next_state();
    rclcpp::spin_some(node);
    looprate.sleep();
  }
}
