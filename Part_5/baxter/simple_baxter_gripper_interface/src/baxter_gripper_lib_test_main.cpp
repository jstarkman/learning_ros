#include "rclcpp/rclcpp.hpp"
#include "simple_baxter_gripper_interface/simple_baxter_gripper_interface.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::node::Node::make_shared("baxter_gripper_test_main");

  BaxterGripper baxterGripper(node);

  while (baxterGripper.get_right_gripper_pos() < -0.5)
  {
    rclcpp::spin_some(node);
    rclcpp::WallRate(0.01).sleep();
    ROS_INFO("waiting for right gripper position filter to settle; pos = %f", baxterGripper.get_right_gripper_pos());
  }

  ROS_INFO("closing right gripper");
  baxterGripper.right_gripper_close();
  rclcpp::WallRate(1.0).sleep();
  ROS_INFO("opening right gripper");
  baxterGripper.right_gripper_open();
  rclcpp::spin_some(node);
  ROS_INFO("right gripper pos = %f; waiting for pos>95", baxterGripper.get_right_gripper_pos());
  while (baxterGripper.get_right_gripper_pos() < 95.0)
  {
    baxterGripper.right_gripper_open();
    rclcpp::spin_some(node);
    ROS_INFO("gripper pos = %f", baxterGripper.get_right_gripper_pos());
    rclcpp::WallRate(0.01).sleep();
  }

  ROS_INFO("closing left gripper");
  baxterGripper.left_gripper_close();
  rclcpp::WallRate(1.0).sleep();
  ROS_INFO("opening left gripper");
  baxterGripper.left_gripper_open();

  ROS_INFO("closing right gripper");
  baxterGripper.right_gripper_close();
  rclcpp::spin_some(node);
  ROS_INFO("right gripper pos = %f; waiting for pos<90", baxterGripper.get_right_gripper_pos());
  while (baxterGripper.get_right_gripper_pos() > 90.0)
  {
    baxterGripper.right_gripper_close();
    rclcpp::spin_some(node);
    ROS_INFO("gripper pos = %f", baxterGripper.get_right_gripper_pos());
    rclcpp::WallRate(0.01).sleep();
  }

  return 0;
}
