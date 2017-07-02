// baxter_gripper:  a library to simplify baxter gripper I/O
// wsn, August, 2016
#ifndef BAXTER_GRIPPER_H
#define BAXTER_GRIPPER_H
#include "baxter_core_msgs/msg/end_effector_command.hpp"
#include "baxter_core_msgs/msg/end_effector_state.hpp"
#include "rclcpp/rclcpp.hpp"

#define ROS_INFO printf
#define ROS_WARN printf

class BaxterGripper
{
private:
  rclcpp::node::Node::SharedPtr node_;
  rclcpp::subscription::Subscription<baxter_core_msgs::msg::EndEffectorState>::SharedPtr gripper_subscriber_right_,
      gripper_subscriber_left_;
  rclcpp::publisher::Publisher<baxter_core_msgs::msg::EndEffectorCommand>::SharedPtr gripper_publisher_right_,
      gripper_publisher_left_;

  void initializeSubscribers();
  void initializePublishers();
  double gripper_pos_filter_val_, right_gripper_pos_, left_gripper_pos_;
  baxter_core_msgs::msg::EndEffectorCommand gripper_cmd_open, gripper_cmd_close;

public:
  void right_gripper_close(void)
  {
    gripper_publisher_right_->publish(gripper_cmd_close);
  };
  void left_gripper_close(void)
  {
    gripper_publisher_left_->publish(gripper_cmd_close);
  };
  void right_gripper_open(void)
  {
    gripper_publisher_right_->publish(gripper_cmd_open);
  };
  void left_gripper_open(void)
  {
    gripper_publisher_left_->publish(gripper_cmd_open);
  };
  double get_right_gripper_pos(void)
  {
    return right_gripper_pos_;
  };
  double get_left_gripper_pos(void)
  {
    return left_gripper_pos_;
  };

  BaxterGripper(rclcpp::node::Node::SharedPtr node);
};
#endif
