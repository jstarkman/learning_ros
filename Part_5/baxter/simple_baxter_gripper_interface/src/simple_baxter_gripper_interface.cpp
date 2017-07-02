#include "simple_baxter_gripper_interface/simple_baxter_gripper_interface.hpp"
#include "rclcpp/rclcpp.hpp"

BaxterGripper::BaxterGripper(rclcpp::node::Node::SharedPtr node) : node_(node)
{
  gripper_cmd_open.command = "go";
  gripper_cmd_open.args = "{'position': 100.0}'";
  gripper_cmd_open.sender = "gripper_publisher";
  gripper_cmd_close.command = "go";
  gripper_cmd_close.args = "{'position': 0.0}'";
  gripper_cmd_close.sender = "gripper_publisher";
  gripper_pos_filter_val_ = 0.2;
  right_gripper_pos_ = -0.2;
  left_gripper_pos_ = -10.2;
  initializeSubscribers();
  initializePublishers();
}
void BaxterGripper::initializeSubscribers()
{
  ROS_INFO("Initializing Subscribers");
  gripper_subscriber_right_ = node_->create_subscription<baxter_core_msgs::msg::EndEffectorState>(
      "/robot/end_effector/right_gripper/state",
      [this](const baxter_core_msgs::msg::EndEffectorState::SharedPtr gripper_state) {
        right_gripper_pos_ =
            (1.0 - gripper_pos_filter_val_) * right_gripper_pos_ + gripper_pos_filter_val_ * gripper_state->position;
      },
      rmw_qos_profile_default);

  gripper_subscriber_left_ = node_->create_subscription<baxter_core_msgs::msg::EndEffectorState>(
      "/robot/end_effector/left_gripper/state",
      [this](const baxter_core_msgs::msg::EndEffectorState::SharedPtr gripper_state) {
        left_gripper_pos_ =
            (1.0 - gripper_pos_filter_val_) * left_gripper_pos_ + gripper_pos_filter_val_ * gripper_state->position;
      },
      rmw_qos_profile_default);
}

void BaxterGripper::initializePublishers()
{
  ROS_INFO("Initializing Publishers");
  gripper_publisher_right_ = node_->create_publisher<baxter_core_msgs::msg::EndEffectorCommand>(
      "/robot/end_effector/right_gripper/command", rmw_qos_profile_default);
  gripper_publisher_left_ = node_->create_publisher<baxter_core_msgs::msg::EndEffectorCommand>(
      "/robot/end_effector/left_gripper/command", rmw_qos_profile_default);
}
