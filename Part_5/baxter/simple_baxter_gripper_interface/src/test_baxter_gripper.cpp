#include "baxter_core_msgs/msg/end_effector_command.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::node::Node::make_shared("test_baxter_gripper");

  auto gripper_publisher_object = node->create_publisher<baxter_core_msgs::msg::EndEffectorCommand>(
      "/robot/end_effector/right_gripper/command", rmw_qos_profile_default);
  rclcpp::WallRate naptime(1.0);

  auto gripper_cmd_open = std::make_shared<baxter_core_msgs::msg::EndEffectorCommand>();
  gripper_cmd_open->command = "go";
  gripper_cmd_open->args = "{'position': 100.0}'";
  gripper_cmd_open->sender = "gripper_publisher";

  auto gripper_cmd_close = std::make_shared<baxter_core_msgs::msg::EndEffectorCommand>();
  gripper_cmd_close->command = "go";
  gripper_cmd_close->args = "{'position': 0.0}'";
  gripper_cmd_close->sender = "gripper_publisher";

  while (rclcpp::ok())
  {
    gripper_publisher_object->publish(gripper_cmd_close);
    naptime.sleep();
    gripper_publisher_object->publish(gripper_cmd_open);
    naptime.sleep();
  }
}
