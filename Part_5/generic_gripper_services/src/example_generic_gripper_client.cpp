#include "generic_gripper_services/srv/generic_gripper_interface.hpp"
#include "rclcpp/rclcpp.hpp"

#include "rcutils/logging_macros.h"
#define ROS_INFO RCUTILS_LOG_INFO
#define ROS_WARN RCUTILS_LOG_WARN

generic_gripper_services::srv::GenericGripperInterface_Response::SharedPtr
send_blocking_request(rclcpp::node::Node::SharedPtr node,
                      rclcpp::client::Client<generic_gripper_services::srv::GenericGripperInterface>::SharedPtr client,
                      generic_gripper_services::srv::GenericGripperInterface_Request::SharedPtr request)
{
  auto response = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, response) == rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    return response.get();
  }
  else
  {
    return NULL;
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::node::Node::make_shared("generic_gripper_interface_client");
  auto client = node->create_client<generic_gripper_services::srv::GenericGripperInterface>("generic_gripper_svc");

  auto request = std::make_shared<generic_gripper_services::srv::GenericGripperInterface::Request>();
  request->cmd_code = generic_gripper_services::srv::GenericGripperInterface::Request::TEST_PING;

  ROS_INFO("sending a test ping");
  bool success = false;
  generic_gripper_services::srv::GenericGripperInterface_Response::SharedPtr response;
  while (!success)
  {
    response = send_blocking_request(node, client, request);
    success = response->success;
    ROS_INFO("retrying...is the gripper service running?");
    rclcpp::WallRate(2).sleep();
    rclcpp::spin_some(node);
  }
  ROS_INFO("got test ping response");

  ROS_INFO("try sending release command: ");
  request->cmd_code = generic_gripper_services::srv::GenericGripperInterface::Request::RELEASE;
  response = send_blocking_request(node, client, request);
  success = response->success;
  if (success)
  {
    ROS_INFO("responded w/ success");
  }
  else
  {
    ROS_WARN("responded with failure");
  }

  ROS_INFO("try sending GRASP command: ");
  request->cmd_code = generic_gripper_services::srv::GenericGripperInterface::Request::GRASP;
  response = send_blocking_request(node, client, request);
  success = response->success;
  if (success)
  {
    ROS_INFO("responded w/ success");
  }
  else
  {
    ROS_WARN("responded with failure");
  }

  return 0;
}
