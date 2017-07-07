#include <iostream>
#include <string>
#include "object_manipulation_msgs/srv/query.hpp"
#include "rclcpp/rclcpp.hpp"
#include "xform_utils/xform_utils.hpp"

#include "rcutils/logging_macros.h"
#define ROS_INFO RCUTILS_LOG_INFO
#define ROS_WARN RCUTILS_LOG_WARN

object_manipulation_msgs::srv::Query_Response::SharedPtr send_blocking_request(
    rclcpp::node::Node::SharedPtr node,
    rclcpp::client::Client<object_manipulation_msgs::srv::Query>::SharedPtr client,
    object_manipulation_msgs::srv::Query_Request::SharedPtr request)
{
  std::cout << "Sending..." << std::endl;
  auto response = client->async_send_request(request);
  // wait for the response
  if (rclcpp::spin_until_future_complete(node, response) == rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    std::cout << "Received." << std::endl;
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

  auto node = rclcpp::node::Node::make_shared("object_manip_query_client");
  auto client =
      node->create_client<object_manipulation_msgs::srv::Query>("object_manip_query_svc");

  auto request = std::make_shared<object_manipulation_msgs::srv::Query::Request>();
  XformUtils xformUtils;

  int gripper_id;
  int object_id;
  int grasp_option;
  int query_code;
  int n_options;
  std::vector<geometry_msgs::msg::Pose> grasp_pose_options;

  while (rclcpp::ok())
  {
    std::cout << std::endl;
    std::cout << "enter a gripper code: (e.g. 4 for RETHINK_ELECTRIC_GRIPPER_RT, or -1 to quit): ";
    std::cin >> gripper_id;
    if (gripper_id < 0)
      return 0;
    request->gripper_id = gripper_id;

    std::cout << "enter an object ID code (e.g. 1000 for TOY_BLOCK): ";
    std::cin >> object_id;
    request->object_id = object_id;

    std::cout << "enter a query code (e.g. 1 for GRASP_STRATEGY_OPTIONS_QUERY): ";
    std::cin >> query_code;
    request->query_code = query_code;

    if (query_code < object_manipulation_msgs::srv::Query::Request::GET_GRASP_POSE_TRANSFORMS)
    {
      auto response = send_blocking_request(node, client, request);
      if (!response->valid_reply)
      {
        ROS_WARN("inquiry not valid");
      }
      else
      {
        n_options = response->grasp_strategy_options.size();
        if (n_options < 1)
        {
          ROS_WARN("no grasp strategy options known for this case");
        }
        else
        {
          for (int i = 0; i < n_options; i++)
          {
            ROS_INFO("grasp strategy option %d is: %d", i, response->grasp_strategy_options[i]);
          }
        }
      }
    }
    else
    {
      std::cout << "enter grasp-strategy option code: ";
      std::cin >> grasp_option;
      request->grasp_option = grasp_option;
      auto response = send_blocking_request(node, client, request);
      if (!response->valid_reply)
      {
        ROS_WARN("inquiry not valid");
      }
      else
      {
        int n_pose_options = response->gripper_pose_options.size();
        if (n_pose_options < 1)
        {
          ROS_WARN("no pose options returned for this case");
        }
        else
        {
          grasp_pose_options = response->gripper_pose_options;
          ROS_INFO("gripper pose options: ");
          for (int i = 0; i < n_pose_options; i++)
          {
            xformUtils.printPose(grasp_pose_options[i]);
          }
        }
      }
    }
  }
}
