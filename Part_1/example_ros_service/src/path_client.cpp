#include "nav_msgs/msg/path.hpp"
#include <iostream>
#include <string>
#include "example_ros_service/srv/path_srv.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

/* ugly hack until rosconsole works */
#define ROS_INFO printf

geometry_msgs::msg::Quaternion convertPlanarPhi2Quaternion(double phi)
{
  geometry_msgs::msg::Quaternion quaternion;
  quaternion.x = 0.0;
  quaternion.y = 0.0;
  quaternion.z = std::sin(phi / 2.0);
  quaternion.w = std::cos(phi / 2.0);
  return quaternion;
}

example_ros_service::srv::PathSrv_Response::SharedPtr send_blocking_request(
    rclcpp::node::Node::SharedPtr node, rclcpp::client::Client<example_ros_service::srv::PathSrv>::SharedPtr client,
    example_ros_service::srv::PathSrv_Request::SharedPtr request)
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

  auto node = rclcpp::node::Node::make_shared("path_client");
  auto client = node->create_client<example_ros_service::srv::PathSrv>("path_service");

  auto request = std::make_shared<example_ros_service::srv::PathSrv::Request>();

  geometry_msgs::msg::Quaternion quat;

  /* JAS cannot find ROS2 equivalent to client.exists()
    while (!client.exists())
    {
      ROS_INFO("waiting for service...");
      rclcpp::WallRate naptime(1.0);
      naptime.sleep();
    }
    ROS_INFO("connected client to service");
  */

  auto pose_stamped = std::make_shared<geometry_msgs::msg::PoseStamped>();
  auto pose = pose_stamped->pose;
  pose.position.x = 1.0;
  pose.position.y = 0.0;
  pose.position.z = 0.0;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 1.0;
  request->nav_path.poses.push_back(*pose_stamped);

  quat = convertPlanarPhi2Quaternion(1.57);
  pose_stamped->pose.orientation = quat;
  pose_stamped->pose.position.y = 1.0;
  request->nav_path.poses.push_back(*pose_stamped);

  quat = convertPlanarPhi2Quaternion(3.14);
  pose_stamped->pose.orientation = quat;

  request->nav_path.poses.push_back(*pose_stamped);
  send_blocking_request(node, client, request);

  return 0;
}
