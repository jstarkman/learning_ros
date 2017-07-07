#include <iostream>
#include <string>
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mobot_pub_des_state/srv/path.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

#include "rcutils/logging_macros.h"
#define ROS_INFO RCUTILS_LOG_INFO
#define ROS_WARN RCUTILS_LOG_WARN

geometry_msgs::msg::Quaternion convertPlanarPhi2Quaternion(double phi)
{
  geometry_msgs::msg::Quaternion quaternion;
  quaternion.x = 0.0;
  quaternion.y = 0.0;
  quaternion.z = sin(phi / 2.0);
  quaternion.w = cos(phi / 2.0);
  return quaternion;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::node::Node::make_shared("append_path_client");
  auto client = node->create_client<mobot_pub_des_state::srv::Path>("append_path_queue_service");

  geometry_msgs::msg::Quaternion quat;
  auto path_srv = std::make_shared<mobot_pub_des_state::srv::Path::Request>();

  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = "world";
  geometry_msgs::msg::Pose pose;
  pose.position.x = 3.0;
  pose.position.y = 0.0;
  pose.position.z = 0.0;
  quat = convertPlanarPhi2Quaternion(0);
  pose.orientation = quat;
  pose_stamped.pose = pose;
  path_srv->path.poses.push_back(pose_stamped);

  pose.position.y = 3.0;
  pose_stamped.pose = pose;
  path_srv->path.poses.push_back(pose_stamped);

  pose.position.x = 0.0;
  pose_stamped.pose = pose;
  path_srv->path.poses.push_back(pose_stamped);

  pose.position.y = 0.0;
  pose_stamped.pose = pose;
  path_srv->path.poses.push_back(pose_stamped);

  pose_stamped.pose.orientation = convertPlanarPhi2Quaternion(0);
  path_srv->path.poses.push_back(pose_stamped);

  client->async_send_request(path_srv);

  return 0;
}
