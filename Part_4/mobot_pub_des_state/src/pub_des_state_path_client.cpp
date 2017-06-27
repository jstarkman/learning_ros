#include <iostream>
#include <string>
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mobot_pub_des_state/srv/path.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

#define ROS_INFO printf
#define ROS_WARN printf

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

  while (!client.exists())
  {
    ROS_INFO("waiting for service...");
    rclcpp::WallRate(1.0).sleep();
  }
  ROS_INFO("connected client to service");
  mobot_pub_des_state::srv::Path path_srv;

  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = "world";
  geometry_msgs::msg::Pose pose;
  pose.position.x = 5.0;
  pose.position.y = 0.0;
  pose.position.z = 0.0;
  quat = convertPlanarPhi2Quaternion(0);
  pose.orientation = quat;
  pose_stamped.pose = pose;
  path_srv.request.path.poses.push_back(pose_stamped);

  pose.position.y = 5.0;
  pose_stamped.pose = pose;
  path_srv.request.path.poses.push_back(pose_stamped);

  pose.position.x = 0.0;
  pose_stamped.pose = pose;
  path_srv.request.path.poses.push_back(pose_stamped);

  pose.position.y = 0.0;
  pose_stamped.pose = pose;
  path_srv.request.path.poses.push_back(pose_stamped);

  pose_stamped.pose.orientation = convertPlanarPhi2Quaternion(0);
  path_srv.request.path.poses.push_back(pose_stamped);

  client->call(path_srv);

  return 0;
}
