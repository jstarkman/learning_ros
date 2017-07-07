#include <iostream>
#include <string>
#include "example_ros_service/srv/path_srv.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

/* ugly hack until rosconsole works */
#include "rcutils/logging_macros.h"
#define ROS_INFO RCUTILS_LOG_INFO

void callback(const std::shared_ptr<rmw_request_id_t> request_header,
              const std::shared_ptr<example_ros_service::srv::PathSrv::Request> request,
              std::shared_ptr<example_ros_service::srv::PathSrv::Response> response)
{
  (void)request_header;
  ROS_INFO("callback activated");
  int npts = request->nav_path.poses.size();
  ROS_INFO("received path request with %d poses", npts);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::node::Node::make_shared("path_service");
  auto service = node->create_service<example_ros_service::srv::PathSrv>("example_minimal_service", callback);

  ROS_INFO("Ready accept paths.");
  rclcpp::spin(node);

  return 0;
}
