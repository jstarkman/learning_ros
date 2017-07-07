#include <stdio.h>
#include <string.h>
#include <chrono>
#include "example_rviz_marker/srv/simple_float_srv_msg.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "rcutils/logging_macros.h"
#define ROS_INFO RCUTILS_LOG_INFO
#define ROS_WARN RCUTILS_LOG_WARN

using namespace std;

double g_z_height = 0.0;
bool g_trigger = true;

void displaySvcCB(const std::shared_ptr<rmw_request_id_t> request_header,
                  const example_rviz_marker::srv::SimpleFloatSrvMsg::Request::SharedPtr request,
                  example_rviz_marker::srv::SimpleFloatSrvMsg::Response::SharedPtr response)
{
  g_z_height = request->request_float32;
  ROS_INFO("example_rviz_marker: received request for height %f", g_z_height);
  g_trigger = true;
  response->resp = true;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::node::Node::make_shared("example_rviz_marker");
  auto vis_pub =
      node->create_publisher<visualization_msgs::msg::Marker>("example_marker_topic", rmw_qos_profile_default);
  auto service = node->create_service<example_rviz_marker::srv::SimpleFloatSrvMsg>("rviz_marker_svc", displaySvcCB);

  auto marker = std::make_shared<visualization_msgs::msg::Marker>();
  geometry_msgs::msg::Point point;  // copied into marker repeatedly

  marker->header.frame_id = "/world";
  marker->header.stamp = rclcpp::Time::now();
  marker->ns = "my_namespace";
  marker->id = 0;

  marker->type = visualization_msgs::msg::Marker::SPHERE_LIST;
  marker->action = visualization_msgs::msg::Marker::ADD;

  marker->pose.orientation.x = 0.0;
  marker->pose.orientation.y = 0.0;
  marker->pose.orientation.z = 0.0;
  marker->pose.orientation.w = 1.0;
  marker->scale.x = 0.02;
  marker->scale.y = 0.02;
  marker->scale.z = 0.02;
  marker->color.a = 1.0;
  marker->color.r = 1.0;
  marker->color.g = 0.0;
  marker->color.b = 0.0;

  double z_des;

  double x_min = -1.0;
  double x_max = 1.0;
  double y_min = -1.0;
  double y_max = 1.0;
  double dx_des = 0.1;
  double dy_des = 0.1;

  rclcpp::WallRate loop_rate(10);

  while (rclcpp::ok())
  {
    if (g_trigger)
    {
      g_trigger = false;
      z_des = g_z_height;
      ROS_INFO("constructing plane of markers at height %f", z_des);
      marker->header.stamp = rclcpp::Time::now();
      marker->points.clear();

      for (double x_des = x_min; x_des < x_max; x_des += dx_des)
      {
        for (double y_des = y_min; y_des < y_max; y_des += dy_des)
        {
          point.x = x_des;
          point.y = y_des;
          point.z = z_des;
          marker->points.push_back(point);
        }
      }
    }
    loop_rate.sleep();

    vis_pub->publish(marker);
    rclcpp::spin_some(node);
  }
  return 0;
}
