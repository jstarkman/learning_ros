#include "builtin_interfaces/msg/duration.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging_macros.h"
#include "visualization_msgs/msg/marker.hpp"

#include <math.h>
#include <chrono>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <Eigen/Geometry>

geometry_msgs::msg::PoseStamped g_stamped_pose;
Eigen::Affine3d g_affine_marker_pose;

visualization_msgs::msg::Marker::SharedPtr arrow_marker_x;
visualization_msgs::msg::Marker::SharedPtr arrow_marker_y;
visualization_msgs::msg::Marker::SharedPtr arrow_marker_z;

void update_arrows()
{
  geometry_msgs::msg::Point origin, arrow_x_tip, arrow_y_tip, arrow_z_tip;
  Eigen::Matrix3d R;
  Eigen::Quaterniond quat;
  quat.x() = g_stamped_pose.pose.orientation.x;
  quat.y() = g_stamped_pose.pose.orientation.y;
  quat.z() = g_stamped_pose.pose.orientation.z;
  quat.w() = g_stamped_pose.pose.orientation.w;
  R = quat.toRotationMatrix();
  Eigen::Vector3d x_vec, y_vec, z_vec;
  double veclen = 0.2;
  x_vec = R.col(0) * veclen;
  y_vec = R.col(1) * veclen;
  z_vec = R.col(2) * veclen;

  origin = g_stamped_pose.pose.position;

  arrow_x_tip = origin;
  arrow_x_tip.x += x_vec(0);
  arrow_x_tip.y += x_vec(1);
  arrow_x_tip.z += x_vec(2);

  arrow_marker_x->points.clear();
  arrow_marker_x->points.push_back(origin);
  arrow_marker_x->points.push_back(arrow_x_tip);
  arrow_marker_x->header = g_stamped_pose.header;

  arrow_y_tip = origin;
  arrow_y_tip.x += y_vec(0);
  arrow_y_tip.y += y_vec(1);
  arrow_y_tip.z += y_vec(2);

  arrow_marker_y->points.clear();
  arrow_marker_y->points.push_back(origin);
  arrow_marker_y->points.push_back(arrow_y_tip);
  arrow_marker_y->header = g_stamped_pose.header;

  arrow_z_tip = origin;
  arrow_z_tip.x += z_vec(0);
  arrow_z_tip.y += z_vec(1);
  arrow_z_tip.z += z_vec(2);

  arrow_marker_z->points.clear();
  arrow_marker_z->points.push_back(origin);
  arrow_marker_z->points.push_back(arrow_z_tip);
  arrow_marker_z->header = g_stamped_pose.header;
}

void init_markers()
{
  arrow_marker_x = std::make_shared<visualization_msgs::msg::Marker>();
  arrow_marker_y = std::make_shared<visualization_msgs::msg::Marker>();
  arrow_marker_z = std::make_shared<visualization_msgs::msg::Marker>();

  g_stamped_pose.header.stamp = rclcpp::Time::now();
  g_stamped_pose.header.frame_id = "world";
  g_stamped_pose.pose.position.x = 0;
  g_stamped_pose.pose.position.y = 0;
  g_stamped_pose.pose.position.z = 0;
  g_stamped_pose.pose.orientation.x = 0;
  g_stamped_pose.pose.orientation.y = 0;
  g_stamped_pose.pose.orientation.z = 0;
  g_stamped_pose.pose.orientation.w = 1;

  arrow_marker_x->type = visualization_msgs::msg::Marker::ARROW;
  arrow_marker_x->action = visualization_msgs::msg::Marker::ADD;
  arrow_marker_x->ns = "triad_namespace";
  builtin_interfaces::msg::Duration dur;
  dur.sec = 1;
  arrow_marker_x->lifetime = dur;

  arrow_marker_x->scale.x = 0.01;
  arrow_marker_x->scale.y = 0.01;
  arrow_marker_x->scale.z = 0.01;
  arrow_marker_x->color.r = 1.0;
  arrow_marker_x->color.g = 0.0;
  arrow_marker_x->color.b = 0.0;
  arrow_marker_x->color.a = 1.0;
  arrow_marker_x->id = 0;
  arrow_marker_x->header = g_stamped_pose.header;

  arrow_marker_y = arrow_marker_x;
  arrow_marker_y->color.r = 0.0;
  arrow_marker_y->color.g = 1.0;
  arrow_marker_y->color.b = 0.0;
  arrow_marker_y->color.a = 1.0;
  arrow_marker_y->id = 1;

  arrow_marker_z = arrow_marker_x;
  arrow_marker_z->id = 2;
  arrow_marker_z->color.r = 0.0;
  arrow_marker_z->color.g = 0.0;
  arrow_marker_z->color.b = 1.0;
  arrow_marker_z->color.a = 1.0;

  update_arrows();
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("triad_display");

  auto pose_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      "triad_display_pose",
      [](const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg) {
        RCUTILS_LOG_DEBUG("got pose message");
        g_stamped_pose.header = pose_msg->header;
        g_stamped_pose.pose = pose_msg->pose;
      },
      rmw_qos_profile_default);

  auto vis_pub = node->create_publisher<visualization_msgs::msg::Marker>("triad_display", rmw_qos_profile_default);

  init_markers();

  rclcpp::WallRate timer(20);

  while (rclcpp::ok())
  {
    update_arrows();

    vis_pub->publish(arrow_marker_x);
    // ros::Duration(0.01).sleep();
    vis_pub->publish(arrow_marker_y);
    // ros::Duration(0.01).sleep();
    vis_pub->publish(arrow_marker_z);

    rclcpp::spin_some(node);
    timer.sleep();
  }
}
