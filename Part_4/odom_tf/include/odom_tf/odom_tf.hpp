#ifndef ODOM_TF_H_
#define ODOM_TF_H_

#define _USE_MATH_DEFINES
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tf2/LinearMath/QuadWord.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/buffer_core.h"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/buffer_interface.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "xform_utils/xform_utils.hpp"

#include "rcutils/logging_macros.h"
#define ROS_INFO RCUTILS_LOG_INFO
#define ROS_WARN RCUTILS_LOG_WARN

class OdomTf
{
public:
  OdomTf(rclcpp::node::Node::SharedPtr node);

  XformUtils xform_utils;
  geometry_msgs::msg::TransformStamped get_tfBaseLinkWrtDriftyOdom()
  {
    return stfBaseLinkWrtDriftyOdom_;
  }

  geometry_msgs::msg::TransformStamped stfBaseLinkWrtOdom_;
  geometry_msgs::msg::TransformStamped stfOdomWrtMap_;
  geometry_msgs::msg::TransformStamped stfBaseLink_wrt_Map_;

  geometry_msgs::msg::TransformStamped stfAmclBaseLinkWrtMap_;
  geometry_msgs::msg::TransformStamped stfEstBaseWrtMap_;

  geometry_msgs::msg::PoseStamped base_link_wrt_odom_;
  geometry_msgs::msg::PoseStamped base_link_wrt_map_;

  geometry_msgs::msg::PoseStamped estBasePoseWrtMap_;

  geometry_msgs::msg::TransformStamped tfLink2ToOdom_;
  geometry_msgs::msg::TransformStamped stfBaseLinkWrtDriftyOdom_;
  geometry_msgs::msg::TransformStamped stfDriftyOdomWrtBase_;
  geometry_msgs::msg::TransformStamped stfDriftyOdomWrtMap_;

  bool odom_tf_ready_;
  bool odom_tf_is_ready()
  {
    return odom_tf_ready_;
  }
  bool odom_ready_;
  bool amcl_ready_;

private:
  rclcpp::node::Node::SharedPtr node_;

  rclcpp::subscription::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  rclcpp::subscription::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_subscriber_;

  tf2_ros::Buffer* tfBuffer_;
  tf2_ros::TransformListener* tfListener_;
  tf2_ros::TransformBroadcaster* tfBr_;

  void initializeSubscribers();

  rclcpp::publisher::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;

  int odom_count_;

  // double odom_x_;
  // double odom_y_;
  double odom_phi_;
  // geometry_msgs::msg::Quaternion odom_quat_;
  geometry_msgs::msg::Quaternion amcl_quat_;
  geometry_msgs::msg::Pose amcl_pose_;
};

#endif
