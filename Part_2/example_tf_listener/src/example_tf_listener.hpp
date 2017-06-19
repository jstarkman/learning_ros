#ifndef EXAMPLE_TF_LISTENER_H_
#define EXAMPLE_TF_LISTENER_H_

#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/buffer_interface.h"
#include "tf2_ros/transform_listener.h"

#define ROS_INFO printf
#define ROS_WARN printf

class DemoTfListener
{
public:
  DemoTfListener(rclcpp::node::Node::SharedPtr node);

  geometry_msgs::msg::PoseStamped get_pose_from_transform(geometry_msgs::msg::TransformStamped tf);

  bool multiply_stamped_tfs(geometry_msgs::msg::TransformStamped A_stf, geometry_msgs::msg::TransformStamped B_stf,
                            geometry_msgs::msg::TransformStamped& C_stf);

  void printStampedTf(geometry_msgs::msg::TransformStamped sTf);
  void printStampedPose(geometry_msgs::msg::PoseStamped stPose);
  void printTf(tf2::Transform tf);

  tf2::Transform get_tf_from_stamped_tf(geometry_msgs::msg::TransformStamped sTf);
  tf2_ros::TransformListener* tfListener_;
  tf2_ros::Buffer* tfBuffer_;

private:
  rclcpp::node::Node::SharedPtr node;
};

#endif
