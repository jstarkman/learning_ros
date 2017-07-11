#ifndef XFORM_UTILS_H_
#define XFORM_UTILS_H_

#define _USE_MATH_DEFINES
#include <math.h>
#include <stdlib.h>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
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
#include "tf2_ros/buffer.h"
#include "tf2_ros/buffer_interface.h"
#include "tf2_ros/transform_listener.h"

class XformUtils
{
public:
  Eigen::Affine3f transformTFToAffine3f(const tf2::Transform &t);
  Eigen::Affine3d transformTFToAffine3d(const tf2::Transform &t);
  double convertPlanarQuat2Phi(geometry_msgs::msg::Quaternion quaternion);
  geometry_msgs::msg::Quaternion convertPlanarPsi2Quaternion(double psi);
  tf2::Transform get_tf_from_stamped_tf(geometry_msgs::msg::TransformStamped sTf);
  geometry_msgs::msg::TransformStamped get_stamped_tf_from_tf(tf2::Transform tf);
  geometry_msgs::msg::PoseStamped get_pose_from_stamped_tf(geometry_msgs::msg::TransformStamped sTf);
  bool multiply_stamped_tfs(geometry_msgs::msg::TransformStamped A_stf, geometry_msgs::msg::TransformStamped B_stf,
                            geometry_msgs::msg::TransformStamped &C_stf);
  geometry_msgs::msg::TransformStamped stamped_transform_inverse(geometry_msgs::msg::TransformStamped sTf);
  geometry_msgs::msg::Pose transformEigenAffine3dToPose(Eigen::Affine3d e);
  Eigen::Affine3d transformPoseToEigenAffine3d(geometry_msgs::msg::Pose pose);
  Eigen::Affine3d transformPoseToEigenAffine3d(geometry_msgs::msg::PoseStamped stPose);
  Eigen::Affine3d transformStampedTfToEigenAffine3d(geometry_msgs::msg::TransformStamped sTf);
  geometry_msgs::msg::TransformStamped convert_pose_to_stampedTransform(geometry_msgs::msg::Pose pose,
                                                                        std::string child_frame_id);
  geometry_msgs::msg::TransformStamped convert_poseStamped_to_stampedTransform(geometry_msgs::msg::PoseStamped stPose,
                                                                               std::string child_frame_id);
  geometry_msgs::msg::PoseStamped get_pose_stamped_from_odom(nav_msgs::msg::Odometry odom);

  void test_stf(geometry_msgs::msg::PoseStamped stPose);
  void printTf(tf2::Transform tf);
  void printStampedTf(geometry_msgs::msg::TransformStamped sTf);
  void printStampedPose(geometry_msgs::msg::PoseStamped stPose);
  void printPose(geometry_msgs::msg::Pose pose);
  // overload printPose to work with either Pose or PoseStamped:
  void printPose(geometry_msgs::msg::PoseStamped stPose)
  {
    printStampedPose(stPose);
  }
  void printAffine(Eigen::Affine3d affine);
};

#endif
