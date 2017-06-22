#include "rclcpp/rclcpp.hpp"
#include "xform_utils/xform_utils.hpp"

int main()
{
  XformUtils xformUtils;
  geometry_msgs::msg::Pose object_pose, gripper_pose;
  geometry_msgs::msg::PoseStamped gripper_pose_stamped;

  object_pose.position.x = 0.5;
  object_pose.position.y = -0.35;
  object_pose.position.z = -0.155;

  object_pose.orientation.x = 0;
  object_pose.orientation.y = 0;
  object_pose.orientation.z = 0.842;
  object_pose.orientation.w = 0.54;
  std::cout << "object pose: " << std::endl;
  xformUtils.printPose(object_pose);
  Eigen::Affine3d object_affine, gripper_affine;

  object_affine = xformUtils.transformPoseToEigenAffine3d(object_pose);
  std::cout << "object_affine origin: " << object_affine.translation().transpose() << std::endl;
  std::cout << "object_affine R matrix: " << std::endl;
  std::cout << object_affine.linear() << std::endl;

  Eigen::Vector3d x_axis, y_axis, z_axis;
  Eigen::Matrix3d R_object, R_gripper;
  R_object = object_affine.linear();
  x_axis = R_object.col(0);
  z_axis = -R_object.col(2);
  y_axis = z_axis.cross(x_axis);
  R_gripper.col(0) = x_axis;
  R_gripper.col(1) = y_axis;
  R_gripper.col(2) = z_axis;
  gripper_affine.linear() = R_gripper;
  gripper_affine.translation() = object_affine.translation();
  std::cout << "gripper_affine origin: " << gripper_affine.translation().transpose() << std::endl;
  std::cout << "gripper_affine R matrix: " << std::endl;
  std::cout << gripper_affine.linear() << std::endl;

  gripper_pose = xformUtils.transformEigenAffine3dToPose(gripper_affine);
  gripper_pose_stamped.pose = gripper_pose;
  builtin_interfaces::msg::Time t0;
  t0.sec = (uint64_t)std::chrono::system_clock::now().time_since_epoch().count();
  gripper_pose_stamped.header.stamp = t0;  // tf2::TimePoint(std::chrono::seconds(0));
  gripper_pose_stamped.header.frame_id = "torso";
  std::cout << "desired gripper pose: " << std::endl;
  xformUtils.printStampedPose(gripper_pose_stamped);

  geometry_msgs::msg::PoseStamped test_pst;
  std::string child_frame = "generic_gripper_frame";
  std::cout << "stf from stamped pose: " << std::endl;

  geometry_msgs::msg::TransformStamped gripper_stf =
      xformUtils.convert_poseStamped_to_stampedTransform(gripper_pose_stamped, "generic_gripper_frame");
  xformUtils.printStampedTf(gripper_stf);
}
