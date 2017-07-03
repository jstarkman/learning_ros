#include "object_manipulation_properties/object_manipulation_properties.hpp"

using namespace std;

ObjectManipulationProperties::ObjectManipulationProperties(void)
{
  ROS_INFO("in constructor of objectManipulationProperties");
}

bool ObjectManipulationProperties::get_object_info(int object_id, Eigen::Affine3d &grasp_transform,
                                                   double &approach_dist, double &gripper_close_test)
{
  // bool got_info = false;
  Eigen::Vector3d object_origin_wrt_gripper_frame;

  Eigen::Matrix3d R_object_wrt_gripper;
  Eigen::Matrix3d R_gripper;
  Eigen::Vector3d x_axis, y_axis, z_axis;
  Eigen::Vector3d origin_object_wrt_gripper;
  geometry_msgs::msg::Pose object_pose_wrt_gripper;
  switch (object_id)
  {
    case ObjectIdCodes::TOY_BLOCK_ID:

      approach_dist = 0.05;
      gripper_close_test = 83.0;

      origin_object_wrt_gripper << 0, 0, 0;
      x_axis << 1, 0, 0;
      z_axis << 0, 0, -1;
      y_axis = z_axis.cross(x_axis);

      R_gripper.col(0) = x_axis;
      R_gripper.col(1) = y_axis;
      R_gripper.col(2) = z_axis;

      grasp_transform.linear() = R_gripper;
      grasp_transform.translation() = origin_object_wrt_gripper;
      object_pose_wrt_gripper = xformUtils.transformEigenAffine3dToPose(grasp_transform);
      ROS_INFO("object pose w/rt gripper: ");
      cout << "R_gripper: " << endl;
      cout << R_gripper << endl;
      xformUtils.printPose(object_pose_wrt_gripper);

      R_gripper.col(0) = -x_axis;
      R_gripper.col(1) = z_axis.cross(-x_axis);
      grasp_transform.linear() = R_gripper;

      object_pose_wrt_gripper = xformUtils.transformEigenAffine3dToPose(grasp_transform);
      ROS_INFO("object pose w/rt gripper, x-axis antiparallel: ");
      cout << "R_gripper: " << endl;
      cout << R_gripper << endl;
      xformUtils.printPose(object_pose_wrt_gripper);
      return true;
      break;

    default:
      ROS_WARN("object ID not recognized");
      return false;
      break;
  }
}
