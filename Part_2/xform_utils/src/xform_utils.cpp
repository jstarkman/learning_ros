#include "xform_utils/xform_utils.hpp"

geometry_msgs::msg::Pose XformUtils::transformEigenAffine3dToPose(Eigen::Affine3d e)
{
  Eigen::Vector3d Oe;
  Eigen::Matrix3d Re;
  geometry_msgs::msg::Pose pose;
  Oe = e.translation();
  Re = e.linear();

  Eigen::Quaterniond q(Re);

  pose.position.x = Oe(0);
  pose.position.y = Oe(1);
  pose.position.z = Oe(2);

  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();

  return pose;
}

Eigen::Affine3d XformUtils::transformPoseToEigenAffine3d(geometry_msgs::msg::PoseStamped stPose)
{
  Eigen::Affine3d affine;
  geometry_msgs::msg::Pose pose = stPose.pose;
  Eigen::Vector3d Oe;
  // ROS_WARN("xformUtils: input pose:");
  printPose(pose);
  Oe(0) = pose.position.x;
  Oe(1) = pose.position.y;
  Oe(2) = pose.position.z;
  affine.translation() = Oe;

  Eigen::Quaterniond q;
  q.x() = pose.orientation.x;
  q.y() = pose.orientation.y;
  q.z() = pose.orientation.z;
  q.w() = pose.orientation.w;
  Eigen::Matrix3d Re(q);

  affine.linear() = Re;
  affine.translation() = Oe;
  printAffine(affine);
  return affine;
}

Eigen::Affine3d XformUtils::transformPoseToEigenAffine3d(geometry_msgs::msg::Pose pose)
{
  Eigen::Affine3d affine;

  Eigen::Vector3d Oe;
  printPose(pose);
  Oe(0) = pose.position.x;
  Oe(1) = pose.position.y;
  Oe(2) = pose.position.z;
  affine.translation() = Oe;

  Eigen::Quaterniond q;
  q.x() = pose.orientation.x;
  q.y() = pose.orientation.y;
  q.z() = pose.orientation.z;
  q.w() = pose.orientation.w;
  Eigen::Matrix3d Re(q);

  affine.linear() = Re;
  affine.translation() = Oe;
  printAffine(affine);
  return affine;
}

Eigen::Affine3d XformUtils::transformStampedTfToEigenAffine3d(geometry_msgs::msg::TransformStamped sTf)
{
  tf2::Transform transform = get_tf_from_stamped_tf(sTf);
  Eigen::Affine3d affine = transformTFToAffine3d(transform);
  return affine;
}

Eigen::Affine3f XformUtils::transformTFToAffine3f(const tf2::Transform &t)
{
  Eigen::Affine3f e;

  for (int i = 0; i < 3; i++)
  {
    e.matrix()(i, 3) = t.getOrigin()[i];
    for (int j = 0; j < 3; j++)
    {
      e.matrix()(i, j) = t.getBasis()[i][j];
    }
  }

  for (int col = 0; col < 3; col++)
    e.matrix()(3, col) = 0;
  e.matrix()(3, 3) = 1;
  return e;
}

Eigen::Affine3d XformUtils::transformTFToAffine3d(const tf2::Transform &t)
{
  Eigen::Affine3d e;

  for (int i = 0; i < 3; i++)
  {
    e.matrix()(i, 3) = t.getOrigin()[i];
    for (int j = 0; j < 3; j++)
    {
      e.matrix()(i, j) = t.getBasis()[i][j];
    }
  }

  for (int col = 0; col < 3; col++)
    e.matrix()(3, col) = 0;
  e.matrix()(3, 3) = 1;
  return e;
}

double XformUtils::convertPlanarQuat2Phi(geometry_msgs::msg::Quaternion quaternion)
{
  double quat_z = quaternion.z;
  double quat_w = quaternion.w;
  double phi = 2.0 * atan2(quat_z, quat_w);
  return phi;
}

geometry_msgs::msg::Quaternion XformUtils::convertPlanarPsi2Quaternion(double psi)
{
  geometry_msgs::msg::Quaternion quaternion;
  quaternion.x = 0.0;
  quaternion.y = 0.0;
  quaternion.z = sin(psi / 2.0);
  quaternion.w = cos(psi / 2.0);
  return (quaternion);
}

tf2::Transform XformUtils::get_tf_from_stamped_tf(geometry_msgs::msg::TransformStamped sTf)
{
  tf2::Quaternion tfQuat(sTf.transform.rotation.x, sTf.transform.rotation.y, sTf.transform.rotation.z,
                         sTf.transform.rotation.w);
  tf2::Vector3 tfVec(sTf.transform.translation.x, sTf.transform.translation.y, sTf.transform.translation.z);
  tf2::Transform tf(tfQuat, tfVec);
  return tf;
}

geometry_msgs::msg::TransformStamped XformUtils::get_stamped_tf_from_tf(tf2::Transform tf)
{
  geometry_msgs::msg::TransformStamped sTf;
  sTf.transform.translation.x = tf.getOrigin().x();
  sTf.transform.translation.y = tf.getOrigin().y();
  sTf.transform.translation.z = tf.getOrigin().z();
  sTf.transform.rotation.x = tf.getRotation().x();
  sTf.transform.rotation.y = tf.getRotation().y();
  sTf.transform.rotation.z = tf.getRotation().z();
  sTf.transform.rotation.w = tf.getRotation().w();
  return sTf;
}

geometry_msgs::msg::PoseStamped XformUtils::get_pose_from_stamped_tf(geometry_msgs::msg::TransformStamped tf)
{
  geometry_msgs::msg::PoseStamped stPose;
  stPose.pose.orientation = tf.transform.rotation;

  geometry_msgs::msg::Vector3 tfVec = tf.transform.translation;
  geometry_msgs::msg::Point pt;
  pt.x = tfVec.x;
  pt.y = tfVec.y;
  pt.z = tfVec.z;
  stPose.pose.position = pt;
  stPose.header.frame_id = tf.header.frame_id;
  return stPose;
}

geometry_msgs::msg::TransformStamped
XformUtils::convert_poseStamped_to_stampedTransform(geometry_msgs::msg::PoseStamped stPose, std::string child_frame_id)
{
  tf2::Transform transform;
  geometry_msgs::msg::Pose pose = stPose.pose;

  geometry_msgs::msg::Point position = pose.position;
  geometry_msgs::msg::Quaternion orientation = pose.orientation;
  transform.setOrigin(tf2::Vector3(position.x, position.y, position.z));
  printStampedPose(stPose);
  transform.setRotation(tf2::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
  geometry_msgs::msg::TransformStamped stTransform;
  stTransform.header = stPose.header;
  stTransform.child_frame_id = child_frame_id;
  stTransform.transform = XformUtils::get_stamped_tf_from_tf(transform).transform;
  return stTransform;
}

void XformUtils::test_stf(geometry_msgs::msg::PoseStamped stPose)
{
  printStampedPose(stPose);
}

bool XformUtils::multiply_stamped_tfs(geometry_msgs::msg::TransformStamped A_stf,
                                      geometry_msgs::msg::TransformStamped B_stf,
                                      geometry_msgs::msg::TransformStamped &C_stf)
{
  tf2::Transform A, B, C;
  std::string str1(A_stf.child_frame_id);
  std::string str2(B_stf.header.frame_id);
  if (str1.compare(str2) != 0)
  {
    std::cout << "can't multiply transforms; mismatched frames" << std::endl;
    std::cout << str1 << " is not " << str2 << std::endl;
    return false;
  }

  A = XformUtils::get_tf_from_stamped_tf(A_stf);
  B = XformUtils::get_tf_from_stamped_tf(B_stf);
  C = A * B;
  C_stf.header.frame_id = A_stf.header.frame_id;
  C_stf.child_frame_id = B_stf.child_frame_id;

  tf2::Vector3 cVec = C.getOrigin();
  C_stf.transform.translation.x = cVec.x();
  C_stf.transform.translation.y = cVec.y();
  C_stf.transform.translation.z = cVec.z();

  tf2::Matrix3x3 cBasis = C.getBasis();
  tf2::Quaternion cQuat;
  cBasis.getRotation(cQuat);
  C_stf.transform.rotation.x = cQuat.x();
  C_stf.transform.rotation.y = cQuat.y();
  C_stf.transform.rotation.z = cQuat.z();
  C_stf.transform.rotation.w = cQuat.w();
  return true;
}

geometry_msgs::msg::TransformStamped XformUtils::stamped_transform_inverse(geometry_msgs::msg::TransformStamped stf)
{
  geometry_msgs::msg::TransformStamped stf_inv;
  tf2::Transform tf = get_tf_from_stamped_tf(stf);
  tf2::Transform tf_inv = tf.inverse();

  stf_inv.header = stf.header;
  stf_inv.header.frame_id = stf.child_frame_id;
  stf_inv.child_frame_id = stf.header.frame_id;

  tf2::Vector3 cVec = tf_inv.getOrigin();
  stf_inv.transform.translation.x = cVec.x();
  stf_inv.transform.translation.y = cVec.y();
  stf_inv.transform.translation.z = cVec.z();

  tf2::Matrix3x3 cBasis = tf_inv.getBasis();
  tf2::Quaternion cQuat;
  cBasis.getRotation(cQuat);
  stf_inv.transform.rotation.x = cQuat.x();
  stf_inv.transform.rotation.y = cQuat.y();
  stf_inv.transform.rotation.z = cQuat.z();
  stf_inv.transform.rotation.w = cQuat.w();
  return stf_inv;
}

void XformUtils::printStampedTf(geometry_msgs::msg::TransformStamped sTf)
{
  tf2::Transform tf;
  std::cout << "frame_id: " << sTf.header.frame_id << std::endl;
  std::cout << "child_frame_id: " << sTf.child_frame_id << std::endl;
  tf = get_tf_from_stamped_tf(sTf);
  printTf(tf);
}

void XformUtils::printTf(tf2::Transform tf)
{
  tf2::Vector3 tfVec;
  tf2::Matrix3x3 tfR;
  tf2::Quaternion quat;
  tfVec = tf.getOrigin();
  std::cout << "vector from reference frame to child frame: " << tfVec.getX() << "," << tfVec.getY() << ","
            << tfVec.getZ() << std::endl;
  tfR = tf.getBasis();
  std::cout << "orientation of child frame w/rt reference frame: " << std::endl;
  tfVec = tfR.getRow(0);
  std::cout << tfVec.getX() << "," << tfVec.getY() << "," << tfVec.getZ() << std::endl;
  tfVec = tfR.getRow(1);
  std::cout << tfVec.getX() << "," << tfVec.getY() << "," << tfVec.getZ() << std::endl;
  tfVec = tfR.getRow(2);
  std::cout << tfVec.getX() << "," << tfVec.getY() << "," << tfVec.getZ() << std::endl;
  quat = tf.getRotation();
  std::cout << "quaternion: " << quat.x() << ", " << quat.y() << ", " << quat.z() << ", " << quat.w() << std::endl;
}

// fnc to print out a pose:
void XformUtils::printPose(geometry_msgs::msg::Pose pose)
{
  std::cout << "origin: " << pose.position.x << ", " << pose.position.y << ", " << pose.position.z << std::endl;
  std::cout << "quaternion: " << pose.orientation.x << ", " << pose.orientation.y << ", " << pose.orientation.z << ", "
            << pose.orientation.w << std::endl;
}

void XformUtils::printStampedPose(geometry_msgs::msg::PoseStamped stPose)
{
  std::cout << "frame id = " << stPose.header.frame_id << std::endl;
  std::cout << "origin: " << stPose.pose.position.x << ", " << stPose.pose.position.y << ", " << stPose.pose.position.z
            << std::endl;
  std::cout << "quaternion: " << stPose.pose.orientation.x << ", " << stPose.pose.orientation.y << ", "
            << stPose.pose.orientation.z << ", " << stPose.pose.orientation.w << std::endl;
}

void XformUtils::printAffine(Eigen::Affine3d affine)
{
  std::cout << "origin: " << affine.translation().transpose() << std::endl;
  Eigen::Matrix3d R;
  R = affine.linear();
  // Eigen::Vector3d x_vec,y_vec,z_vec;
  // x_vec = R.col(0);
  // y_vec = R.col(1);
  // z_vec = R.col(2);
  std::cout << "\n" << R << std::endl;
}
