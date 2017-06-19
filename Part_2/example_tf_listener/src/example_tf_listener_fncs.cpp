#include "example_tf_listener.hpp"

DemoTfListener::DemoTfListener(rclcpp::node::Node::SharedPtr node) : node(node)
{
  ROS_INFO("in class constructor of DemoTfListener");
  tfListener_ = new tf2_ros::TransformListener(*tfBuffer_);

  bool tferr = true;
  ROS_INFO("waiting for tf between link2 and base_link...");
  geometry_msgs::msg::TransformStamped tfLink2WrtBaseLink;
  tf2::TimePoint t0;
  tf2::Duration tDuration(10000000000);
  while (tferr)
  {
    tferr = false;
    try
    {
      tfLink2WrtBaseLink = tfBuffer_->lookupTransform("base_link", "link2", t0, tDuration);
    }
    catch (tf2::TimeoutException& exception)
    {
      ROS_WARN("%s; retrying...", exception.what());
      tferr = true;
      rclcpp::WallRate(0.5).sleep();
      rclcpp::spin_some(node);
    }
  }
  ROS_INFO("tf is good");
}

tf2::Transform DemoTfListener::get_tf_from_stamped_tf(geometry_msgs::msg::TransformStamped sTf)
{
  tf2::Quaternion tfQuat(sTf.transform.rotation.x, sTf.transform.rotation.y, sTf.transform.rotation.z,
                         sTf.transform.rotation.w);
  tf2::Vector3 tfVec(sTf.transform.translation.x, sTf.transform.translation.y, sTf.transform.translation.z);
  tf2::Transform tf(tfQuat, tfVec);
  return tf;
}

geometry_msgs::msg::PoseStamped DemoTfListener::get_pose_from_transform(geometry_msgs::msg::TransformStamped tf)
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

bool DemoTfListener::multiply_stamped_tfs(geometry_msgs::msg::TransformStamped A_stf,
                                          geometry_msgs::msg::TransformStamped B_stf,
                                          geometry_msgs::msg::TransformStamped& C_stf)
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

  A = get_tf_from_stamped_tf(A_stf);
  B = get_tf_from_stamped_tf(B_stf);
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

void DemoTfListener::printTf(tf2::Transform tf)
{
  tf2::Vector3 tfVec;
  tf2::Matrix3x3 tfR;
  tf2::Quaternion quat;
  tfVec = tf.getOrigin();
  std::cout << "vector from reference frame to to child frame: " << tfVec.getX() << "," << tfVec.getY() << ","
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

void DemoTfListener::printStampedTf(geometry_msgs::msg::TransformStamped sTf)
{
  tf2::Transform tf;
  std::cout << "frame_id: " << sTf.header.frame_id << std::endl;
  std::cout << "child_frame_id: " << sTf.child_frame_id << std::endl;
  tf = get_tf_from_stamped_tf(sTf);
  printTf(tf);
}

void DemoTfListener::printStampedPose(geometry_msgs::msg::PoseStamped stPose)
{
  std::cout << "frame id = " << stPose.header.frame_id << std::endl;
  std::cout << "origin: " << stPose.pose.position.x << ", " << stPose.pose.position.y << ", " << stPose.pose.position.z
            << std::endl;
  std::cout << "quaternion: " << stPose.pose.orientation.x << ", " << stPose.pose.orientation.y << ", "
            << stPose.pose.orientation.z << ", " << stPose.pose.orientation.w << std::endl;
}
