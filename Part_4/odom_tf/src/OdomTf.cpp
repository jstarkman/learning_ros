#include "odom_tf/odom_tf.hpp"

OdomTf::OdomTf(rclcpp::node::Node::SharedPtr node) : node_(node)
{
  ROS_INFO("in class constructor of DemoTfListener");
  tfBuffer_ = new tf2_ros::Buffer;
  tfBr_ = new tf2_ros::TransformBroadcaster;
  tfListener_ = new tf2_ros::TransformListener(*tfBuffer_);

  odom_tf_ready_ = false;
  odom_ready_ = false;
  amcl_ready_ = false;

  pose_publisher_ =
      node->create_publisher<geometry_msgs::msg::PoseStamped>("triad_display_pose", rmw_qos_profile_default);

  tf2::TimePoint tp;
  tf2::Duration tDuration(10000000000);
  ROS_INFO("waiting for tf between base_link and odom...");

  bool tferr = true;
  while (tferr)
  {
    tferr = false;
    try
    {
      stfBaseLinkWrtOdom_ = tfBuffer_->lookupTransform("odom", "base_link", tp, tDuration);
    }
    catch (tf2::TimeoutException& exception)
    {
      ROS_WARN("%s; retrying...", exception.what());
      tferr = true;
      rclcpp::WallRate(0.5).sleep();
      rclcpp::spin_some(node);
    }
  }

  tferr = true;
  ROS_INFO("waiting for tf between odom and map...");
  while (tferr)
  {
    tferr = false;
    try
    {
      stfOdomWrtMap_ = tfBuffer_->lookupTransform("map", "odom", tp, tDuration);
    }
    catch (tf2::TimeoutException& exception)
    {
      ROS_WARN("%s; retrying...", exception.what());
      tferr = true;
      rclcpp::WallRate(0.5).sleep();
      rclcpp::spin_some(node);
    }
  }
  ROS_INFO("map to odom tf is good");

  builtin_interfaces::msg::Time t0;
  t0.sec = (double)std::chrono::system_clock::now().time_since_epoch().count();
  stfAmclBaseLinkWrtMap_.header.stamp = t0;
  stfAmclBaseLinkWrtMap_.transform.translation.x = 0;
  stfAmclBaseLinkWrtMap_.transform.translation.y = 0;
  stfAmclBaseLinkWrtMap_.transform.translation.z = 0;
  stfAmclBaseLinkWrtMap_.transform.rotation.x = 0;
  stfAmclBaseLinkWrtMap_.transform.rotation.y = 0;
  stfAmclBaseLinkWrtMap_.transform.rotation.z = 0;
  stfAmclBaseLinkWrtMap_.transform.rotation.w = 1;
  stfAmclBaseLinkWrtMap_.header.frame_id = "map";
  stfAmclBaseLinkWrtMap_.child_frame_id = "base_link";

  std::cout << std::endl << "init stfAmclBaseLinkWrtMap_" << std::endl;
  xform_utils.printStampedTf(stfAmclBaseLinkWrtMap_);

  stfDriftyOdomWrtMap_ = stfAmclBaseLinkWrtMap_;
  stfDriftyOdomWrtMap_.header.frame_id = "map";
  stfDriftyOdomWrtMap_.child_frame_id = "drifty_odom";

  initializeSubscribers();

  odom_count_ = 0;
  odom_phi_ = 1000.0;
  ROS_INFO("waiting for valid odom message...");
  while (odom_phi_ > 500.0)
  {
    rclcpp::WallRate(2).sleep();
    std::cout << ".";
    rclcpp::spin_some(node);
  }
  ROS_INFO("stfBaseLinkWrtDriftyOdom_");
  xform_utils.printStampedTf(stfBaseLinkWrtDriftyOdom_);
  ROS_WARN("waiting for amcl publication...");
  while (!amcl_ready_)
  {
    rclcpp::WallRate(10).sleep();
    std::cout << ",";
    rclcpp::spin_some(node);
  }
  ROS_INFO("got amcl callback; stfAmclBaseLinkWrtMap_:");
  xform_utils.printStampedTf(stfAmclBaseLinkWrtMap_);

  ROS_INFO("constructor: got an odom message; ready to roll");
  odom_tf_ready_ = true;
}

void OdomTf::initializeSubscribers()
{
  ROS_INFO("Initializing Subscribers");
  odom_subscriber_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      "/drifty_odom",
      [this](const nav_msgs::msg::Odometry::SharedPtr odom_rcvd) {
        odom_count_++;

        // double odom_x_ = odom_rcvd->pose.pose.position.x;
        // double odom_y_ = odom_rcvd->pose.pose.position.y;
        stfBaseLinkWrtDriftyOdom_ = xform_utils.convert_pose_to_stampedTransform(odom_rcvd->pose.pose, "base_link");
        stfBaseLinkWrtDriftyOdom_.header.frame_id = "drifty_odom";

        odom_phi_ = xform_utils.convertPlanarQuat2Phi(stfBaseLinkWrtDriftyOdom_.transform.rotation);

        stfDriftyOdomWrtBase_ = xform_utils.stamped_transform_inverse(stfBaseLinkWrtDriftyOdom_);

        tfBr_->sendTransform(stfDriftyOdomWrtBase_);

        xform_utils.multiply_stamped_tfs(stfDriftyOdomWrtMap_, stfBaseLinkWrtDriftyOdom_, stfEstBaseWrtMap_);

        stfEstBaseWrtMap_.child_frame_id = "est_base";

        tfBr_->sendTransform(stfEstBaseWrtMap_);
        estBasePoseWrtMap_ = xform_utils.get_pose_from_stamped_tf(stfEstBaseWrtMap_);
        pose_publisher_->publish(estBasePoseWrtMap_);  // send this to triad marker display node
      },
      rmw_qos_profile_default);

  amcl_subscriber_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/amcl_pose",
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr amcl_rcvd) {
        amcl_pose_ = amcl_rcvd->pose.pose;
        ROS_WARN("amcl pose: x, y, yaw: %f, %f, %f\n", amcl_pose_.position.x, amcl_pose_.position.y,
                 xform_utils.convertPlanarQuat2Phi(amcl_pose_.orientation));

        stfAmclBaseLinkWrtMap_ = xform_utils.convert_pose_to_stampedTransform(amcl_rcvd->pose.pose, "base_link");
        stfAmclBaseLinkWrtMap_.header.frame_id = "map";

        odom_phi_ = xform_utils.convertPlanarQuat2Phi(stfAmclBaseLinkWrtMap_.transform.rotation);

        stfDriftyOdomWrtBase_ = xform_utils.stamped_transform_inverse(stfBaseLinkWrtDriftyOdom_);

        if (!xform_utils.multiply_stamped_tfs(stfAmclBaseLinkWrtMap_, stfDriftyOdomWrtBase_, stfDriftyOdomWrtMap_))
        {
          ROS_WARN("stfAmclBaseLinkWrtMap_,stfDriftyOdomWrtBase_ multiply invalid");
        }

        stfDriftyOdomWrtMap_.child_frame_id = "corrected_odom";
        tfBr_->sendTransform(stfDriftyOdomWrtMap_);
        stfDriftyOdomWrtMap_.child_frame_id = "drifty_odom";

        stfAmclBaseLinkWrtMap_.child_frame_id = "amcl_base_link";
        tfBr_->sendTransform(stfAmclBaseLinkWrtMap_);

        amcl_ready_ = true;
      },
      rmw_qos_profile_default);
}
