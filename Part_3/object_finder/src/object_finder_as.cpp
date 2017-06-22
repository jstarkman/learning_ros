

#include <actionlib/server/simple_action_server.h>
#include <object_finder/objectFinderAction.h>
#include <object_manipulation_properties/object_ID_codes.h>
#include <pcl_utils/pcl_utils.h>
#include <ros/ros.h>
#include <xform_utils/xform_utils.h>

Eigen::Affine3f g_affine_kinect_wrt_base;

class ObjectFinder
{
private:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<object_finder::objectFinderAction> object_finder_as_;

  object_finder::objectFinderGoal goal_;
  object_finder::objectFinderResult result_;
  object_finder::objectFinderFeedback feedback_;

  PclUtils pclUtils_;
  tf::TransformListener *tfListener_;

  bool find_upright_coke_can(float surface_height, geometry_msgs::PoseStamped &object_pose);
  bool find_toy_block(float surface_height, geometry_msgs::PoseStamped &object_pose);
  float find_table_height();
  double surface_height_;
  bool found_surface_height_;

public:
  ObjectFinder();

  ~ObjectFinder(void)
  {
  }

  void executeCB(const actionlib::SimpleActionServer<object_finder::objectFinderAction>::GoalConstPtr &goal);
  XformUtils xformUtils_;
};

ObjectFinder::ObjectFinder()
  : object_finder_as_(nh_, "object_finder_action_service", boost::bind(&ObjectFinder::executeCB, this, _1), false)
  , pclUtils_(&nh_)
{
  ROS_INFO("in constructor of ObjectFinder...");

  object_finder_as_.start();
  tfListener_ = new tf::TransformListener;
  found_surface_height_ = false;
}

bool ObjectFinder::find_upright_coke_can(float surface_height, geometry_msgs::PoseStamped &object_pose)
{
  bool found_object = true;
  object_pose.header.frame_id = "world";
  object_pose.pose.position.x = 0.680;
  object_pose.pose.position.y = -0.205;
  object_pose.pose.position.z = surface_height;
  object_pose.pose.orientation.x = 0;
  object_pose.pose.orientation.y = 0;
  object_pose.pose.orientation.z = 0;
  object_pose.pose.orientation.w = 1;
  return found_object;
}

bool ObjectFinder::find_toy_block(float surface_height, geometry_msgs::PoseStamped &object_pose)
{
  Eigen::Vector3f plane_normal;
  double plane_dist;

  Eigen::Vector3f major_axis;
  Eigen::Vector3f centroid;
  bool found_object = true;
  double block_height = 0.035;

  found_object = pclUtils_.find_plane_fit(0.4, 1, -0.5, 0.5, surface_height + 0.025, surface_height + 0.045, 0.001,
                                          plane_normal, plane_dist, major_axis, centroid);

  if (plane_normal(2) < 0)
    plane_normal(2) *= -1.0;
  Eigen::Matrix3f R;
  Eigen::Vector3f y_vec;
  R.col(0) = major_axis;
  R.col(2) = plane_normal;
  R.col(1) = plane_normal.cross(major_axis);
  Eigen::Quaternionf quat(R);
  object_pose.header.frame_id = "base_link";
  object_pose.pose.position.x = centroid(0);
  object_pose.pose.position.y = centroid(1);

  object_pose.pose.position.z = centroid(2) - 0.5 * block_height;

  object_pose.pose.orientation.x = quat.x();
  object_pose.pose.orientation.y = quat.y();
  object_pose.pose.orientation.z = quat.z();
  object_pose.pose.orientation.w = quat.w();
  return found_object;
}

float ObjectFinder::find_table_height()
{
  int npts_plane_max = 0;
  int npts_slab;
  double z_eps = 0.005;
  double table_height = 0.0;
  vector<int> indices;
  for (double plane_height = 0.6; plane_height < 1.2; plane_height += z_eps)
  {
    pclUtils_.find_coplanar_pts_z_height(plane_height, z_eps, indices);
    npts_slab = (int)indices.size();
    ROS_INFO("height %f has npts  = %d", plane_height, npts_slab);
    if (npts_slab > npts_plane_max)
    {
      npts_plane_max = npts_slab;
      table_height = plane_height;
    }
  }
  return (table_height);
}

void ObjectFinder::executeCB(const actionlib::SimpleActionServer<object_finder::objectFinderAction>::GoalConstPtr &goal)
{
  int object_id = goal->object_id;
  geometry_msgs::PoseStamped object_pose;
  bool known_surface_ht = goal->known_surface_ht;
  float surface_height;
  if (known_surface_ht)
  {
    surface_height = goal->surface_ht;
  }
  bool found_object = false;

  pclUtils_.reset_got_kinect_cloud();
  while (!pclUtils_.got_kinect_cloud())
  {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
    ROS_INFO("waiting for snapshot...");
  }

  ROS_INFO("transforming point cloud");
  pclUtils_.transform_kinect_cloud(g_affine_kinect_wrt_base);

  if (!known_surface_ht)
  {
    ros::Time tstart = ros::Time::now();
    double table_ht;

    table_ht = pclUtils_.find_table_height(0.0, 1, -0.5, 0.5, 0.6, 1.2, 0.005);
    ROS_INFO("table ht: %f", table_ht);
    ros::Time t3 = ros::Time::now();
    surface_height_ = table_ht;
    found_surface_height_ = true;
  }
  result_.object_id = goal->object_id;

  switch (object_id)
  {
    case ObjectIdCodes::COKE_CAN_UPRIGHT:

      found_object = find_upright_coke_can(surface_height, object_pose);
      if (found_object)
      {
        ROS_INFO("found upright Coke can!");
        result_.found_object_code = object_finder::objectFinderResult::OBJECT_FOUND;
        result_.object_pose = object_pose;
        object_finder_as_.setSucceeded(result_);
      }
      else
      {
        ROS_WARN("could not find requested object");
        object_finder_as_.setAborted(result_);
      }
      break;
    case ObjectIdCodes::TOY_BLOCK_ID:

      found_object = find_toy_block(surface_height, object_pose);
      if (found_object)
      {
        ROS_INFO("found toy block!");
        result_.found_object_code = object_finder::objectFinderResult::OBJECT_FOUND;
        result_.object_pose = object_pose;
        object_finder_as_.setSucceeded(result_);
      }
      else
      {
        ROS_WARN("could not find requested object");
        result_.found_object_code = object_finder::objectFinderResult::OBJECT_NOT_FOUND;
        object_finder_as_.setAborted(result_);
      }
      break;
    case ObjectIdCodes::TABLE_SURFACE:

      ROS_INFO("object finder: finding/returning table height");
      result_.found_object_code = object_finder::objectFinderResult::OBJECT_FOUND;

      object_pose.header.frame_id = "base_link";
      object_pose.pose.position.x = 0.5;
      object_pose.pose.position.y = 0.0;
      object_pose.pose.position.z = surface_height_;
      object_pose.pose.orientation.x = 0;
      object_pose.pose.orientation.y = 0;
      object_pose.pose.orientation.z = 0;
      object_pose.pose.orientation.w = 1;
      result_.object_pose = object_pose;
      ROS_INFO("returning height %f", surface_height_);
      result_.found_object_code = object_finder::objectFinderResult::OBJECT_FOUND;
      object_finder_as_.setSucceeded(result_);
      break;
    default:
      ROS_WARN("this object ID is not implemented");
      result_.found_object_code = object_finder::objectFinderResult::OBJECT_CODE_NOT_RECOGNIZED;
      object_finder_as_.setAborted(result_);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_finder_node");

  ROS_INFO("instantiating the object finder action server: ");

  ObjectFinder object_finder_as;
  tf::TransformListener tfListener;
  ROS_INFO("listening for kinect-to-base transform:");
  tf::StampedTransform stf_kinect_wrt_base;
  bool tferr = true;
  ROS_INFO("waiting for tf between kinect_pc_frame and base_link...");
  while (tferr)
  {
    tferr = false;
    try
    {
      tfListener.lookupTransform("base_link", "kinect_pc_frame", ros::Time(0), stf_kinect_wrt_base);
    }
    catch (tf::TransformException &exception)
    {
      ROS_WARN("%s; retrying...", exception.what());
      tferr = true;
      ros::Duration(0.5).sleep();
      ros::spinOnce();
    }
  }
  ROS_INFO("kinect to base_link tf is good");
  object_finder_as.xformUtils_.printStampedTf(stf_kinect_wrt_base);
  tf::Transform tf_kinect_wrt_base = object_finder_as.xformUtils_.get_tf_from_stamped_tf(stf_kinect_wrt_base);
  g_affine_kinect_wrt_base = object_finder_as.xformUtils_.transformTFToAffine3f(tf_kinect_wrt_base);
  cout << "affine rotation: " << endl;
  cout << g_affine_kinect_wrt_base.linear() << endl;
  cout << "affine offset: " << g_affine_kinect_wrt_base.translation().transpose() << endl;

  ROS_INFO("going into spin");

  while (ros::ok())
  {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }

  return 0;
}
