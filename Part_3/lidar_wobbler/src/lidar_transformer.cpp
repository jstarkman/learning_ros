

#include <math.h>
#include <ros/ros.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <xform_utils/xform_utils.h>
using namespace std;

tf::TransformListener* g_listener_ptr;
XformUtils xformUtils;
vector<Eigen::Vector3d> g_pt_vecs_wrt_lidar_frame;
vector<Eigen::Vector3d> g_pt_vecs_wrt_world_frame;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
  tf::StampedTransform stfLidar2World;

  g_listener_ptr->lookupTransform("world", "lidar_link", ros::Time(0), stfLidar2World);

  tf::Transform tf = xformUtils.get_tf_from_stamped_tf(stfLidar2World);

  Eigen::Affine3d affine_tf, affine_tf_inv;

  affine_tf = xformUtils.transformTFToAffine3d(tf);
  affine_tf_inv = affine_tf.inverse();
  vector<float> ranges = scan_in->ranges;
  int npts = ranges.size();
  g_pt_vecs_wrt_lidar_frame.clear();
  g_pt_vecs_wrt_world_frame.clear();

  ROS_INFO("received %d ranges: ", npts);
  double start_ang = scan_in->angle_min;
  double end_ang = scan_in->angle_max;
  double d_ang = (end_ang - start_ang) / (npts - 1);
  ROS_INFO("d_ang = %f", d_ang);
  Eigen::Vector3d vec;
  vec[2] = 0.0;

  double ang;
  for (int i = 0; i < npts; i++)
  {
    if (ranges[i] < 5.0)
    {
      ang = start_ang + i * d_ang;
      vec[0] = ranges[i] * cos(ang);
      vec[1] = ranges[i] * sin(ang);
      g_pt_vecs_wrt_lidar_frame.push_back(vec);
    }
  }
  int npts3d = g_pt_vecs_wrt_lidar_frame.size();
  ROS_INFO("computed %d 3-D pts w/rt LIDAR frame", npts3d);
  g_pt_vecs_wrt_world_frame.resize(npts3d);

  for (int i = 0; i < npts3d; i++)
  {
    g_pt_vecs_wrt_world_frame[i] = affine_tf * g_pt_vecs_wrt_lidar_frame[i];
  }

  for (int i = 0; i < npts3d; i++)
  {
    vec = g_pt_vecs_wrt_world_frame[i];
    if (vec[2] < 0.1)
    {
      ROS_INFO("(x,y,z) = (%6.3f, %6.3f, %6.3f)", vec[0], vec[1], vec[2]);
    }
    else
    {
      ROS_WARN("(x,y,z) = (%6.3f, %6.3f, %6.3f)", vec[0], vec[1], vec[2]);
    }
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lidar_wobbler_transformer");
  ros::NodeHandle nh;

  g_listener_ptr = new tf::TransformListener;
  tf::StampedTransform stfLidar2World;
  bool tferr = true;
  ROS_INFO("trying to get tf of lidar_link w/rt world: ");

  while (tferr)
  {
    tferr = false;
    try
    {
      g_listener_ptr->lookupTransform("world", "lidar_link", ros::Time(0), stfLidar2World);
    }
    catch (tf::TransformException& exception)
    {
      ROS_WARN("%s; retrying...", exception.what());
      tferr = true;
      ros::Duration(0.5).sleep();
      ros::spinOnce();
    }
  }
  ROS_INFO("transform received; ready to process lidar scans");
  ros::Subscriber lidar_subscriber = nh.subscribe("/scan", 1, scanCallback);
  ros::spin();

  return 0;
}
