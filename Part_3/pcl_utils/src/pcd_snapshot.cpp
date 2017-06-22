

#include <pcl-1.7/pcl/PCLHeader.h>
#include <pcl-1.7/pcl/point_cloud.h>
#include <pcl/ros/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>

using namespace std;

bool got_kinect_image = false;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclKinect_clr_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

void kinectCB(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  if (!got_kinect_image)
  {
    ROS_INFO("got new selected kinect image");
    pcl::fromROSMsg(*cloud, *pclKinect_clr_ptr);
    ROS_INFO("image has  %d * %d points", pclKinect_clr_ptr->width, pclKinect_clr_ptr->height);
    got_kinect_image = true;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcl_snapshot_main");
  ros::NodeHandle nh;
  ros::Subscriber pointcloud_subscriber = nh.subscribe("/camera/depth_registered/points", 1, kinectCB);

  ROS_INFO("waiting for kinect data");
  while (!got_kinect_image)
  {
    ROS_INFO("waiting...");
    ros::spinOnce();
    ros::Duration(0.5).sleep();
  }
  ROS_INFO("got snapshot; saving to file kinect_snapshot.pcd");
  pcl::io::savePCDFile("kinect_snapshot.pcd", *pclKinect_clr_ptr, true);

  return 0;
}
