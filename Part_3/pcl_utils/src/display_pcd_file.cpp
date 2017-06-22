

#include <math.h>
#include <pcl-1.7/pcl/PCLHeader.h>
#include <pcl-1.7/pcl/point_cloud.h>
#include <pcl/common/common_headers.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <stdlib.h>

using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcd_publisher");
  ros::NodeHandle nh;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_clr_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

  cout << "enter pcd file name: ";
  string fname;
  cin >> fname;

  if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(fname, *pcl_clr_ptr) == -1)
  {
    ROS_ERROR("Couldn't read file \n");
    return (-1);
  }
  std::cout << "Loaded " << pcl_clr_ptr->width * pcl_clr_ptr->height << " data points from file " << fname << std::endl;

  ros::Publisher pubCloud = nh.advertise<sensor_msgs::PointCloud2>("/pcd", 1);
  sensor_msgs::PointCloud2 ros_cloud;
  pcl::toROSMsg(*pcl_clr_ptr, ros_cloud);
  ros_cloud.header.frame_id = "camera_depth_optical_frame";
  cout << "view in rviz; choose: topic= pcd; and fixed frame= camera_depth_optical_frame" << endl;

  while (ros::ok())
  {
    pubCloud.publish(ros_cloud);
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }
  return 0;
}
