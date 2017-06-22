

#include <math.h>
#include <ros/ros.h>
#include <stdlib.h>

#include <pcl/ros/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl-1.7/pcl/PCLHeader.h>
#include <pcl-1.7/pcl/point_cloud.h>

using namespace std;

extern void make_clouds(pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr,
                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ellipse");
  ros::NodeHandle nh;

  pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_clr_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

  cout << "Generating example point-cloud ellipse.\n\n";
  cout << "view in rviz; choose: topic= ellipse; and fixed frame= camera" << endl;

  make_clouds(basic_cloud_ptr, point_cloud_clr_ptr);
  pcl::io::savePCDFileASCII("ellipse.pcd", *point_cloud_clr_ptr);

  sensor_msgs::PointCloud2 ros_cloud;

  pcl::toROSMsg(*point_cloud_clr_ptr, ros_cloud);

  ros::Publisher pubCloud = nh.advertise<sensor_msgs::PointCloud2>("/ellipse", 1);

  while (ros::ok())
  {
    pubCloud.publish(ros_cloud);
    ros::Duration(0.5).sleep();
  }
  return 0;
}
