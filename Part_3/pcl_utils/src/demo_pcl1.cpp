

#include <math.h>
#include <ros/ros.h>
#include <stdlib.h>

#include <pcl/ros/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl-1.7/pcl/PCLHeader.h>
#include <pcl-1.7/pcl/point_cloud.h>
#include <pcl/common/common_headers.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl_utils/pcl_utils.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "plane_finder");
  ros::NodeHandle nh;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl1_ptrA(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl1_ptrB(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl1_ptrC(new pcl::PointCloud<pcl::PointXYZRGB>);

  vector<int> indices;
  string fname;
  cout << "enter pcd file name: ";
  cin >> fname;

  if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(fname, *pcl1_ptrA) == -1)
  {
    ROS_ERROR("Couldn't read file \n");
    return (-1);
  }

  cout << "starting voxel filtering" << endl;

  pcl::VoxelGrid<pcl::PointXYZRGB> vox;
  vox.setInputCloud(pcl1_ptrA);
  vox.setLeafSize(0.02f, 0.02f, 0.02f);
  vox.filter(*pcl1_ptrB);
  cout << "done voxel filtering" << endl;

  cout << "num bytes in original cloud data = " << pcl1_ptrA->points.size() << endl;
  cout << "num bytes in filtered cloud data = " << pcl1_ptrB->points.size() << endl;

  Eigen::Vector4f xyz_centroid;
  pcl::compute3DCentroid(*pcl1_ptrB, xyz_centroid);

  float curvature;
  Eigen::Vector4f plane_parameters;
  pcl::computePointNormal(*pcl1_ptrB, plane_parameters, curvature);

  Eigen::Affine3f A(Eigen::Affine3f::Identity());
  pcl::transformPointCloud(*pcl1_ptrB, *pcl1_ptrC, A);
}
