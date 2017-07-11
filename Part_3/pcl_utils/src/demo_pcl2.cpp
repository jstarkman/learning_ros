

#define _USE_MATH_DEFINES
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
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/common_headers.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl-1.7/pcl/pcl_base.h>
#include <pcl_utils/pcl_utils.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "plane_finder");
  ros::NodeHandle nh;
  pcl::PCLPointCloud2::Ptr pcl2_ptrA(new pcl::PCLPointCloud2());
  pcl::PCLPointCloud2::Ptr pcl2_ptrB(new pcl::PCLPointCloud2());
  pcl::PCLPointCloud2::Ptr pcl2_ptrC(new pcl::PCLPointCloud2());

  vector<int> indices;
  string fname;
  cout << "enter pcd file name: ";
  cin >> fname;
  pcl::PCDReader reader;
  reader.read(fname, *pcl2_ptrA);

  cout << "starting voxel filtering" << endl;

  pcl::VoxelGrid<pcl::PCLPointCloud2> vox;
  vox.setInputCloud(pcl2_ptrA);
  vox.setLeafSize(0.02f, 0.02f, 0.02f);
  vox.filter(*pcl2_ptrB);
  cout << "done voxel filtering" << endl;

  cout << "num bytes in original cloud data = " << pcl2_ptrA->data.size() << endl;
  cout << "num bytes in filtered cloud data = " << pcl2_ptrB->data.size() << endl;

  pcl::PassThrough<pcl::PointXYZ> pass;

  pcl::ExtractIndices<pcl::PCLPointCloud2> extract;

  Eigen::Vector4f xyz_centroid;

  float curvature;
  Eigen::Vector4f plane_parameters;

  Eigen::Affine3f A(Eigen::Affine3f::Identity());
}
