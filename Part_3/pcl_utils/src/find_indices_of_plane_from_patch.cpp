

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
#include <pcl/common/common_headers.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl_utils/pcl_utils.h>

using namespace std;
PclUtils *g_pcl_utils_ptr;

void find_indices_of_plane_from_patch(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr,
                                      pcl::PointCloud<pcl::PointXYZ>::Ptr patch_cloud_ptr, vector<int> &indices)
{
  float curvature;
  Eigen::Vector4f plane_parameters;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::computePointNormal(*patch_cloud_ptr, plane_parameters, curvature);
  cout << "PCL: plane params of patch: " << plane_parameters.transpose() << endl;

  Eigen::Affine3f A_plane_wrt_camera;

  A_plane_wrt_camera = g_pcl_utils_ptr->make_affine_from_plane_params(plane_parameters);
  cout << "A_plane_wrt_camera rotation:" << endl;
  cout << A_plane_wrt_camera.linear() << endl;
  cout << "origin: " << A_plane_wrt_camera.translation().transpose() << endl;

  cout << "transforming all points to plane coordinates..." << endl;

  pcl::transformPointCloud(*input_cloud_ptr, *transformed_cloud_ptr, A_plane_wrt_camera.inverse());

  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud(transformed_cloud_ptr);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(-0.02, 0.02);
  pass.filter(indices);
  cout << "number of points passing the filter = " << indices.size() << endl;
}
