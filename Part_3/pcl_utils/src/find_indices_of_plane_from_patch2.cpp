

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

#include <pcl-1.7/pcl/common/centroid.h>
#include <pcl_utils/pcl_utils.h>

using namespace std;
PclUtils *g_pcl_utils_ptr;

void find_indices_of_plane_from_patch(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr,
                                      pcl::PointCloud<pcl::PointXYZ>::Ptr patch_cloud_ptr, vector<int> &indices)
{
  float curvature;
  Eigen::Vector4f plane_parameters;
  Eigen::Vector3f patch_centroid;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::computePointNormal(*patch_cloud_ptr, plane_parameters, curvature);
  cout << "PCL: plane params of patch: " << plane_parameters.transpose() << endl;
  Eigen::Vector4f xyz_centroid4f;
  pcl::compute3DCentroid(*patch_cloud_ptr, xyz_centroid4f);
  Eigen::Vector3f xyz_centroid3f;
  for (int i = 0; i < 3; i++)
    xyz_centroid3f(i) = xyz_centroid4f(i);

  cout << "pcl: centroid = " << xyz_centroid4f.transpose() << endl;

  Eigen::Affine3f A_plane_wrt_camera;

  A_plane_wrt_camera = g_pcl_utils_ptr->make_affine_from_plane_params(plane_parameters, xyz_centroid3f);
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

void box_filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud, Eigen::Vector3f pt_min, Eigen::Vector3f pt_max,
                vector<int> &indices)
{
  int npts = inputCloud->points.size();
  Eigen::Vector3f pt;
  indices.clear();
  cout << "box min: " << pt_min.transpose() << endl;
  cout << "box max: " << pt_max.transpose() << endl;
  ROS_INFO("box filtering %d points", npts);
  for (int i = 0; i < npts; ++i)
  {
    pt = inputCloud->points[i].getVector3fMap();

    if ((pt[0] > pt_min[0]) && (pt[0] < pt_max[0]) && (pt[1] > pt_min[1]) && (pt[1] < pt_max[1]) &&
        (pt[2] > pt_min[2]) && (pt[2] < pt_max[2]))
    {
      indices.push_back(i);
    }
  }
  int n_extracted = indices.size();
  cout << " number of points within box = " << n_extracted << endl;
}

Eigen::Affine3f compute_plane_affine_from_patch(pcl::PointCloud<pcl::PointXYZ>::Ptr patch_cloud_ptr)
{
  float curvature;
  Eigen::Vector4f plane_parameters;
  Eigen::Vector3f patch_centroid;
  Eigen::Affine3f A_plane_wrt_camera;

  pcl::computePointNormal(*patch_cloud_ptr, plane_parameters, curvature);
  cout << "PCL: plane params of patch: " << plane_parameters.transpose() << endl;
  Eigen::Vector4f xyz_centroid4f;
  pcl::compute3DCentroid(*patch_cloud_ptr, xyz_centroid4f);
  Eigen::Vector3f xyz_centroid3f;
  for (int i = 0; i < 3; i++)
    xyz_centroid3f(i) = xyz_centroid4f(i);

  cout << "pcl: centroid = " << xyz_centroid4f.transpose() << endl;

  A_plane_wrt_camera = g_pcl_utils_ptr->make_affine_from_plane_params(plane_parameters, xyz_centroid3f);
  cout << "A_plane_wrt_camera rotation:" << endl;
  cout << A_plane_wrt_camera.linear() << endl;
  cout << "origin: " << A_plane_wrt_camera.translation().transpose() << endl;
  return A_plane_wrt_camera;
}

void find_indices_box_filtered_from_patch(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr,
                                          pcl::PointCloud<pcl::PointXYZ>::Ptr patch_cloud_ptr,
                                          Eigen::Vector3f box_pt_min, Eigen::Vector3f box_pt_max, vector<int> &indices)
{
  float curvature;
  Eigen::Vector4f plane_parameters;
  Eigen::Vector3f patch_centroid;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::computePointNormal(*patch_cloud_ptr, plane_parameters, curvature);
  cout << "PCL: plane params of patch: " << plane_parameters.transpose() << endl;
  Eigen::Vector4f xyz_centroid4f;
  pcl::compute3DCentroid(*patch_cloud_ptr, xyz_centroid4f);
  Eigen::Vector3f xyz_centroid3f;
  for (int i = 0; i < 3; i++)
    xyz_centroid3f(i) = xyz_centroid4f(i);

  cout << "pcl: centroid = " << xyz_centroid4f.transpose() << endl;

  Eigen::Affine3f A_plane_wrt_camera;

  A_plane_wrt_camera = g_pcl_utils_ptr->make_affine_from_plane_params(plane_parameters, xyz_centroid3f);
  cout << "A_plane_wrt_camera rotation:" << endl;
  cout << A_plane_wrt_camera.linear() << endl;
  cout << "origin: " << A_plane_wrt_camera.translation().transpose() << endl;

  cout << "transforming all points to plane coordinates..." << endl;

  pcl::transformPointCloud(*input_cloud_ptr, *transformed_cloud_ptr, A_plane_wrt_camera.inverse());
  box_filter(input_cloud_ptr, box_pt_min, box_pt_max, indices);

  pcl::PassThrough<pcl::PointXYZRGB> pass;
  /*
  pass.setInputCloud(transformed_cloud_ptr);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(-0.02, 0.02);
  pass.filter(indices);
  cout << "number of points passing the filter = " << indices.size() << endl;
   * */
}

void transform_points_to_plane_frame(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr,
                                     Eigen::Affine3f A_plane_wrt_camera,
                                     pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud_ptr)
{
  pcl::transformPointCloud(*input_cloud_ptr, *transformed_cloud_ptr, A_plane_wrt_camera.inverse());
}

void find_indices_box_filtered(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr, Eigen::Vector3f box_pt_min,
                               Eigen::Vector3f box_pt_max, vector<int> &indices)
{
  int npts = input_cloud_ptr->points.size();
  Eigen::Vector3f pt;
  indices.clear();
  cout << "box min: " << box_pt_min.transpose() << endl;
  cout << "box max: " << box_pt_max.transpose() << endl;
  for (int i = 0; i < npts; ++i)
  {
    pt = input_cloud_ptr->points[i].getVector3fMap();

    if ((pt[0] > box_pt_min[0]) && (pt[0] < box_pt_max[0]) && (pt[1] > box_pt_min[1]) && (pt[1] < box_pt_max[1]) &&
        (pt[2] > box_pt_min[2]) && (pt[2] < box_pt_max[2]))
    {
      indices.push_back(i);
    }
  }
  int n_extracted = indices.size();
  cout << " number of points within box = " << n_extracted << endl;
}
