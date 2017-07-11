

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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "plane_finder");
  ros::NodeHandle nh;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclKinect_clr_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_pts_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr selected_pts_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_kinect_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

  vector<int> indices;

  string fname;
  cout << "enter pcd file name: ";
  cin >> fname;
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(fname, *pclKinect_clr_ptr) == -1)
  {
    ROS_ERROR("Couldn't read file \n");
    return (-1);
  }

  pclKinect_clr_ptr->header.frame_id = "camera_depth_optical_frame";

  ros::Publisher pubCloud = nh.advertise<sensor_msgs::PointCloud2>("/pcd", 1);
  ros::Publisher pubPlane = nh.advertise<sensor_msgs::PointCloud2>("planar_pts", 1);
  ros::Publisher pubDnSamp = nh.advertise<sensor_msgs::PointCloud2>("downsampled_pcd", 1);

  sensor_msgs::PointCloud2 ros_cloud, ros_planar_cloud, downsampled_cloud;
  pcl::toROSMsg(*pclKinect_clr_ptr, ros_cloud);

  cout << "starting voxel filtering" << endl;
  pcl::VoxelGrid<pcl::PointXYZRGB> vox;
  vox.setInputCloud(pclKinect_clr_ptr);

  vox.setLeafSize(0.02f, 0.02f, 0.02f);
  vox.filter(*downsampled_kinect_ptr);
  cout << "done voxel filtering" << endl;

  cout << "num bytes in original cloud data = " << pclKinect_clr_ptr->points.size() << endl;
  cout << "num bytes in filtered cloud data = " << downsampled_kinect_ptr->points.size() << endl;
  pcl::toROSMsg(*downsampled_kinect_ptr, downsampled_cloud);

  PclUtils pclUtils(&nh);
  g_pcl_utils_ptr = &pclUtils;

  cout << " select a patch of points to find corresponding plane..." << endl;

  while (ros::ok())
  {
    if (pclUtils.got_selected_points())
    {
      pclUtils.reset_got_selected_points();
      pclUtils.get_copy_selected_points(selected_pts_cloud_ptr);
      cout << "got new patch with number of selected pts = " << selected_pts_cloud_ptr->points.size() << endl;

      find_indices_of_plane_from_patch(downsampled_kinect_ptr, selected_pts_cloud_ptr, indices);
      pcl::copyPointCloud(*downsampled_kinect_ptr, indices, *plane_pts_ptr);

      pcl::toROSMsg(*plane_pts_ptr, ros_planar_cloud);
    }
    pubCloud.publish(ros_cloud);
    pubPlane.publish(ros_planar_cloud);
    pubDnSamp.publish(downsampled_cloud);
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }

  return 0;
}
