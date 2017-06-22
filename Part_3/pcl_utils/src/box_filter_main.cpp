

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
extern PclUtils *g_pcl_utils_ptr;

extern void find_indices_of_plane_from_patch(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr,
                                             pcl::PointCloud<pcl::PointXYZ>::Ptr patch_cloud_ptr, vector<int> &indices);

extern void find_indices_box_filtered_from_patch(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr,
                                                 pcl::PointCloud<pcl::PointXYZ>::Ptr patch_cloud_ptr,
                                                 Eigen::Vector3f box_pt_min, Eigen::Vector3f box_pt_max,
                                                 vector<int> &indices);

extern Eigen::Affine3f compute_plane_affine_from_patch(pcl::PointCloud<pcl::PointXYZ>::Ptr patch_cloud_ptr);

extern void find_indices_box_filtered(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr,
                                      Eigen::Vector3f box_pt_min, Eigen::Vector3f box_pt_max, vector<int> &indices);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "plane_finder");
  ros::NodeHandle nh;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclKinect_clr_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_pts_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr selected_pts_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_kinect_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr box_filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  vector<int> indices;
  Eigen::Affine3f A_plane_wrt_camera;

  string fname;
  cout << "enter pcd file name: ";
  cin >> fname;
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(fname, *pclKinect_clr_ptr) == -1)
  {
    ROS_ERROR("Couldn't read file \n");
    return (-1);
  }

  pclKinect_clr_ptr->header.frame_id = "camera_depth_optical_frame";
  ROS_INFO("view frame camera_depth_optical_frame on topics pcd, planar_pts and downsampled_pcd");

  ros::Publisher pubCloud = nh.advertise<sensor_msgs::PointCloud2>("/pcd", 1);
  ros::Publisher pubPlane = nh.advertise<sensor_msgs::PointCloud2>("planar_pts", 1);
  ros::Publisher pubDnSamp = nh.advertise<sensor_msgs::PointCloud2>("downsampled_pcd", 1);
  ros::Publisher pubBoxFilt = nh.advertise<sensor_msgs::PointCloud2>("box_filtered_pcd", 1);
  sensor_msgs::PointCloud2 ros_cloud, ros_planar_cloud, downsampled_cloud, ros_box_filtered_cloud;
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

  while (!pclUtils.got_selected_points())
  {
    pubCloud.publish(ros_cloud);
    ros::spinOnce();
    ros::Duration(0.3).sleep();
  }
  pclUtils.get_copy_selected_points(selected_pts_cloud_ptr);
  cout << "got new patch with number of selected pts = " << selected_pts_cloud_ptr->points.size() << endl;

  A_plane_wrt_camera = compute_plane_affine_from_patch(selected_pts_cloud_ptr);

  pcl::transformPointCloud(*pclKinect_clr_ptr, *transformed_cloud_ptr, A_plane_wrt_camera.inverse());

  Eigen::Vector3f box_pt_min, box_pt_max;
  box_pt_min << 0.5, -0.5, 0.02;
  box_pt_max << 2.0, 0.5, 0.5;
  int dim;

  while (ros::ok())
  {
    cout << "enter dim (0,1,2): ";
    cin >> dim;
    if ((dim >= 0) && (dim < 3))
    {
      cout << "enter min val: ";
      cin >> box_pt_min(dim);
      cout << "enter max val: ";
      cin >> box_pt_max(dim);
    }
    else
    {
      ROS_WARN("index out of range");
    }

    find_indices_box_filtered(transformed_cloud_ptr, box_pt_min, box_pt_max, indices);
    pcl::copyPointCloud(*pclKinect_clr_ptr, indices, *box_filtered_cloud_ptr);

    pcl::toROSMsg(*box_filtered_cloud_ptr, ros_box_filtered_cloud);
    pubBoxFilt.publish(ros_box_filtered_cloud);
    ros::spinOnce();
    ros::Duration(0.3).sleep();
  }

  return 0;
}
