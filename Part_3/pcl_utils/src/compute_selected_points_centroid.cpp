

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

using namespace std;

pcl::PointCloud<pcl::PointXYZ>::Ptr g_pclSelectedPoints_ptr;
void selectCB(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  ROS_INFO("RECEIVED NEW PATCH");

  pcl::fromROSMsg(*cloud, *g_pclSelectedPoints_ptr);
  ROS_INFO("patch dimensions: %d * %d points", g_pclSelectedPoints_ptr->width, g_pclSelectedPoints_ptr->height);

  ROS_INFO_STREAM("frame_id = " << g_pclSelectedPoints_ptr->header.frame_id << endl);

  Eigen::MatrixXf points_mat;
  Eigen::Vector3f cloud_pt;

  int npts = g_pclSelectedPoints_ptr->points.size();
  points_mat.resize(3, npts);

  for (int i = 0; i < npts; ++i)
  {
    cloud_pt = g_pclSelectedPoints_ptr->points[i].getVector3fMap();
    points_mat.col(i) = cloud_pt;
  }

  Eigen::Vector3f centroid = Eigen::MatrixXf::Zero(3, 1);

  for (int ipt = 0; ipt < npts; ipt++)
  {
    centroid += points_mat.col(ipt);
  }
  centroid /= npts;
  ROS_INFO("centroid of selected points is: (%f, %f, %f)", centroid(0), centroid(1), centroid(2));
  /**/
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "compute_selected_points_centroid");
  ros::NodeHandle nh;

  ros::Subscriber selected_points_subscriber = nh.subscribe<sensor_msgs::PointCloud2>("/selected_points", 1, selectCB);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclSelectedPoints_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  g_pclSelectedPoints_ptr = pclSelectedPoints_ptr;
  ROS_INFO(" select a patch of points to find the selected-points centroid...");

  while (ros::ok())
  {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }

  return 0;
}
