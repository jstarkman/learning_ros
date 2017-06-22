

#include <math.h>
#include <ros/ros.h>
#include <stdlib.h>

#include <pcl/ros/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl-1.7/pcl/PCLHeader.h>
#include <pcl-1.7/pcl/point_cloud.h>

using namespace std;

void make_clouds(pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr,
                 pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr)
{
  uint8_t r(255), g(15), b(15);

  pcl::PointXYZ basic_point;
  pcl::PointXYZRGB point;

  for (float z = -1.0; z <= 1.0; z += 0.05)
  {
    uint32_t rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));

    float rgb_float = *reinterpret_cast<float*>(&rgb);

    for (float ang = 0.0; ang <= 2.0 * M_PI; ang += 2.0 * M_PI / 72.0)
    {
      basic_point.x = 0.5 * cosf(ang);
      basic_point.y = sinf(ang);
      basic_point.z = z;
      basic_cloud_ptr->points.push_back(basic_point);

      point.x = basic_point.x;
      point.y = basic_point.y;
      point.z = basic_point.z;

      point.rgb = rgb_float;
      point_cloud_ptr->points.push_back(point);
    }
    if (z < 0.0)
    {
      r -= 12;
      g += 12;
    }
    else
    {
      g -= 12;
      b += 12;
    }
  }

  basic_cloud_ptr->width = (int)basic_cloud_ptr->points.size();
  basic_cloud_ptr->height = 1;
  basic_cloud_ptr->header.frame_id = "camera";

  point_cloud_ptr->width = (int)point_cloud_ptr->points.size();
  point_cloud_ptr->height = 1;
  point_cloud_ptr->header.frame_id = "camera";
}
