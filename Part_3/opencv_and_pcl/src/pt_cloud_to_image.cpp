

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <math.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <stdlib.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <pcl/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl-1.7/pcl/PCLHeader.h>
#include <pcl-1.7/pcl/point_cloud.h>
#include <pcl/common/common_headers.h>

using namespace std;

int Nv = 640;
int Nu = 480;
double focal_len = 520;

double z_min = 0.5;
double z_max = 1.5;

cv::Mat g_image(Nu, Nv, CV_8U, cv::Scalar(0));

cv::Mat g_pts_per_cell(Nu, Nv, CV_8U, cv::Scalar(0));

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter() : it_(nh_)
  {
    image_sub_ = it_.subscribe("/cameras/left_hand_camera/image", 1, &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255, 0, 0));

    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");

  string fname;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclKinect_clr_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

  cout << "enter pcd file name: ";
  cin >> fname;
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(fname, *pclKinect_clr_ptr) == -1)
  {
    ROS_ERROR("Couldn't read file \n");
    return (-1);
  }
  int npts_cloud = pclKinect_clr_ptr->width * pclKinect_clr_ptr->height;
  std::cout << "Loaded " << npts_cloud << " data points from file " << fname << std::endl;
  Eigen::Vector3f cloud_pt;
  double x, y, z;
  double v, vc, u, uc;
  int i, j;
  uc = Nu / 2;
  vc = Nv / 2;
  uchar gray_level;
  double r;

  while (z_min > 0)
  {
    cout << "enter z_min: ";
    cin >> z_min;
    cout << "enter z_max: ";
    cin >> z_max;
    for (int ipt = 0; ipt < npts_cloud; ipt++)
    {
      cloud_pt = pclKinect_clr_ptr->points[ipt].getVector3fMap();
      z = cloud_pt[2];
      y = cloud_pt[1];
      x = cloud_pt[0];
      if ((z == z) && (x == x) && (y == y))
      {
        u = uc + focal_len * x / z;
        i = round(u);
        v = vc + focal_len * y / z;
        j = round(v);
        if ((i >= 0) && (i < Nu) && (j >= 0) && (j < Nv))
        {
          r = sqrt(z * z + y * y + x * x);
          if (r > z_max)
            gray_level = 0;
          else if (r < z_min)
            gray_level = 0;
          else
          {
            gray_level = (uchar)(255 * (z_max - r) / (z_max - z_min));
          }
          g_image.at<uchar>(j, i) = gray_level;
        }
      }
    }
    std::cout << "output image size: " << g_image.size().height << " , " << g_image.size().width << std::endl;

    cv::imwrite("output.bmp", g_image);
  }

  return 0;
}
