

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

static const std::string OPENCV_WINDOW = "Open-CV display window";
using namespace std;

int g_redratio;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter(ros::NodeHandle& nodehandle) : it_(nh_)
  {
    image_sub_ = it_.subscribe("simple_camera/image_raw", 1, &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

<<<<<<< Updated upstream
  // image comes in as a ROS message, but gets converted to an OpenCV type
  void imageCb(const sensor_msgs::ImageConstPtr& msg);

};  // end of class definition

void ImageConverter::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;  // OpenCV data type
=======

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr; 
>>>>>>> Stashed changes
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  int npix = 0;
  int isum = 0;
  int jsum = 0;
  int redval, blueval, greenval, testval;
  cv::Vec3b rgbpix;

  for (int i = 0; i < cv_ptr->image.cols; i++)
  {
    for (int j = 0; j < cv_ptr->image.rows; j++)
    {
      rgbpix = cv_ptr->image.at<cv::Vec3b>(j, i);

      redval = rgbpix[2] + 1;
      blueval = rgbpix[0] + 1;
      greenval = rgbpix[1] + 1;

      testval = redval / (blueval + greenval);

      if (testval > g_redratio)
      {
        cv_ptr->image.at<cv::Vec3b>(j, i)[0] = 255;
        cv_ptr->image.at<cv::Vec3b>(j, i)[1] = 255;
        cv_ptr->image.at<cv::Vec3b>(j, i)[2] = 255;
        npix++;
        isum += i;
        jsum += j;
      }
      else
      {
        cv_ptr->image.at<cv::Vec3b>(j, i)[0] = 0;
        cv_ptr->image.at<cv::Vec3b>(j, i)[1] = 0;
        cv_ptr->image.at<cv::Vec3b>(j, i)[2] = 0;
      }
    }
  }

  int half_box = 5;
  int i_centroid, j_centroid;
  double x_centroid, y_centroid;
  if (npix > 0)
  {
    i_centroid = isum / npix;
    j_centroid = jsum / npix;
    x_centroid = ((double)isum) / ((double)npix);
    y_centroid = ((double)jsum) / ((double)npix);
    ROS_INFO("u_avg: %f; v_avg: %f", x_centroid, y_centroid);

    for (int i_box = i_centroid - half_box; i_box <= i_centroid + half_box; i_box++)
    {
      for (int j_box = j_centroid - half_box; j_box <= j_centroid + half_box; j_box++)
      {
        if ((i_box >= 0) && (j_box >= 0) && (i_box < cv_ptr->image.cols) && (j_box < cv_ptr->image.rows))
        {
          cv_ptr->image.at<cv::Vec3b>(j_box, i_box)[0] = 255;
          cv_ptr->image.at<cv::Vec3b>(j_box, i_box)[1] = 0;
          cv_ptr->image.at<cv::Vec3b>(j_box, i_box)[2] = 0;
        }
      }
    }
  }

  cv::imshow(OPENCV_WINDOW, cv_ptr->image);
  cv::waitKey(3);

  image_pub_.publish(cv_ptr->toImageMsg());
}
<<<<<<< Updated upstream
=======
}
; 
>>>>>>> Stashed changes

int main(int argc, char** argv)
{
  ros::init(argc, argv, "red_pixel_finder");
  ros::NodeHandle n;
  ImageConverter ic(n);

  g_redratio = 10;
  ros::Duration timer(0.1);
  double x, y, z;
  while (ros::ok())
  {
    ros::spinOnce();
    timer.sleep();
  }
  return 0;
}
