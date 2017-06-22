

#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

class ImageSyncher
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber camera_sub_left_;
  image_transport::CameraSubscriber camera_sub_right_;
  image_transport::CameraPublisher camera_pub_left_;
  image_transport::CameraPublisher camera_pub_right_;

  sensor_msgs::Image img_left_, img_right_;
  sensor_msgs::CameraInfo info_left_, info_right_;

  int sequence;

  void imageLeftCb(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
    int img_datasize = image_msg->data.size();

    int cpy_img_datasize = img_left_.data.size();
    if (cpy_img_datasize != img_datasize)
    {
      img_left_.data.resize(img_datasize);
      ROS_INFO("resizing image");
    }

    img_left_ = *image_msg;

    int info_datasize = info_msg->D.size();

    int cpy_info_datasize = info_left_.D.size();

    if (cpy_info_datasize != info_datasize)
    {
      info_left_.D.resize(info_datasize);
    }
    info_left_ = *info_msg;
    got_new_image_left_ = true;
  }
  void imageRightCb(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
    int img_datasize = image_msg->data.size();

    int cpy_img_datasize = img_right_.data.size();
    if (cpy_img_datasize != img_datasize)
    {
      img_right_.data.resize(img_datasize);
    }
    img_right_ = *image_msg;

    int info_datasize = info_msg->D.size();

    int cpy_info_datasize = info_right_.D.size();

    if (cpy_info_datasize != info_datasize)
    {
      info_right_.D.resize(info_datasize);
    }
    info_right_ = *info_msg;
    got_new_image_right_ = true;
  }

public:
  bool got_new_image_left_;
  bool got_new_image_right_;

  ImageSyncher(ros::NodeHandle& nh)
    : nh_(nh)
    , it_(nh_)
    , camera_sub_left_(it_.subscribeCamera("/unsynced/left/image_raw", 1, &ImageSyncher::imageLeftCb, this))
    , camera_sub_right_(it_.subscribeCamera("/unsynced/right/image_raw", 1, &ImageSyncher::imageRightCb, this))
    , camera_pub_left_(it_.advertiseCamera("/stereo_sync/left/image_raw", 1))
    , camera_pub_right_(it_.advertiseCamera("/stereo_sync/right/image_raw", 1))
    , sequence(0)
    , got_new_image_left_(false)
    , got_new_image_right_(false)
  {
    img_left_.header.frame_id = "left_camera_optical_frame";
    img_right_.header.frame_id = "right_camera_optical_frame";
    info_left_.header.frame_id = "left_camera_optical_frame";
    info_right_.header.frame_id = "right_camera_optical_frame";
  }

  void pub_both_images()
  {
    ros::Time tnow = ros::Time::now();
    img_left_.header.stamp = tnow;
    img_right_.header.stamp = tnow;
    info_left_.header.stamp = tnow;
    info_right_.header.stamp = tnow;

    img_left_.header.seq = sequence;
    img_right_.header.seq = sequence;
    info_left_.header.seq = sequence;
    info_right_.header.seq = sequence;

    camera_pub_left_.publish(img_left_, info_left_);
    got_new_image_left_ = false;
    camera_pub_right_.publish(img_right_, info_right_);
    got_new_image_right_ = false;
    sequence++;
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");

  ros::NodeHandle nh;
  ros::Rate ratetimer(10);
  ImageSyncher is(nh);
  while (ros::ok())
  {
    ros::spinOnce();
    if (is.got_new_image_left_ && is.got_new_image_right_)
      is.pub_both_images();
    ratetimer.sleep();
  }
  return 0;
}
