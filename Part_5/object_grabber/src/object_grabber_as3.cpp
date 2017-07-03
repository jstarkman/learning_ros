

#include <object_grabber/object_grabber3.h>
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "object_grabber_action_server_node");
  ros::NodeHandle nh;
  ObjectGrabber object_grabber_as(&nh);

  ROS_INFO("going into spin");
  while (ros::ok())
  {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }
  return 0;
}
