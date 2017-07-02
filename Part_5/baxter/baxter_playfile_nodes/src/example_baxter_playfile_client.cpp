

#include <baxter_playfile_nodes/playfileSrv.h>
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "example_baxter_playfile_client");
  ros::NodeHandle nh;

  ros::ServiceClient client = nh.serviceClient<baxter_playfile_nodes::playfileSrv>("playfile_service");
  baxter_playfile_nodes::playfileSrv playfile_srv_msg;

  playfile_srv_msg.request.playfile_code = baxter_playfile_nodes::playfileSrvRequest::PRE_POSE;

  ROS_INFO("sending pre-pose command to playfile service: ");
  client.call(playfile_srv_msg);

  ROS_INFO("service responded with code %d", playfile_srv_msg.response.return_code);
  return 0;
}
