#include <laser_assembler/AssembleScans2.h>
#include <ros/ros.h>
using namespace laser_assembler;
int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_client");
  ros::NodeHandle n;
  ros::service::waitForService("assemble_scans");
  ROS_INFO("found assemble-scans service");
  ros::ServiceClient client = n.serviceClient<AssembleScans2>("assemble_scans2");
  AssembleScans2 srv;
  ros::Duration one_sec(1.0);
  ros::Time begin = ros::Time::now();
  ros::Time end = begin;
  while (ros::ok())
  {
    begin = end;
    end = begin + one_sec;
    srv.request.begin = begin;
    srv.request.end = end;
    if (client.call(srv))

      ROS_INFO("Got cloud with %d points\n", (int)srv.response.cloud.data.size());
    else
      ROS_WARN("Service call failed\n");
    ros::Duration(1.0).sleep();
  }

  return 0;
}
