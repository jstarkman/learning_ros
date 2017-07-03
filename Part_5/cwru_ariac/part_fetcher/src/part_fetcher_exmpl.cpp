#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib/server/simple_action_server.h>
#include <generic_gripper_services/genericGripperInterface.h>
#include <object_grabber/object_grabberAction.h>
#include <object_manipulation_properties/object_ID_codes.h>
#include <part_fetcher/PartFetcherAction.h>
#include <ros/ros.h>
#include <xform_utils/xform_utils.h>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <Eigen/Geometry>

using namespace std;
XformUtils xformUtils;

geometry_msgs::PoseStamped g_object_pickup_poseStamped;
geometry_msgs::PoseStamped g_object_dropoff_poseStamped;
int g_object_ID;
bool g_received_order = false;

class PartFetcherActionServer
{
private:
  ros::NodeHandle nh_;

  actionlib::SimpleActionServer<part_fetcher::PartFetcherAction> as_;

  part_fetcher::PartFetcherGoal goal_;
  part_fetcher::PartFetcherResult result_;
  part_fetcher::PartFetcherFeedback feedback_;

public:
  PartFetcherActionServer();

  ~PartFetcherActionServer(void)
  {
  }

  void executeCB(const actionlib::SimpleActionServer<part_fetcher::PartFetcherAction>::GoalConstPtr& goal);
};

PartFetcherActionServer::PartFetcherActionServer()
  : as_(nh_, "part_fetcher", boost::bind(&PartFetcherActionServer::executeCB, this, _1), false)
{
  ROS_INFO("in constructor of PartFetcherActionServer...");

  as_.start();
}

void PartFetcherActionServer::executeCB(
    const actionlib::SimpleActionServer<part_fetcher::PartFetcherAction>::GoalConstPtr& goal)
{
  g_object_pickup_poseStamped = goal->object_frame;
  g_object_dropoff_poseStamped = goal->desired_frame;
  g_object_ID = goal->object_id;
  ROS_INFO("requested fetch of part_id = %d from location ", g_object_ID);
  xformUtils.printStampedPose(g_object_pickup_poseStamped);
  ROS_INFO("requested dropoff at pose: ");
  xformUtils.printStampedPose(g_object_dropoff_poseStamped);
  g_received_order = true;

  result_.rtn_code = 0;
  as_.setSucceeded(result_);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "part_fetcher_action_server");

  ROS_INFO("instantiating the part-fetcher action server: ");

  PartFetcherActionServer as_object;
  ROS_INFO("going into spin");

  while (ros::ok())
  {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
    if (g_received_order)
    {
      ROS_WARN("main: received new order; should act on it!");
      g_received_order = false;
    }
  }

  return 0;
}
