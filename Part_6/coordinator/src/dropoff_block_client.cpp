

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <coordinator/ManipTaskAction.h>
#include <example_gazebo_set_state/SrvInt.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <object_finder/objectFinderAction.h>
#include <object_grabber/object_grabberAction.h>
#include <object_manipulation_properties/object_ID_codes.h>
#include <ros/ros.h>
#include <fstream>
bool g_goal_done = true;
int g_callback_status = coordinator::ManipTaskResult::PENDING;
int g_fdbk_count = 0;

coordinator::ManipTaskResult g_result;

using namespace std;

void doneCb(const actionlib::SimpleClientGoalState& state, const coordinator::ManipTaskResultConstPtr& result)
{
  ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
  g_goal_done = true;
  g_result = *result;
  g_callback_status = result->manip_return_code;

  switch (g_callback_status)
  {
    case coordinator::ManipTaskResult::MANIP_SUCCESS:
      ROS_INFO("returned MANIP_SUCCESS");

      break;
    default:
      ROS_ERROR("action server response indicates failure");
  }
}

void feedbackCb(const coordinator::ManipTaskFeedbackConstPtr& fdbk_msg)
{
  g_fdbk_count++;
  if (g_fdbk_count > 1000)
  {
    g_fdbk_count = 0;
  }
}

void activeCb()
{
  ROS_INFO("Goal just went active");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dropoff_client_node");
  ros::NodeHandle nh;

  coordinator::ManipTaskGoal goal;
  geometry_msgs::PoseStamped dropoff_pose;

  actionlib::SimpleActionClient<coordinator::ManipTaskAction> action_client("manip_task_action_service", true);

  ROS_INFO("waiting for server: ");
  bool server_exists = false;
  while ((!server_exists) && (ros::ok()))
  {
    server_exists = action_client.waitForServer(ros::Duration(0.5));
    ros::spinOnce();
    ros::Duration(0.5).sleep();
    ROS_INFO("retrying...");
  }
  ROS_INFO("connected to action server");

  dropoff_pose.header.frame_id = "torso";
  dropoff_pose.pose.position.x = 0.5;
  dropoff_pose.pose.position.y = -0.35;
  dropoff_pose.pose.position.z = -0.12;
  dropoff_pose.pose.orientation.x = 0;
  dropoff_pose.pose.orientation.y = 0;
  dropoff_pose.pose.orientation.z = 0.707;
  dropoff_pose.pose.orientation.w = 0.707;
  dropoff_pose.header.stamp = ros::Time::now();

  ROS_INFO("sending a goal: dropoff block");

  g_goal_done = false;
  goal.action_code = coordinator::ManipTaskGoal::DROPOFF_OBJECT;

  goal.dropoff_frame = dropoff_pose;
  goal.object_code = ObjectIdCodes::TOY_BLOCK_ID;
  action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
  while (!g_goal_done)
  {
    ros::Duration(0.1).sleep();
  }
  if (g_callback_status != coordinator::ManipTaskResult::MANIP_SUCCESS)
  {
    ROS_ERROR("failed to drop off block; quitting");
    return 0;
  }

  return 0;
}
