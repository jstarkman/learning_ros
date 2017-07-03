

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <coordinator/ManipTaskAction.h>
#include <object_manipulation_properties/object_ID_codes.h>
#include <ros/ros.h>

#include <object_finder/objectFinderAction.h>
#include <object_grabber/object_grabberAction.h>
bool g_goal_done = true;
int g_callback_status = coordinator::ManipTaskResult::PENDING;
int g_object_grabber_return_code = 0;
int g_object_finder_return_code = 0;
int g_fdbk_count = 0;

geometry_msgs::PoseStamped g_des_flange_pose_stamped_wrt_torso;
geometry_msgs::PoseStamped g_object_pose;
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

    case coordinator::ManipTaskResult::FAILED_PERCEPTION:
      ROS_WARN("returned FAILED_PERCEPTION");
      g_object_finder_return_code = result->object_finder_return_code;
      break;
    case coordinator::ManipTaskResult::FAILED_PICKUP:
      ROS_WARN("returned FAILED_PICKUP");
      g_object_grabber_return_code = result->object_grabber_return_code;
      g_object_pose = result->object_pose;

      break;
    case coordinator::ManipTaskResult::FAILED_DROPOFF:
      ROS_WARN("returned FAILED_DROPOFF");

      break;
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
  ros::init(argc, argv, "task_client_node");
  ros::NodeHandle nh;

  coordinator::ManipTaskGoal goal;

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

  ROS_INFO("sending a goal: move to pre-pose");
  g_goal_done = false;
  goal.action_code = coordinator::ManipTaskGoal::MOVE_TO_PRE_POSE;
  action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
  while (!g_goal_done)
  {
    ros::Duration(0.1).sleep();
  }
  if (g_callback_status != coordinator::ManipTaskResult::MANIP_SUCCESS)
  {
    ROS_ERROR("failed to move quitting");
    return 0;
  }

  ROS_INFO("sending a goal: seeking table top");
  g_goal_done = false;
  goal.action_code = coordinator::ManipTaskGoal::FIND_TABLE_SURFACE;

  action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
  while (!g_goal_done)
  {
    ros::Duration(0.1).sleep();
  }

  ROS_INFO("sending a goal: find block");
  g_goal_done = false;
  goal.action_code = coordinator::ManipTaskGoal::GET_PICKUP_POSE;
  goal.object_code = ObjectIdCodes::TOY_BLOCK_ID;
  goal.perception_source = coordinator::ManipTaskGoal::PCL_VISION;
  action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
  while (!g_goal_done)
  {
    ros::Duration(0.1).sleep();
  }
  if (g_callback_status != coordinator::ManipTaskResult::MANIP_SUCCESS)
  {
    ROS_ERROR("failed to find block quitting");
    return 0;
  }
  g_object_pose = g_result.object_pose;
  ROS_INFO_STREAM("object pose w/rt frame-id " << g_object_pose.header.frame_id << endl);
  ROS_INFO_STREAM("object origin: (x,y,z) = (" << g_object_pose.pose.position.x << ", " << g_object_pose.pose.position.y
                                               << ", " << g_object_pose.pose.position.z << ")" << endl);
  ROS_INFO_STREAM("orientation: (qx,qy,qz,qw) = ("
                  << g_object_pose.pose.orientation.x << "," << g_object_pose.pose.orientation.y << ","
                  << g_object_pose.pose.orientation.z << "," << g_object_pose.pose.orientation.w << ")" << endl);

  ROS_INFO("sending a goal: grab block");
  g_goal_done = false;
  goal.action_code = coordinator::ManipTaskGoal::GRAB_OBJECT;
  goal.pickup_frame = g_result.object_pose;
  goal.object_code = ObjectIdCodes::TOY_BLOCK_ID;
  action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
  while (!g_goal_done)
  {
    ros::Duration(0.1).sleep();
  }
  if (g_callback_status != coordinator::ManipTaskResult::MANIP_SUCCESS)
  {
    ROS_ERROR("failed to grab block; quitting");
    return 0;
  }

  ROS_INFO("sending a goal: move to pre-pose");
  g_goal_done = false;
  goal.action_code = coordinator::ManipTaskGoal::MOVE_TO_PRE_POSE;
  action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
  while (!g_goal_done)
  {
    ros::Duration(0.1).sleep();
  }
  if (g_callback_status != coordinator::ManipTaskResult::MANIP_SUCCESS)
  {
    ROS_ERROR("failed to move to pre-pose; quitting");
    return 0;
  }

  return 0;
}
