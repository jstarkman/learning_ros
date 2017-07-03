

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <generic_gripper_services/genericGripperInterface.h>
#include <object_grabber/object_grabber3Action.h>
#include <object_manipulation_properties/object_ID_codes.h>
#include <ros/ros.h>
#include <xform_utils/xform_utils.h>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <Eigen/Geometry>

using namespace std;
XformUtils xformUtils;

int g_object_grabber_return_code;

void objectGrabberDoneCb(const actionlib::SimpleClientGoalState& state,
                         const object_grabber::object_grabber3ResultConstPtr& result)
{
  ROS_INFO(" objectGrabberDoneCb: server responded with state [%s]", state.toString().c_str());
  g_object_grabber_return_code = result->return_code;
  ROS_INFO("got result output = %d; ", g_object_grabber_return_code);
}

void set_example_object_frames(geometry_msgs::PoseStamped& object_poseStamped,
                               geometry_msgs::PoseStamped& object_dropoff_poseStamped)
{
  object_poseStamped.header.frame_id = "system_ref_frame";
  object_poseStamped.pose.position.x = 0.5;
  object_poseStamped.pose.position.y = -0.35;
  object_poseStamped.pose.position.z = 0.7921;
  object_poseStamped.pose.orientation.x = 0;
  object_poseStamped.pose.orientation.y = 0;
  object_poseStamped.pose.orientation.z = 0.842;
  object_poseStamped.pose.orientation.w = 0.54;
  object_poseStamped.header.stamp = ros::Time::now();

  object_dropoff_poseStamped = object_poseStamped;
  object_dropoff_poseStamped.pose.orientation.z = 1;
  object_dropoff_poseStamped.pose.orientation.w = 0;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "example_object_grabber_action_client");
  ros::NodeHandle nh;
  double dt_wait = 0.5;
  double wait_time;
  const double MAX_WAIT_TIME = 30.0;
  ros::Duration sleeper(dt_wait);

  int object_id = ObjectIdCodes::TOY_BLOCK_ID;

  object_grabber::object_grabber3Goal object_grabber_goal;
  geometry_msgs::PoseStamped object_pickup_poseStamped;
  geometry_msgs::PoseStamped object_dropoff_poseStamped;

  set_example_object_frames(object_pickup_poseStamped, object_dropoff_poseStamped);

  actionlib::SimpleActionClient<object_grabber::object_grabber3Action> object_grabber_ac(
      "object_grabber_action_service", true);

  ROS_INFO("waiting for server: ");
  bool server_exists = false;
  while ((!server_exists) && (ros::ok()))
  {
    server_exists = object_grabber_ac.waitForServer(ros::Duration(0.5));
    ros::spinOnce();
    ros::Duration(0.5).sleep();
    ROS_INFO("retrying...");
  }
  ROS_INFO("connected to object_grabber action server");

  bool finished_before_timeout;
  ROS_INFO("sending test code: ");
  object_grabber_goal.action_code = object_grabber::object_grabber3Goal::TEST_CODE;
  object_grabber_ac.sendGoal(object_grabber_goal, &objectGrabberDoneCb);
  finished_before_timeout = object_grabber_ac.waitForResult(ros::Duration(30.0));
  if (!finished_before_timeout)
  {
    ROS_WARN("giving up waiting on result ");
    return 1;
  }

  ROS_INFO("sending command to move to waiting pose");
  object_grabber_goal.action_code = object_grabber::object_grabber3Goal::MOVE_TO_WAITING_POSE;
  object_grabber_ac.sendGoal(object_grabber_goal, &objectGrabberDoneCb);
  finished_before_timeout = object_grabber_ac.waitForResult(ros::Duration(30.0));
  if (!finished_before_timeout)
  {
    ROS_WARN("giving up waiting on result ");
    return 1;
  }

  ROS_INFO("sending a grab-object command");
  object_grabber_goal.action_code = object_grabber::object_grabber3Goal::GRAB_OBJECT;
  object_grabber_goal.object_id = ObjectIdCodes::TOY_BLOCK_ID;
  object_grabber_goal.object_frame = object_pickup_poseStamped;
  object_grabber_goal.grasp_option = object_grabber::object_grabber3Goal::DEFAULT_GRASP_STRATEGY;
  object_grabber_goal.speed_factor = 1.0;
  ROS_INFO("sending goal to grab object: ");
  object_grabber_ac.sendGoal(object_grabber_goal, &objectGrabberDoneCb);
  ROS_INFO("waiting on result");
  finished_before_timeout = object_grabber_ac.waitForResult(ros::Duration(30.0));

  if (!finished_before_timeout)
  {
    ROS_WARN("giving up waiting on result ");
    return 1;
  }

  ROS_INFO("sending a dropoff-object command");
  object_grabber_goal.action_code = object_grabber::object_grabber3Goal::DROPOFF_OBJECT;
  object_grabber_goal.object_id = ObjectIdCodes::TOY_BLOCK_ID;
  object_grabber_goal.object_frame = object_dropoff_poseStamped;
  object_grabber_goal.grasp_option = object_grabber::object_grabber3Goal::DEFAULT_GRASP_STRATEGY;
  object_grabber_goal.speed_factor = 1.0;
  ROS_INFO("sending goal to dropoff object: ");
  object_grabber_ac.sendGoal(object_grabber_goal, &objectGrabberDoneCb);
  ROS_INFO("waiting on result");
  finished_before_timeout = object_grabber_ac.waitForResult(ros::Duration(30.0));

  if (!finished_before_timeout)
  {
    ROS_WARN("giving up waiting on result ");
    return 1;
  }

  return 0;

  /*

  object_grabber_goal.object_id = ObjectIdCodes::TOY_BLOCK_ID;
  object_grabber_goal.desired_frame = object_poseStamped;
  object_grabber_goal.action_code = object_grabber::object_grabberGoal::PLAN_OBJECT_GRASP;
  g_object_grabber_return_code = object_grabber::object_grabberResult::OBJECT_GRABBER_BUSY;
  object_grabber_ac.sendGoal(object_grabber_goal, &objectGrabberDoneCb);
  wait_time = MAX_WAIT_TIME;

  while (g_object_grabber_return_code==object_grabber::object_grabberResult::OBJECT_GRABBER_BUSY) {
  wait_time-= dt_wait;
  ros::spinOnce();
  sleeper.sleep();
  if (wait_time<0) {
 ROS_WARN("giving up waiting!");
 return 1;
  }
  }











  ROS_INFO("attempt to grab toy block at object pose: ");
  xformUtils.printStampedPose(object_poseStamped);
  ROS_INFO("sending goal: ");
  object_grabber_ac.sendGoal(object_grabber_goal, &objectGrabberDoneCb);
  finished_before_timeout = object_grabber_ac.waitForResult(ros::Duration(30.0));
  if (!finished_before_timeout) {
  ROS_WARN("giving up waiting on result ");
  return 1;
  }

  if (g_object_grabber_return_code != object_grabber::object_grabberResult::SUCCESS) {
  ROS_WARN("return code was not SUCCESS; giving up");
  return 1;
  }


  object_grabber_goal.action_code = object_grabber::object_grabberGoal::DROPOFF_ALONG_TOOL_Z;
  object_grabber_goal.desired_frame = toy_block_dropoff_poseStamped;
  object_grabber_goal.object_id = object_ID_codes::TOY_BLOCK_ID;

  ROS_INFO("attempting to place toy block at object pose: ");
  xformUtils.printStampedPose(toy_block_dropoff_poseStamped);
  ROS_INFO("sending goal: ");
  object_grabber_ac.sendGoal(object_grabber_goal, &objectGrabberDoneCb);
  finished_before_timeout = object_grabber_ac.waitForResult(ros::Duration(30.0));
  if (!finished_before_timeout) {
  ROS_WARN("giving up waiting on result ");
  return 1;
  }
   */
  return 0;
}
