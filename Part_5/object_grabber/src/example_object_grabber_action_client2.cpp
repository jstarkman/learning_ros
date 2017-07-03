

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <object_grabber/object_grabberAction.h>
#include <object_manipulation_properties/object_manipulation_properties.h>
#include <ros/ros.h>
#include <xform_utils/xform_utils.h>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <Eigen/Geometry>

int g_object_grabber_return_code;
void objectGrabberDoneCb(const actionlib::SimpleClientGoalState& state,
                         const object_grabber::object_grabberResultConstPtr& result)
{
  ROS_INFO(" objectGrabberDoneCb: server responded with state [%s]", state.toString().c_str());
  g_object_grabber_return_code = result->return_code;
  ROS_INFO("got result output = %d; ", g_object_grabber_return_code);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "example_object_grabber_action_client");
  ros::NodeHandle nh;
  XformUtils xformUtils;
  actionlib::SimpleActionClient<object_grabber::object_grabberAction> object_grabber_ac("object_grabber_action_service",
                                                                                        true);
  geometry_msgs::PoseStamped toy_block_poseStamped;
  geometry_msgs::PoseStamped toy_block_dropoff_poseStamped;

  toy_block_poseStamped.header.frame_id = "torso";
  toy_block_poseStamped.pose.position.x = 0.5;
  toy_block_poseStamped.pose.position.y = -0.35;
  toy_block_poseStamped.pose.position.z = -0.125;
  toy_block_poseStamped.pose.orientation.x = 0;
  toy_block_poseStamped.pose.orientation.y = 0;
  toy_block_poseStamped.pose.orientation.z = 0.842;
  toy_block_poseStamped.pose.orientation.w = 0.54;
  toy_block_poseStamped.header.stamp = ros::Time::now();

  toy_block_dropoff_poseStamped = toy_block_poseStamped;
  toy_block_dropoff_poseStamped.pose.orientation.z = 1;
  toy_block_dropoff_poseStamped.pose.orientation.w = 0;

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

  object_grabber::object_grabberGoal object_grabber_goal;
  bool finished_before_timeout;

  object_grabber_goal.action_code = object_grabber::object_grabberGoal::GRAB_W_TOOL_Z_APPROACH;
  object_grabber_goal.object_id = TOY_BLOCK_ID;
  object_grabber_goal.desired_frame = toy_block_poseStamped;

  ROS_INFO("attempt to grab toy block at object pose: ");
  xformUtils.printStampedPose(toy_block_poseStamped);
  ROS_INFO("sending goal: ");
  object_grabber_ac.sendGoal(object_grabber_goal, &objectGrabberDoneCb);
  finished_before_timeout = object_grabber_ac.waitForResult(ros::Duration(30.0));
  if (!finished_before_timeout)
  {
    ROS_WARN("giving up waiting on result ");
    return 1;
  }

  if (g_object_grabber_return_code != object_grabber::object_grabberResult::SUCCESS)
  {
    ROS_WARN("return code was not SUCCESS; giving up");
    return 1;
  }

  object_grabber_goal.action_code = object_grabber::object_grabberGoal::DROPOFF_ALONG_TOOL_Z;
  object_grabber_goal.desired_frame = toy_block_dropoff_poseStamped;
  object_grabber_goal.object_id = TOY_BLOCK_ID;

  ROS_INFO("attempting to place toy block at object pose: ");
  xformUtils.printStampedPose(toy_block_dropoff_poseStamped);
  ROS_INFO("sending goal: ");
  object_grabber_ac.sendGoal(object_grabber_goal, &objectGrabberDoneCb);
  finished_before_timeout = object_grabber_ac.waitForResult(ros::Duration(30.0));
  if (!finished_before_timeout)
  {
    ROS_WARN("giving up waiting on result ");
    return 1;
  }
  return 0;
}
