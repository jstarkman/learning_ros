

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <xform_utils/xform_utils.h>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <Eigen/Geometry>

geometry_msgs::PoseStamped g_destination_pose;

void set_des_pose()
{
  g_destination_pose.header.frame_id = "/map";
  g_destination_pose.header.stamp = ros::Time::now();
  g_destination_pose.pose.position.z = 0;
  g_destination_pose.pose.position.x = -8.8;
  g_destination_pose.pose.position.y = 0.18;
  g_destination_pose.pose.orientation.z = -0.707;
  g_destination_pose.pose.orientation.w = 0.707;
}

void navigatorDoneCb(const actionlib::SimpleClientGoalState& state,
                     const move_base_msgs::MoveBaseResultConstPtr& result)
{
  ROS_INFO(" navigatorDoneCb: server responded with state [%s]", state.toString().c_str());
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "example_navigator_action_client");
  ros::NodeHandle nh;
  set_des_pose();
  tf::TransformListener tfListener;
  geometry_msgs::PoseStamped current_pose;
  move_base_msgs::MoveBaseGoal move_base_goal;
  XformUtils xform_utils;

  bool tferr = true;
  ROS_INFO("waiting for tf between map and base_link...");
  tf::StampedTransform tfBaseLinkWrtMap;
  while (tferr)
  {
    tferr = false;
    try
    {
      tfListener.lookupTransform("map", "base_link", ros::Time(0), tfBaseLinkWrtMap);
    }
    catch (tf::TransformException& exception)
    {
      ROS_WARN("%s; retrying...", exception.what());
      tferr = true;
      ros::Duration(0.5).sleep();
      ros::spinOnce();
    }
  }
  ROS_INFO("tf is good; current pose is:");
  current_pose = xform_utils.get_pose_from_stamped_tf(tfBaseLinkWrtMap);
  xform_utils.printStampedPose(current_pose);

  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> navigator_ac("move_base", true);

  ROS_INFO("waiting for move_base server: ");
  bool server_exists = false;
  while ((!server_exists) && (ros::ok()))
  {
    server_exists = navigator_ac.waitForServer(ros::Duration(0.5));
    ros::spinOnce();
    ros::Duration(0.5).sleep();
    ROS_INFO("retrying...");
  }
  ROS_INFO("connected to move_base action server");

  move_base_goal.target_pose = g_destination_pose;

  ROS_INFO("sending goal: ");
  xform_utils.printStampedPose(g_destination_pose);
  navigator_ac.sendGoal(move_base_goal, &navigatorDoneCb);

  bool finished_before_timeout = navigator_ac.waitForResult(ros::Duration(120.0));

  if (!finished_before_timeout)
  {
    ROS_WARN("giving up waiting on result ");
    return 1;
  }

  return 0;
}
