

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <generic_gripper_services/genericGripperInterface.h>
#include <object_grabber/object_grabberAction.h>
#include <object_manipulation_properties/object_ID_codes.h>
#include <ros/ros.h>
#include <xform_utils/xform_utils.h>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <Eigen/Geometry>

using namespace std;
XformUtils xformUtils;

int g_object_grabber_return_code;
actionlib::SimpleActionClient<object_grabber::object_grabberAction>* g_object_grabber_ac_ptr;
bool g_got_callback = false;

void objectGrabberDoneCb(const actionlib::SimpleClientGoalState& state,
                         const object_grabber::object_grabberResultConstPtr& result)
{
  ROS_INFO(" objectGrabberDoneCb: server responded with state [%s]", state.toString().c_str());
  g_object_grabber_return_code = result->return_code;
  ROS_INFO("got result output = %d; ", g_object_grabber_return_code);
  g_got_callback = true;  // flag that action server has called back
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

<<<<<<< Updated upstream
void move_to_waiting_pose()
{
  ROS_INFO("sending command to move to waiting pose");
  g_got_callback = false;  // reset callback-done flag
  object_grabber::object_grabberGoal object_grabber_goal;
  object_grabber_goal.action_code = object_grabber::object_grabberGoal::MOVE_TO_WAITING_POSE;
  g_object_grabber_ac_ptr->sendGoal(object_grabber_goal, &objectGrabberDoneCb);
}
=======
int main(int argc, char** argv)
{
  ros::init(argc, argv, "example_object_grabber_action_client");
  ros::NodeHandle nh;
  double dt_wait = 0.5;
  double wait_time;
  const double MAX_WAIT_TIME = 30.0;
  ros::Duration sleeper(dt_wait);

  int object_id = ObjectIdCodes::TOY_BLOCK_ID; 
>>>>>>> Stashed changes

void grab_object(geometry_msgs::PoseStamped object_pickup_poseStamped)
{
  ROS_INFO("sending a grab-object command");
  g_got_callback = false;  // reset callback-done flag
  object_grabber::object_grabberGoal object_grabber_goal;
  object_grabber_goal.action_code =
      object_grabber::object_grabberGoal::GRAB_OBJECT;           // specify the action to be performed
  object_grabber_goal.object_id = ObjectIdCodes::TOY_BLOCK_ID;   // specify the object to manipulate
  object_grabber_goal.object_frame = object_pickup_poseStamped;  // and the object's current pose
  object_grabber_goal.grasp_option = object_grabber::object_grabberGoal::DEFAULT_GRASP_STRATEGY;  // from above
  object_grabber_goal.speed_factor = 1.0;
  ROS_INFO("sending goal to grab object: ");
  g_object_grabber_ac_ptr->sendGoal(object_grabber_goal, &objectGrabberDoneCb);
}

void dropoff_object(geometry_msgs::PoseStamped object_dropoff_poseStamped)
{
  ROS_INFO("sending a dropoff-object command");
  object_grabber::object_grabberGoal object_grabber_goal;
  object_grabber_goal.action_code =
      object_grabber::object_grabberGoal::DROPOFF_OBJECT;         // specify the action to be performed
  object_grabber_goal.object_id = ObjectIdCodes::TOY_BLOCK_ID;    // specify the object to manipulate
  object_grabber_goal.object_frame = object_dropoff_poseStamped;  // and the object's current pose
  object_grabber_goal.grasp_option = object_grabber::object_grabberGoal::DEFAULT_GRASP_STRATEGY;  // from above
  object_grabber_goal.speed_factor = 1.0;
  ROS_INFO("sending goal to dropoff object: ");
  g_object_grabber_ac_ptr->sendGoal(object_grabber_goal, &objectGrabberDoneCb);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "example_object_grabber_action_client");
  ros::NodeHandle nh;
  geometry_msgs::PoseStamped object_pickup_poseStamped;
  geometry_msgs::PoseStamped object_dropoff_poseStamped;
    
<<<<<<< Updated upstream
  // specify object pick-up and drop-off frames using simple test fnc
  // more generally, pick-up comes from perception and drop-off comes from task
=======
    
>>>>>>> Stashed changes
  set_example_object_frames(object_pickup_poseStamped, object_dropoff_poseStamped);
  // instantiate an action client of object_grabber_action_service:
  actionlib::SimpleActionClient<object_grabber::object_grabberAction> object_grabber_ac("object_grabber_action_service",
                                                                                        true);
  g_object_grabber_ac_ptr = &object_grabber_ac;  // make available to fncs
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

<<<<<<< Updated upstream
  // move to waiting pose
  move_to_waiting_pose();
  while (!g_got_callback)
  {
    ROS_INFO("waiting on move...");
    ros::Duration(0.5).sleep();  // could do something useful
  }

  grab_object(object_pickup_poseStamped);
  while (!g_got_callback)
  {
    ROS_INFO("waiting on grab...");
    ros::Duration(0.5).sleep();  // could do something useful
  }

  dropoff_object(object_dropoff_poseStamped);
  while (!g_got_callback)
  {
    ROS_INFO("waiting on dropoff...");
    ros::Duration(0.5).sleep();  // could do something useful
  }   
=======

  bool finished_before_timeout;
  ROS_INFO("sending test code: ");
  object_grabber_goal.action_code = object_grabber::object_grabberGoal::TEST_CODE;
  object_grabber_ac.sendGoal(object_grabber_goal, &objectGrabberDoneCb);
  finished_before_timeout = object_grabber_ac.waitForResult(ros::Duration(30.0));
  if (!finished_before_timeout)
  {
    ROS_WARN("giving up waiting on result ");
    return 1;
  }

  ROS_INFO("sending command to move to waiting pose");
  object_grabber_goal.action_code = object_grabber::object_grabberGoal::MOVE_TO_WAITING_POSE;
  object_grabber_ac.sendGoal(object_grabber_goal, &objectGrabberDoneCb);
  finished_before_timeout = object_grabber_ac.waitForResult(ros::Duration(30.0));
  if (!finished_before_timeout)
  {
    ROS_WARN("giving up waiting on result ");
    return 1;
  }

  ROS_INFO("sending a grab-object command");
  object_grabber_goal.action_code = object_grabber::object_grabberGoal::GRAB_OBJECT;
  object_grabber_goal.object_id = ObjectIdCodes::TOY_BLOCK_ID;
  object_grabber_goal.object_frame = object_pickup_poseStamped;
  object_grabber_goal.grasp_option = object_grabber::object_grabberGoal::DEFAULT_GRASP_STRATEGY;
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
  object_grabber_goal.action_code = object_grabber::object_grabberGoal::DROPOFF_OBJECT;
  object_grabber_goal.object_id = ObjectIdCodes::TOY_BLOCK_ID;
  object_grabber_goal.object_frame = object_dropoff_poseStamped;
  object_grabber_goal.grasp_option = object_grabber::object_grabberGoal::DEFAULT_GRASP_STRATEGY;
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
>>>>>>> Stashed changes
  return 0;
}
