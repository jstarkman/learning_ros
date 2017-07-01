

#include <actionlib/server/simple_action_server.h>
#include <navigator/navigatorAction.h>
#include <ros/ros.h>

class Navigator
{
private:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<navigator::navigatorAction> navigator_as_;

  navigator::navigatorGoal goal_;
  navigator::navigatorResult result_;
  navigator::navigatorFeedback feedback_;

  int navigate_home();
  int navigate_to_table();
  int navigate_to_pose(geometry_msgs::PoseStamped goal_pose);

public:
  Navigator();

  ~Navigator(void)
  {
  }

  void executeCB(const actionlib::SimpleActionServer<navigator::navigatorAction>::GoalConstPtr& goal);
};

Navigator::Navigator()
  : navigator_as_(nh_, "navigatorActionServer", boost::bind(&Navigator::executeCB, this, _1), false)
{
  ROS_INFO("in constructor of navigator...");

  navigator_as_.start();
}

int Navigator::navigate_home()
{
  return navigator::navigatorResult::DESIRED_POSE_ACHIEVED;
}
int Navigator::navigate_to_table()
{
  return navigator::navigatorResult::DESIRED_POSE_ACHIEVED;
}
int Navigator::navigate_to_pose(geometry_msgs::PoseStamped goal_pose)
{
  return navigator::navigatorResult::DESIRED_POSE_ACHIEVED;
}

void Navigator::executeCB(const actionlib::SimpleActionServer<navigator::navigatorAction>::GoalConstPtr& goal)
{
  int destination_id = goal->location_code;
  geometry_msgs::PoseStamped destination_pose;
  int navigation_status;

  if (destination_id == navigator::navigatorGoal::COORDS)
  {
    destination_pose = goal->desired_pose;
  }

  switch (destination_id)
  {
    case navigator::navigatorGoal::HOME:

      navigation_status = navigate_home();
      if (navigation_status == navigator::navigatorResult::DESIRED_POSE_ACHIEVED)
      {
        ROS_INFO("reached home");
        result_.return_code = navigator::navigatorResult::DESIRED_POSE_ACHIEVED;
        navigator_as_.setSucceeded(result_);
      }
      else
      {
        ROS_WARN("could not navigate home!");
        navigator_as_.setAborted(result_);
      }
      break;
    case navigator::navigatorGoal::TABLE:

      navigation_status = navigate_to_table();
      if (navigation_status == navigator::navigatorResult::DESIRED_POSE_ACHIEVED)
      {
        ROS_INFO("reached table");
        result_.return_code = navigator::navigatorResult::DESIRED_POSE_ACHIEVED;
        navigator_as_.setSucceeded(result_);
      }
      else
      {
        ROS_WARN("could not navigate to table!");
        navigator_as_.setAborted(result_);
      }
      break;
    case navigator::navigatorGoal::COORDS:

      destination_pose = goal->desired_pose;
      navigation_status = navigate_to_pose(destination_pose);
      if (navigation_status == navigator::navigatorResult::DESIRED_POSE_ACHIEVED)
      {
        ROS_INFO("reached desired pose");
        result_.return_code = navigator::navigatorResult::DESIRED_POSE_ACHIEVED;
        navigator_as_.setSucceeded(result_);
      }
      else
      {
        ROS_WARN("could not navigate to desired pose!");
        navigator_as_.setAborted(result_);
      }
      break;

    default:
      ROS_WARN("this location ID is not implemented");
      result_.return_code = navigator::navigatorResult::DESTINATION_CODE_UNRECOGNIZED;
      navigator_as_.setAborted(result_);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "navigation_action_server");

  ROS_INFO("instantiating the navigation action server: ");

  Navigator navigator_as;

  ROS_INFO("going into spin");

  while (ros::ok())
  {
    ros::spinOnce();
  }

  return 0;
}
