

#include <actionlib/server/simple_action_server.h>
#include <ros/ros.h>

#include <example_action_server/demoAction.h>

int g_count = 0;
bool g_count_failure = false;

class ExampleActionServer
{
private:
  ros::NodeHandle nh_;

  actionlib::SimpleActionServer<example_action_server::demoAction> as_;

  example_action_server::demoGoal goal_;
  example_action_server::demoResult result_;
  example_action_server::demoFeedback feedback_;

  int countdown_val_;

public:
  ExampleActionServer();

  ~ExampleActionServer(void)
  {
  }

  void executeCB(const actionlib::SimpleActionServer<example_action_server::demoAction>::GoalConstPtr& goal);
};

ExampleActionServer::ExampleActionServer()
  : as_(nh_, "timer_action", boost::bind(&ExampleActionServer::executeCB, this, _1), false)

{
  ROS_INFO("in constructor of exampleActionServer...");

  as_.start();
}

void ExampleActionServer::executeCB(
    const actionlib::SimpleActionServer<example_action_server::demoAction>::GoalConstPtr& goal)
{
  ROS_INFO("in executeCB");
  ROS_INFO("goal input is: %d", goal->input);

  ros::Rate timer(1.0);
  countdown_val_ = goal->input;

  while (countdown_val_ > 0)
  {
    ROS_INFO("countdown = %d", countdown_val_);

    if (as_.isPreemptRequested())
    {
      ROS_WARN("goal cancelled!");
      result_.output = countdown_val_;
      as_.setAborted(result_);
      return;
    }

    feedback_.fdbk = countdown_val_;
    as_.publishFeedback(feedback_);
    countdown_val_--;
    timer.sleep();
  }

  result_.output = countdown_val_;
  as_.setSucceeded(result_);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "timer_action_server_node");

  ROS_INFO("instantiating the timer_action_server: ");

  ExampleActionServer as_object;

  ROS_INFO("going into spin");

  ros::spin();

  return 0;
}
