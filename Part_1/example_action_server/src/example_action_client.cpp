

#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>

#include <example_action_server/demoAction.h>

void doneCb(const actionlib::SimpleClientGoalState& state, const example_action_server::demoResultConstPtr& result)
{
  ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
  int diff = result->output - result->goal_stamp;
  ROS_INFO("got result output = %d; goal_stamp = %d; diff = %d", result->output, result->goal_stamp, diff);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "demo_action_client_node");
  int g_count = 0;

  example_action_server::demoGoal goal;

  actionlib::SimpleActionClient<example_action_server::demoAction> action_client("example_action", true);

  ROS_INFO("waiting for server: ");
  bool server_exists = action_client.waitForServer(ros::Duration(5.0));

  if (!server_exists)
  {
    ROS_WARN("could not connect to server; halting");
    return 0;
  }

  ROS_INFO("connected to action server");

  while (true)
  {
    g_count++;
    goal.input = g_count;

    action_client.sendGoal(goal, &doneCb);

    bool finished_before_timeout = action_client.waitForResult(ros::Duration(5.0));

    if (!finished_before_timeout)
    {
      ROS_WARN("giving up waiting on result for goal number %d", g_count);
      return 0;
    }
    else
    {
    }
  }

  return 0;
}
