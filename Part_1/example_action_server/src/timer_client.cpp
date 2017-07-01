

#include <actionlib/client/simple_action_client.h>
#include <example_action_server/demoAction.h>
#include <ros/ros.h>

using namespace std;

bool g_goal_active = false;
int g_result_output = -1;
int g_fdbk = -1;

void doneCb(const actionlib::SimpleClientGoalState& state, const example_action_server::demoResultConstPtr& result)
{
  ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
  ROS_INFO("got result output = %d", result->output);
  g_result_output = result->output;
  g_goal_active = false;
}

void feedbackCb(const example_action_server::demoFeedbackConstPtr& fdbk_msg)
{
  ROS_INFO("feedback status = %d", fdbk_msg->fdbk);
  g_fdbk = fdbk_msg->fdbk;
}

void activeCb()
{
  ROS_INFO("Goal just went active");
  g_goal_active = true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "timer_client_node");
  ros::NodeHandle n;
  ros::Rate main_timer(1.0);

  example_action_server::demoGoal goal;

  actionlib::SimpleActionClient<example_action_server::demoAction> action_client("timer_action", true);

  ROS_INFO("attempting to connect to server: ");
  bool server_exists = action_client.waitForServer(ros::Duration(1.0));

  while (!server_exists)
  {
    ROS_WARN("could not connect to server; retrying...");
    server_exists = action_client.waitForServer(ros::Duration(1.0));
  }
  ROS_INFO("connected to action server");  
        
<<<<<<< Updated upstream
  int countdown_goal = 1;
  while (countdown_goal >= 0 && ros::ok())
  {
=======
  int countdown_goal = 1;
  while (countdown_goal >= 0)
  {
>>>>>>> Stashed changes
    cout << "enter a desired timer value, in seconds (0 to abort, <0 to quit): ";
    cin >> countdown_goal;
    if (countdown_goal == 0)
    {
      ROS_INFO("cancelling goal");
      action_client.cancelGoal();
    }
    if (countdown_goal < 0)
    {
      ROS_INFO("this client is quitting");
      return 0;
    }

    ROS_INFO("sending timer goal= %d seconds to timer action server", countdown_goal);
    goal.input = countdown_goal;

    action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
  }
  return 0;
}
