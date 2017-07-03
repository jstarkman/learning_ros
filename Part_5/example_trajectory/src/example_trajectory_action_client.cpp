

#include <actionlib/client/simple_action_client.h>
#include <example_trajectory/TrajActionAction.h>
#include <ros/ros.h>

void doneCb(const actionlib::SimpleClientGoalState& state, const example_trajectory::TrajActionResultConstPtr& result)
{
  ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "demo_trajectory_client_node");
  example_trajectory::TrajActionGoal goal;

  double omega = 1.0;
  double amp = 0.5;
  double start_angle = amp;
  double final_phase = 4 * 3.1415927;

  double dt = 0.1;

  actionlib::SimpleActionClient<example_trajectory::TrajActionAction> action_client("example_traj_action_server", true);

  ROS_INFO("waiting for server: ");
  bool server_exists = action_client.waitForServer(ros::Duration(5.0));

  ros::Duration sleep1s(1);
  if (!server_exists)
  {
    ROS_WARN("could not connect to server; retrying");
    bool server_exists = action_client.waitForServer(ros::Duration(1.0));
    sleep1s.sleep();
  }

  ROS_INFO("connected to action server");

  trajectory_msgs::JointTrajectory trajectory;
  trajectory_msgs::JointTrajectoryPoint trajectory_point;

  trajectory.joint_names.push_back("joint1");

  int njnts = trajectory.joint_names.size();
  trajectory_point.positions.resize(njnts);
  trajectory_point.velocities.resize(njnts);

  ROS_INFO("populating trajectory...");
  double final_time;
  double phase = 0.0;
  double time_from_start = 0.0;
  double q_des, qdot_des;

  for (phase = 0.0; phase < final_phase; phase += omega * dt)
  {
    q_des = start_angle + amp * sin(phase);
    qdot_des = amp * omega * cos(phase);
    trajectory_point.positions[0] = q_des;
    trajectory_point.velocities[0] = qdot_des;
    time_from_start += dt;
    ROS_INFO("phase = %f, t = %f", phase, time_from_start);

    trajectory_point.time_from_start = ros::Duration(time_from_start);

    trajectory.points.push_back(trajectory_point);

    dt = (rand() % 100 + 1) * 0.01;
  }
  final_time = time_from_start;
  int npts = trajectory.points.size();
  ROS_INFO("populated trajectory with %d points", npts);

  goal.trajectory = trajectory;

  action_client.sendGoal(goal, &doneCb);

  bool finished_before_timeout = action_client.waitForResult(ros::Duration(final_time + 2.0));

  if (!finished_before_timeout)
  {
    ROS_WARN("giving up waiting on result");
    return 0;
  }
  else
  {
    ROS_INFO("main: goal was reported as successfully executed.  Bye.");
  }

  return 0;
}
