

#include <actionlib/server/simple_action_server.h>
#include <example_trajectory/TrajActionAction.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <vector>

using namespace std;

const double dt = 0.01;
const double min_dt = 0.0000001;

int g_count = 0;
bool g_count_failure = false;

class TrajectoryActionServer
{
private:
  ros::NodeHandle nh_;
  ros::Publisher jnt_cmd_publisher_;

  actionlib::SimpleActionServer<example_trajectory::TrajActionAction> as_;

  example_trajectory::TrajActionGoal goal_;
  example_trajectory::TrajActionResult result_;
  example_trajectory::TrajActionFeedback feedback_;

  void send_joint_commands_(vector<double> q_cmd_jnts);

public:
  TrajectoryActionServer(ros::NodeHandle* nodehandle);

  ~TrajectoryActionServer(void)
  {
  }

  void executeCB(const actionlib::SimpleActionServer<example_trajectory::TrajActionAction>::GoalConstPtr& goal);
};

TrajectoryActionServer::TrajectoryActionServer(ros::NodeHandle* nodehandle)
  : nh_(*nodehandle)
  , as_(nh_, "example_traj_action_server", boost::bind(&TrajectoryActionServer::executeCB, this, _1), false)
{
  ROS_INFO("in constructor of TrajectoryActionServer...");

  ROS_INFO("Initializing Publisher");

  jnt_cmd_publisher_ = nh_.advertise<std_msgs::msg::Float64>("pos_cmd", 1, true);
  as_.start();
}

void TrajectoryActionServer::send_joint_commands_(vector<double> q_cmd_jnts)
{
  std_msgs::msg::Float64 q_cmd_msg;
  q_cmd_msg.data = q_cmd_jnts[0];
  jnt_cmd_publisher_.publish(q_cmd_msg);
  ROS_INFO("commanding: %f", q_cmd_jnts[0]);
}

void TrajectoryActionServer::executeCB(
    const actionlib::SimpleActionServer<example_trajectory::TrajActionAction>::GoalConstPtr& goal)
{
  trajectory_msgs::JointTrajectory trajectory = goal->trajectory;

  ros::Rate rate_timer(1 / dt);

  int npts = trajectory.points.size();
  ROS_INFO("received trajectory with %d points", npts);

  int njoints = trajectory.joint_names.size();
  vector<double> q_prev_jnts;
  vector<double> q_next_jnts;
  vector<double> q_cmd_jnts;
  q_prev_jnts.resize(njoints);
  q_next_jnts.resize(njoints);
  q_cmd_jnts.resize(njoints);

  ROS_INFO("trajectory message commands %d joint(s)", njoints);
  double t_final = trajectory.points[npts - 1].time_from_start.toSec();
  ROS_INFO("t_final = %f", t_final);
  double t_previous = 0.0;
  double t_next = trajectory.points[1].time_from_start.toSec();
  double t_stream = 0.0;
  double fraction_of_range = 0.0;
  double q_prev, q_next, q_cmd;

  int ipt_next = 1;
  double t_range = t_next - t_previous;
  if (t_range < min_dt)
  {
    ROS_WARN("time step invalid in trajectory!");
    as_.setAborted(result_);
  }

  q_prev_jnts = trajectory.points[ipt_next - 1].positions;
  q_next_jnts = trajectory.points[ipt_next].positions;
  while (t_stream < t_final)
  {
    while (t_stream > t_next)
    {
      ipt_next++;
      t_previous = t_next;
      t_next = trajectory.points[ipt_next].time_from_start.toSec();
      t_range = t_next - t_previous;
      if (t_range < min_dt)
      {
        ROS_WARN("time step invalid in trajectory!");
        as_.setAborted(result_);
        break;
      }
      q_prev_jnts = trajectory.points[ipt_next - 1].positions;
      q_next_jnts = trajectory.points[ipt_next].positions;
    }

    fraction_of_range = (t_stream - t_previous) / (t_next - t_previous);

    for (int ijnt = 0; ijnt < njoints; ijnt++)
    {
      q_cmd_jnts[ijnt] = q_prev_jnts[ijnt] + fraction_of_range * (q_next_jnts[ijnt] - q_prev_jnts[ijnt]);
    }
    ROS_INFO("t_prev, t_stream, t_next, fraction = %f, %f, %f, %f", t_previous, t_stream, t_next, fraction_of_range);
    ROS_INFO("q_prev, q_cmd, q_next: %f, %f, %f", q_prev_jnts[0], q_cmd_jnts[0], q_next_jnts[0]);

    send_joint_commands_(q_cmd_jnts);

    rate_timer.sleep();
    t_stream += dt;
  }

  q_cmd_jnts = trajectory.points[npts - 1].positions;
  send_joint_commands_(q_cmd_jnts);

  as_.setSucceeded(result_);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "example_trajectory_action_node");

  ros::NodeHandle nh;

  ROS_INFO("main: instantiating an object of type TrajectoryActionServer");
  TrajectoryActionServer as_object(&nh);

  ROS_INFO("going into spin");

  while (ros::ok())
  {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }

  return 0;
}
