

#include <actionlib/server/simple_action_server.h>
#include <baxter_trajectory_streamer/baxter_trajectory_streamer.h>

#include <baxter_trajectory_streamer/trajAction.h>
using namespace std;

ros::Publisher joint_cmd_pub_left;

bool update_trajectory(double traj_clock, trajectory_msgs::JointTrajectory trajectory, Vectorq7x1 qvec_prev,
                       int& isegment, Vectorq7x1& qvec_new)
{
  trajectory_msgs::JointTrajectoryPoint trajectory_point_from, trajectory_point_to;
  Vectorq7x1 qvec, qvec_to, delta_qvec, dqvec;
  int nsegs = trajectory.points.size() - 1;
  double t_subgoal;

  if (isegment < nsegs)
  {
    trajectory_point_to = trajectory.points[isegment + 1];
    t_subgoal = trajectory_point_to.time_from_start.toSec();
  }
  else
  {
    cout << "reached end of last segment" << endl;
    trajectory_point_to = trajectory.points[nsegs];
    t_subgoal = trajectory_point_to.time_from_start.toSec();
    for (int i = 0; i < 7; i++)
    {
      qvec_new[i] = trajectory_point_to.positions[i];
    }
    cout << "final time: " << t_subgoal << endl;
    return false;
  }

  while ((t_subgoal < traj_clock) && (isegment < nsegs))
  {
    isegment++;
    if (isegment > nsegs - 1)
    {
      trajectory_point_to = trajectory.points[nsegs];
      for (int i = 0; i < 7; i++)
      {
        qvec_new[i] = trajectory_point_to.positions[i];
      }
      cout << "iseg>nsegs" << endl;
      return false;
    }

    trajectory_point_to = trajectory.points[isegment + 1];
    t_subgoal = trajectory_point_to.time_from_start.toSec();
  }

  for (int i = 0; i < 7; i++)
  {
    qvec_to[i] = trajectory_point_to.positions[i];
  }
  delta_qvec = qvec_to - qvec_prev;
  double delta_time = t_subgoal - traj_clock;
  if (delta_time < dt_traj)
    delta_time = dt_traj;
  dqvec = delta_qvec * dt_traj / delta_time;
  qvec_new = qvec_prev + dqvec;
  return true;
}

class trajActionServer
{
private:
  ros::NodeHandle nh_;

  actionlib::SimpleActionServer<baxter_trajectory_streamer::trajAction> as_;

  baxter_trajectory_streamer::trajGoal goal_;
  baxter_trajectory_streamer::trajResult result_;
  baxter_trajectory_streamer::trajFeedback feedback_;

  baxter_core_msgs::JointCommand left_cmd;
  trajectory_msgs::JointTrajectory new_trajectory;
  int g_count;
  bool working_on_trajectory;
  void cmd_pose_left(Vectorq7x1 qvec);

public:
  trajActionServer();

  ~trajActionServer(void)
  {
  }

  void executeCB(const actionlib::SimpleActionServer<baxter_trajectory_streamer::trajAction>::GoalConstPtr& goal);
};

trajActionServer::trajActionServer()
  : as_(nh_, "leftArmTrajActionServer", boost::bind(&trajActionServer::executeCB, this, _1), false)

{
  ROS_INFO("in constructor of leftArmTrajActionServer...");

  left_cmd.mode = 1;

  left_cmd.names.push_back("left_s0");
  left_cmd.names.push_back("left_s1");
  left_cmd.names.push_back("left_e0");
  left_cmd.names.push_back("left_e1");
  left_cmd.names.push_back("left_w0");
  left_cmd.names.push_back("left_w1");
  left_cmd.names.push_back("left_w2");

  for (int i = 0; i < 7; i++)
  {
    left_cmd.command.push_back(0.0);
  }
  g_count = 0;
  working_on_trajectory = false;
  as_.start();
}

void trajActionServer::cmd_pose_left(Vectorq7x1 qvec)
{
  for (int i = 0; i < 7; i++)
  {
    left_cmd.command[i] = qvec[i];
  }
  joint_cmd_pub_left.publish(left_cmd);
}

void trajActionServer::executeCB(
    const actionlib::SimpleActionServer<baxter_trajectory_streamer::trajAction>::GoalConstPtr& goal)
{
  double traj_clock, dt_segment, dq_segment, delta_q_segment, traj_final_time;
  int isegment;
  trajectory_msgs::JointTrajectoryPoint trajectory_point0;

  Vectorq7x1 qvec, qvec0, qvec_prev, qvec_new;

  g_count++;
  result_.return_val = g_count;

  new_trajectory = goal->trajectory;

  int npts = new_trajectory.points.size();
  if (npts < 2)
  {
    ROS_WARN("too few points; aborting goal");
    as_.setAborted(result_);
  }
  else
  {
    ROS_INFO("Cb received traj w/ npts = %d", npts);

    /*
    cout << "subgoals: " << endl;
    for (int i = 0; i < npts; i++) {
        for (int j = 0; j < 7; j++) {
            cout << new_trajectory.points[i].positions[j] << ", ";
        }
        cout << endl;
    }
    */
    as_.isActive();

    working_on_trajectory = true;

    traj_clock = 0.0;
    isegment = 0;
    trajectory_point0 = new_trajectory.points[0];

    for (int i = 0; i < 7; i++)
    {
      qvec0[i] = trajectory_point0.positions[i];
    }
    cmd_pose_left(qvec0);
    qvec_prev = qvec0;
    cout << "start pt: " << qvec0.transpose() << endl;
  }
  while (working_on_trajectory)
  {
    traj_clock += dt_traj;

    working_on_trajectory = update_trajectory(traj_clock, new_trajectory, qvec_prev, isegment, qvec_new);
    cmd_pose_left(qvec_new);
    qvec_prev = qvec_new;

    ros::spinOnce();
    ros::Duration(dt_traj).sleep();
  }
  ROS_INFO("completed execution of a trajectory");
  as_.setSucceeded(result_);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "left_arm_traj_interpolator_action_server");
  ros::NodeHandle nh;

  joint_cmd_pub_left = nh.advertise<baxter_core_msgs::JointCommand>("/robot/limb/left/joint_command", 1);

  ROS_INFO("instantiating the trajectory interpolator action server: ");
  trajActionServer as;

  ROS_INFO("left-arm interpolator ready to receive/execute trajectories");

  while (ros::ok())
  {
    ros::spinOnce();
    ros::Duration(dt_traj).sleep();
  }
}
