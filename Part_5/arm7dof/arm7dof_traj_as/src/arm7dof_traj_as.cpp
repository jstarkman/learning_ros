

#include <actionlib/server/simple_action_server.h>
#include <arm7dof_traj_as/arm7dof_traj_as.h>
#include <arm7dof_traj_as/trajAction.h>
#include <std_msgs/Float64MultiArray.h>
using namespace std;

class TrajActionServer
{
private:
  ros::NodeHandle nh_;

  actionlib::SimpleActionServer<arm7dof_traj_as::trajAction> as_;

  arm7dof_traj_as::trajGoal goal_;
  arm7dof_traj_as::trajResult result_;
  arm7dof_traj_as::trajFeedback feedback_;

  trajectory_msgs::JointTrajectory new_trajectory;
  int g_count;
  bool working_on_trajectory;

  void command_joints(Eigen::VectorXd q_cmd);

  ros::Publisher j0_pub_, j1_pub_, j2_pub_, j3_pub_, j4_pub_, j5_pub_, j6_pub_;
  ros::Publisher vel_loop_jnt_cmd_publisher_;
  std_msgs::msg::Float64MultiArray qdes_msg_;
  void initializePublishers();

public:
  TrajActionServer(ros::NodeHandle &nh);
  ~TrajActionServer(void)
  {
  }

  void executeCB(const actionlib::SimpleActionServer<arm7dof_traj_as::trajAction>::GoalConstPtr &goal);
  bool update_trajectory(double traj_clock, trajectory_msgs::JointTrajectory trajectory, Eigen::VectorXd qvec_prev,
                         int &isegment, Eigen::VectorXd &qvec_new);
};

TrajActionServer::TrajActionServer(ros::NodeHandle &nh)
  : nh_(nh), as_(nh, "trajActionServer", boost::bind(&TrajActionServer::executeCB, this, _1), false)

{
  ROS_INFO("in constructor of TrajActionServer...");
  initializePublishers();

  for (int i = 0; i < 7; i++)
    qdes_msg_.data.push_back(0.0);
  g_count = 0;
  working_on_trajectory = false;
  ROS_INFO("starting action server: trajActionServer ");
  as_.start();
}

void TrajActionServer::initializePublishers()
{
  j0_pub_ = nh_.advertise<std_msgs::msg::Float64>("/arm7dof/joint0_position_controller/command", 1, true);
  j1_pub_ = nh_.advertise<std_msgs::msg::Float64>("/arm7dof/joint1_position_controller/command", 1, true);
  j2_pub_ = nh_.advertise<std_msgs::msg::Float64>("/arm7dof/joint2_position_controller/command", 1, true);
  j3_pub_ = nh_.advertise<std_msgs::msg::Float64>("/arm7dof/joint3_position_controller/command", 1, true);
  j4_pub_ = nh_.advertise<std_msgs::msg::Float64>("/arm7dof/joint4_position_controller/command", 1, true);
  j5_pub_ = nh_.advertise<std_msgs::msg::Float64>("/arm7dof/joint5_position_controller/command", 1, true);
  j6_pub_ = nh_.advertise<std_msgs::msg::Float64>("/arm7dof/joint6_position_controller/command", 1, true);
  vel_loop_jnt_cmd_publisher_ = nh_.advertise<std_msgs::msg::Float64MultiArray>("qdes_attractor_vec", 1, true);
}

void TrajActionServer::command_joints(Eigen::VectorXd q_cmd)
{
  std_msgs::msg::Float64 qval_msg;

  qval_msg.data = q_cmd(0);
  j0_pub_.publish(qval_msg);
  qval_msg.data = q_cmd(1);
  j1_pub_.publish(qval_msg);
  qval_msg.data = q_cmd(2);
  j2_pub_.publish(qval_msg);
  qval_msg.data = q_cmd(3);
  j3_pub_.publish(qval_msg);
  qval_msg.data = q_cmd(4);
  j4_pub_.publish(qval_msg);
  qval_msg.data = q_cmd(5);
  j5_pub_.publish(qval_msg);
  qval_msg.data = q_cmd(6);
  j6_pub_.publish(qval_msg);
  for (int i = 0; i < 7; i++)
    qdes_msg_.data[i] = q_cmd(i);
  vel_loop_jnt_cmd_publisher_.publish(qdes_msg_);
}

bool TrajActionServer::update_trajectory(double traj_clock, trajectory_msgs::JointTrajectory trajectory,
                                         Eigen::VectorXd qvec_prev, int &isegment, Eigen::VectorXd &qvec_new)
{
  trajectory_msgs::JointTrajectoryPoint trajectory_point_from, trajectory_point_to;
  int njnts = qvec_prev.size();

  Eigen::VectorXd qvec, qvec_to, delta_qvec, dqvec;
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

    for (int i = 0; i < njnts; i++)
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

      for (int i = 0; i < njnts; i++)
      {
        qvec_new[i] = trajectory_point_to.positions[i];
      }

      return false;
    }

    trajectory_point_to = trajectory.points[isegment + 1];
    t_subgoal = trajectory_point_to.time_from_start.toSec();
  }

  qvec_to.resize(njnts);
  for (int i = 0; i < njnts; i++)
  {
    qvec_to[i] = trajectory_point_to.positions[i];
  }
  delta_qvec.resize(njnts);
  delta_qvec = qvec_to - qvec_prev;
  double delta_time = t_subgoal - traj_clock;
  if (delta_time < dt_traj)
    delta_time = dt_traj;
  dqvec.resize(njnts);
  dqvec = delta_qvec * dt_traj / delta_time;
  qvec_new = qvec_prev + dqvec;
  return true;
}

void TrajActionServer::executeCB(const actionlib::SimpleActionServer<arm7dof_traj_as::trajAction>::GoalConstPtr &goal)
{
  double traj_clock, dt_segment, dq_segment, delta_q_segment, traj_final_time;
  int isegment;
  trajectory_msgs::JointTrajectoryPoint trajectory_point0;
  std_msgs::msg::Float64 float64_msg;
  Eigen::VectorXd qvec, qvec0, qvec_prev, qvec_new;
  Vectorq7x1 q_vec7x1;
  Eigen::Affine3d affine_arm;
  Eigen::Vector3d Origin;

  ROS_INFO("in executeCB");

  g_count++;
  result_.return_val = g_count;

  cout << "received trajectory w/ " << goal->trajectory.points.size() << " points" << endl;

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

    cout << "subgoals: " << endl;
    int njnts;
    for (int i = 0; i < npts; i++)
    {
      njnts = new_trajectory.points[i].positions.size();
      cout << "njnts: " << njnts << endl;
      for (int j = 0; j < njnts; j++)
      {
        cout << new_trajectory.points[i].positions[j] << ", ";
      }
      cout << endl;
      cout << "time from start: " << new_trajectory.points[i].time_from_start.toSec() << endl;
      cout << endl;
    }

    as_.isActive();

    working_on_trajectory = true;

    traj_clock = 0.0;
    isegment = 0;
    trajectory_point0 = new_trajectory.points[0];
    njnts = new_trajectory.points[0].positions.size();
    int njnts_new;
    qvec_prev.resize(njnts);
    qvec_new.resize(njnts);
    ROS_INFO("populating qvec_prev: ");
    for (int i = 0; i < njnts; i++)
    {
      qvec_prev[i] = trajectory_point0.positions[i];
    }

    cout << "start pt: " << qvec_prev.transpose() << endl;
  }
  while (working_on_trajectory)
  {
    traj_clock += dt_traj;

    working_on_trajectory = update_trajectory(traj_clock, new_trajectory, qvec_prev, isegment, qvec_new);

    command_joints(qvec_new);
    qvec_prev = qvec_new;

    ros::spinOnce();
    ros::Duration(dt_traj).sleep();
  }
  ROS_INFO("completed execution of a trajectory");
  as_.setSucceeded(result_);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "arm7dof_trajectory_action_server");
  ros::NodeHandle nh;

  ROS_INFO("instantiating the trajectory interpolator action server: ");
  TrajActionServer as(nh);

  ROS_INFO("ready to receive/execute trajectories");

  while (ros::ok())
  {
    ros::spinOnce();
    ros::Duration(dt_traj).sleep();
  }
}
