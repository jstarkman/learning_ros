

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <vector>

using namespace std;
#define VECTOR_DIM 6
const double dt_traj = 0.02;
int g_done_count = 0;
int g_done_move = true;
Eigen::VectorXd g_q_vec_arm_Xd;
vector<int> g_arm_joint_indices;
vector<string> g_ur_jnt_names;

double g_qdot_max_vec[] = { 0.1, 0.1, 0.1, 0.1, 0.1, 0.1 };

void set_ur_jnt_names()
{
  g_ur_jnt_names.push_back("shoulder_pan_joint");
  g_ur_jnt_names.push_back("shoulder_lift_joint");
  g_ur_jnt_names.push_back("elbow_joint");
  g_ur_jnt_names.push_back("wrist_1_joint");
  g_ur_jnt_names.push_back("wrist_2_joint");
  g_ur_jnt_names.push_back("wrist_3_joint");
}

double transition_time(Eigen::VectorXd dqvec)
{
  double t_max = fabs(dqvec[0]) / g_qdot_max_vec[0];

  double ti;
  for (int i = 1; i < VECTOR_DIM; i++)
  {
    ti = fabs(dqvec[i]) / g_qdot_max_vec[i];
    if (ti > t_max)
      t_max = ti;
  }
  return t_max;
}

void stuff_trajectory(std::vector<Eigen::VectorXd> qvecs, trajectory_msgs::JointTrajectory& new_trajectory)
{
  trajectory_msgs::JointTrajectoryPoint trajectory_point1;

  trajectory_point1.positions.clear();

  new_trajectory.points.clear();
  new_trajectory.joint_names.clear();
  for (int i = 0; i < VECTOR_DIM; i++)
  {
    new_trajectory.joint_names.push_back(g_ur_jnt_names[i].c_str());
  }

  new_trajectory.header.stamp = ros::Time::now();
  Eigen::VectorXd q_start, q_end, dqvec;
  double del_time;
  double net_time = 0.05;
  q_start = qvecs[0];
  q_end = qvecs[0];
  cout << "stuff_traj: start pt = " << q_start.transpose() << endl;
  ROS_INFO("stuffing trajectory");

  trajectory_point1.time_from_start = ros::Duration(net_time);
  for (int i = 0; i < VECTOR_DIM; i++)
  {
    trajectory_point1.positions.push_back(q_start[i]);
  }
  new_trajectory.points.push_back(trajectory_point1);

  for (int iq = 1; iq < qvecs.size(); iq++)
  {
    q_start = q_end;
    q_end = qvecs[iq];
    dqvec = q_end - q_start;

    del_time = transition_time(dqvec);
    if (del_time < dt_traj)
      del_time = dt_traj;

    net_time += del_time;

    for (int i = 0; i < VECTOR_DIM; i++)
    {
      trajectory_point1.positions[i] = q_end[i];
    }

    trajectory_point1.time_from_start = ros::Duration(net_time);
    new_trajectory.points.push_back(trajectory_point1);
  }

  for (int iq = 1; iq < qvecs.size(); iq++)
  {
    cout << "traj pt: ";
    for (int j = 0; j < VECTOR_DIM; j++)
    {
      cout << new_trajectory.points[iq].positions[j] << ", ";
    }
    cout << endl;
    cout << "arrival time: " << new_trajectory.points[iq].time_from_start.toSec() << endl;
  }
}

void map_arm_joint_indices(vector<string> joint_names)
{
  g_arm_joint_indices.clear();
  int index;
  int n_jnts = VECTOR_DIM;

  std::string j_name;

  for (int j = 0; j < VECTOR_DIM; j++)
  {
    j_name = g_ur_jnt_names[j];
    for (int i = 0; i < n_jnts; i++)
    {
      if (j_name.compare(joint_names[i]) == 0)
      {
        index = i;

        g_arm_joint_indices.push_back(index);
        break;
      }
    }
  }
  cout << "indices of arm joints: " << endl;
  for (int i = 0; i < VECTOR_DIM; i++)
  {
    cout << g_arm_joint_indices[i] << ", ";
  }
  cout << endl;
}

void jointStatesCb(const sensor_msgs::JointState& js_msg)
{
  if (g_arm_joint_indices.size() < 1)
  {
    int njnts = js_msg.position.size();
    ROS_INFO("finding joint mappings for %d jnts", njnts);
    map_arm_joint_indices(js_msg.name);
  }
  for (int i = 0; i < VECTOR_DIM; i++)
  {
    g_q_vec_arm_Xd[i] = js_msg.position[g_arm_joint_indices[i]];
  }
  cout << "CB: q_vec_right_arm: " << g_q_vec_arm_Xd.transpose() << endl;
}

void armDoneCb(const actionlib::SimpleClientGoalState& state,
               const control_msgs::FollowJointTrajectoryResultConstPtr& result)
{
  ROS_INFO(" armDoneCb: server responded with state [%s]", state.toString().c_str());
  g_done_move = true;

  g_done_count++;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ur_traj_client_node");
  ros::NodeHandle nh;

  Eigen::VectorXd q_pre_pose;
  Eigen::VectorXd q_vec_arm;
  g_q_vec_arm_Xd.resize(VECTOR_DIM);

  std::vector<Eigen::VectorXd> des_path;
  trajectory_msgs::JointTrajectory des_trajectory;
  set_ur_jnt_names();

  cout << "setting pre-pose: " << endl;
  q_pre_pose.resize(VECTOR_DIM);
  q_pre_pose << 0, 0, 0, 0, 0, 0;

  ros::Subscriber joint_state_sub = nh.subscribe("/joint_states", 1, jointStatesCb);

  cout << "warming up callbacks..." << endl;
  while (g_arm_joint_indices.size() < 1)
  {
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }

  cout << "current pose:" << g_q_vec_arm_Xd.transpose() << endl;

  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> arm_action_client(
      "/arm_controller/follow_joint_trajectory", true);

  ROS_INFO("waiting for arm-control server: ");
  bool server_exists = arm_action_client.waitForServer(ros::Duration(1.0));
  while (!server_exists)
  {
    ROS_WARN("waiting on arm server...");
    ros::spinOnce();
    ros::Duration(1.0).sleep();
    server_exists = arm_action_client.waitForServer(ros::Duration(1.0));
  }
  ROS_INFO("connected to arm action server");

  int nposes = 6;
  int ans;
  control_msgs::FollowJointTrajectoryGoal goal;

  double q234;
  Eigen::Vector3d bz61;
  for (int i = 0; i < nposes; i++)
  {
    cout << "enter 1: ";
    cin >> ans;
    q_pre_pose[i] = -2.0;
    des_path.clear();
    des_path.push_back(g_q_vec_arm_Xd);
    des_path.push_back(q_pre_pose);
    stuff_trajectory(des_path, des_trajectory);
    goal.trajectory = des_trajectory;

    ROS_INFO("sending goal to  arm: ");
    arm_action_client.sendGoal(goal, &armDoneCb);
    while (g_done_count < 1)
    {
      ROS_INFO("waiting to finish move..");
      ros::spinOnce();
      cout << "arm is at: " << g_q_vec_arm_Xd.transpose() << endl;
      ros::Duration(1.0).sleep();
    }

    ros::spinOnce();
    cout << "arm is at: " << g_q_vec_arm_Xd.transpose() << endl;
  }
  int jnt = 1;
  double qval;
  while (jnt >= 0)
  {
    cout << "enter jnt num, 0 through 6: ";
    cin >> jnt;
    cout << "enter jnt angle: ";
    cin >> qval;
    q_pre_pose[jnt] = qval;
    des_path.clear();
    des_path.push_back(g_q_vec_arm_Xd);
    des_path.push_back(q_pre_pose);
    stuff_trajectory(des_path, des_trajectory);
    goal.trajectory = des_trajectory;

    ROS_INFO("sending goal to  arm: ");
    arm_action_client.sendGoal(goal, &armDoneCb);
    while (g_done_count < 1)
    {
      ROS_INFO("waiting to finish move..");
      ros::spinOnce();
      cout << "arm is at: " << g_q_vec_arm_Xd.transpose() << endl;
      ros::Duration(1.0).sleep();
    }

    ros::spinOnce();
    cout << "arm is at: " << g_q_vec_arm_Xd.transpose() << endl;
  }

  return 0;
}
