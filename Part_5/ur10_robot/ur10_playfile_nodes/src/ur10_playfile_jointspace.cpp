

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <ur_fk_ik/ur_kin.h>

#include <Eigen/Dense>
#include <Eigen/Eigen>
using namespace std;
#define VECTOR_DIM 6

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace std;
typedef vector<double> record_t;
typedef vector<record_t> data_t;

istream& operator>>(istream& ins, record_t& record)
{
  record.clear();

  string line;
  getline(ins, line);

  stringstream ss(line);
  string field;
  while (getline(ss, field, ','))
  {
    stringstream fs(field);
    double f = 0.0;
    fs >> f;

    record.push_back(f);
  }

  return ins;
}

istream& operator>>(istream& ins, data_t& data)
{
  data.clear();

  record_t record;
  while (ins >> record)
  {
    data.push_back(record);
  }

  return ins;
}

const double dt_traj = 0.02;
int g_done_count = 0;
int g_done_move = true;
Eigen::VectorXd g_q_vec_arm_Xd;
vector<int> g_arm_joint_indices;
vector<string> g_ur_jnt_names;

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

  Eigen::VectorXd q_start, q_end, dqvec;
  double del_time;
  double net_time = 0.0;
  q_start = qvecs[0];
  q_end = qvecs[0];

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
}

void armDoneCb(const actionlib::SimpleClientGoalState& state,
               const control_msgs::FollowJointTrajectoryResultConstPtr& result)
{
  ROS_INFO(" armDoneCb: server responded with state [%s]", state.toString().c_str());
  g_done_move = true;

  g_done_count++;
}

int read_traj_file(string fname, trajectory_msgs::JointTrajectory& des_trajectory)
{
  ifstream infile(fname.c_str());
  if (infile.is_open())
  {
    ROS_INFO("opened file");
  }
  if (!infile)
  {
    cerr << "Error: file " << fname << " could not be opened" << endl;
    return 1;
  }
  cout << "opened file " << fname << endl;

  data_t data;

  infile >> data;

  if (!infile.eof())
  {
    cout << "error reading file!\n";
    return 1;
  }

  infile.close();

  cout << "CSV file contains " << data.size() << " records.\n";

  unsigned min_record_size = data[0].size();
  unsigned max_record_size = 0;
  for (unsigned n = 0; n < data.size(); n++)
  {
    if (max_record_size < data[n].size())
      max_record_size = data[n].size();
    if (min_record_size > data[n].size())
      min_record_size = data[n].size();
  }
  if (max_record_size > VECTOR_DIM + 1)
  {
    ROS_WARN("bad file");
    cout << "The largest record has " << max_record_size << " fields.\n";
    return 1;
  }
  if (min_record_size < VECTOR_DIM + 1)
  {
    ROS_WARN("bad file");
    cout << "The smallest record has " << min_record_size << " fields.\n";
    return 1;
  }

  des_trajectory.points.clear();
  des_trajectory.joint_names.clear();

  des_trajectory.header.stamp = ros::Time::now();

  trajectory_msgs::JointTrajectoryPoint trajectory_point;
  trajectory_point.positions.resize(VECTOR_DIM);
  double t_arrival;
  for (unsigned n = 0; n < data.size(); n++)
  {
    for (int i = 0; i < VECTOR_DIM; i++)
    {
      trajectory_point.positions[i] = data[n][i];
    }
    t_arrival = data[n][VECTOR_DIM];
    trajectory_point.time_from_start = ros::Duration(t_arrival);
    des_trajectory.points.push_back(trajectory_point);
  }
  return 0;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "playfile_jointspace");
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
  control_msgs::FollowJointTrajectoryGoal goal;

  ros::spinOnce();
  des_path.clear();
  des_path.push_back(g_q_vec_arm_Xd);

  bool got_good_traj = false;

  if (0 == read_traj_file(argv[1], des_trajectory))
  {
    ROS_INFO("read file OK");
    got_good_traj = true;
  }
  else
  {
    ROS_ERROR("could not read playfile");
    exit(1);
  }

  Eigen::VectorXd q_firstpoint;
  trajectory_msgs::JointTrajectory approach_trajectory;
  trajectory_msgs::JointTrajectoryPoint trajectory_point0;
  q_firstpoint.resize(VECTOR_DIM);
  trajectory_point0 = des_trajectory.points[0];
  for (int i = 0; i < VECTOR_DIM; i++)
  {
    q_firstpoint[i] = trajectory_point0.positions[i];
  }

  des_path.push_back(q_firstpoint);

  stuff_trajectory(des_path, approach_trajectory);

  g_done_move = true;

  goal.trajectory = approach_trajectory;
  g_done_move = false;
  arm_action_client.sendGoal(goal, &armDoneCb);

  while (!g_done_move)
  {
    ROS_INFO("waiting on arm server to approach start of traj");
    ros::Duration(0.5).sleep();
    ros::spinOnce();
  }

  if (got_good_traj && (des_trajectory.points.size() > 1))
  {
    goal.trajectory = des_trajectory;
    g_done_move = false;
    arm_action_client.sendGoal(goal, &armDoneCb);
  }

  while (!g_done_move)
  {
    ROS_INFO("waiting on arm server to execute playfile");
    ros::Duration(0.5).sleep();
    ros::spinOnce();
  }

  return 0;
}
