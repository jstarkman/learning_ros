

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <baxter_trajectory_streamer/baxter_trajectory_streamer.h>
#include <ros/ros.h>

#include <baxter_trajectory_streamer/trajAction.h>
#include <std_msgs/UInt32.h>
using namespace std;
#define VECTOR_DIM 7

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

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

bool g_got_code_trigger = false;
int g_playfile_code = 0;
bool g_got_good_traj_right = false;
bool g_got_good_traj_left = false;
bool g_right_arm_done = false;
bool g_left_arm_done = false;

void rightArmDoneCb(const actionlib::SimpleClientGoalState& state,
                    const baxter_trajectory_streamer::trajResultConstPtr& result)
{
  ROS_INFO(" rtArmDoneCb: server responded with state [%s]", state.toString().c_str());
  ROS_INFO("got return val = %d", result->return_val);
  g_right_arm_done = true;
}

void leftArmDoneCb(const actionlib::SimpleClientGoalState& state,
                   const baxter_trajectory_streamer::trajResultConstPtr& result)
{
  ROS_INFO(" leftArmDoneCb: server responded with state [%s]", state.toString().c_str());
  ROS_INFO("got return val = %d", result->return_val);
  g_left_arm_done = true;
}

void playfileCB(const std_msgs::msg::UInt32& code_msg)
{
  g_playfile_code = code_msg.data;
  ROS_INFO("received code: %d", g_playfile_code);
  g_got_code_trigger = true;
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
  if (max_record_size > 8)
  {
    ROS_WARN("bad file");
    cout << "The largest record has " << max_record_size << " fields.\n";
    return 1;
  }
  if (min_record_size < 8)
  {
    ROS_WARN("bad file");
    cout << "The smallest record has " << min_record_size << " fields.\n";
    return 1;
  }

  des_trajectory.points.clear();
  des_trajectory.joint_names.clear();

  des_trajectory.header.stamp = ros::Time::now();

  trajectory_msgs::JointTrajectoryPoint trajectory_point;
  trajectory_point.positions.resize(7);
  double t_arrival;
  for (unsigned n = 0; n < data.size(); n++)
  {
    for (int i = 0; i < 7; i++)
    {
      trajectory_point.positions[i] = data[n][i];
    }
    t_arrival = data[n][7];
    trajectory_point.time_from_start = ros::Duration(t_arrival);
    des_trajectory.points.push_back(trajectory_point);
  }
  return 0;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "playfile_jointspace");
  ros::NodeHandle nh;

  Eigen::VectorXd q_right_state, q_right_firstpoint, q_left_state, q_left_firstpoint;
  q_right_firstpoint.resize(7);
  q_left_firstpoint.resize(7);
  Eigen::VectorXd dqvec;
  dqvec.resize(7);
  Vectorq7x1 q_vec_right_arm, q_vec_left_arm;

  std::vector<Eigen::VectorXd> des_path_right, des_path_left;

  trajectory_msgs::JointTrajectory des_trajectory_right, des_trajectory_left;
  trajectory_msgs::JointTrajectory approach_trajectory_right, approach_trajectory_left;
  trajectory_msgs::JointTrajectoryPoint trajectory_point0;

  if (argc < 2)
  {
    ROS_WARN("usage error: no filename listed");
    ROS_WARN("use: rosrun baxter_playfile_nodes baxter_playback rt_fname.jsp [lft_fname.jsp]");
    ROS_WARN("make sure the right (and optional left) playfiles are in the current directory");
    exit(1);
  }
  int num_arms_ctl = 0;
  if (argc == 2)
  {
    num_arms_ctl = 1;
  }
  if (argc == 3)
  {
    num_arms_ctl = 2;
  }

  Baxter_traj_streamer baxter_traj_streamer(&nh);

  for (int i = 0; i < 100; i++)
  {
    ros::spinOnce();

    ros::Duration(0.01).sleep();
  }

  baxter_trajectory_streamer::trajGoal goal_right, goal_left;

  actionlib::SimpleActionClient<baxter_trajectory_streamer::trajAction> right_arm_action_client(
      "rightArmTrajActionServer", true);
  actionlib::SimpleActionClient<baxter_trajectory_streamer::trajAction> left_arm_action_client(
      "leftArmTrajActionServer", true);

  ROS_INFO("waiting for right-arm server: ");
  bool server_exists = right_arm_action_client.waitForServer(ros::Duration(1.0));
  while (!server_exists)
  {
    ROS_WARN("waiting on right-arm server...");
    ros::spinOnce();
    ros::Duration(0.5).sleep();
    server_exists = right_arm_action_client.waitForServer(ros::Duration(1.0));
  }
  ROS_INFO("connected to right-arm action server");

  if (num_arms_ctl == 2)
  {
    ROS_INFO("waiting for left-arm server: ");
    server_exists = left_arm_action_client.waitForServer(ros::Duration(1.0));
    while (!server_exists)
    {
      ROS_WARN("waiting on left-arm server...");
      ros::spinOnce();
      ros::Duration(0.5).sleep();
      server_exists = left_arm_action_client.waitForServer(ros::Duration(1.0));
    }
    ROS_INFO("connected to left-arm action server");
  }

  ros::spinOnce();

  q_vec_right_arm = baxter_traj_streamer.get_qvec_right_arm();
  q_right_state = q_vec_right_arm;
  ROS_INFO_STREAM("right-arm current state:" << q_vec_right_arm.transpose());

  q_vec_left_arm = baxter_traj_streamer.get_qvec_left_arm();
  q_left_state = q_vec_left_arm;
  ROS_INFO_STREAM("left-arm current state:" << q_vec_left_arm.transpose());

  des_path_right.clear();
  des_path_right.push_back(q_right_state);
  des_path_left.clear();
  des_path_left.push_back(q_left_state);

  g_got_good_traj_right = false;

  if (0 == read_traj_file(argv[1], des_trajectory_right))
  {
    ROS_INFO("read file OK");
    g_got_good_traj_right = true;
  }
  else
  {
    ROS_ERROR("could not read right-arm file");
    exit(1);
  }

  g_got_good_traj_left = false;
  if (num_arms_ctl == 2)
  {
    if (0 == read_traj_file(argv[2], des_trajectory_left))
    {
      ROS_INFO("read left-arm file OK");
      g_got_good_traj_left = true;
    }
    else
    {
      ROS_ERROR("could not read right-arm file");
      exit(1);
    }
  }

  if (g_got_good_traj_right)
  {
    trajectory_point0 = des_trajectory_right.points[0];
    for (int i = 0; i < 7; i++)
    {
      q_right_firstpoint[i] = trajectory_point0.positions[i];
    }

    des_path_right.push_back(q_right_firstpoint);

    baxter_traj_streamer.stuff_trajectory_right_arm(des_path_right, approach_trajectory_right);
  }
  if (g_got_good_traj_left)
  {
    trajectory_point0 = des_trajectory_left.points[0];
    for (int i = 0; i < 7; i++)
    {
      q_left_firstpoint[i] = trajectory_point0.positions[i];
    }

    des_path_left.push_back(q_left_firstpoint);

    baxter_traj_streamer.stuff_trajectory_left_arm(des_path_left, approach_trajectory_left);
  }

  g_right_arm_done = true;

  if (g_got_good_traj_right)
  {
    goal_right.trajectory = approach_trajectory_right;
    g_right_arm_done = false;
    right_arm_action_client.sendGoal(goal_right, &rightArmDoneCb);
  }
  g_left_arm_done = true;
  if (g_got_good_traj_left)
  {
    goal_left.trajectory = approach_trajectory_left;
    g_left_arm_done = false;
    left_arm_action_client.sendGoal(goal_left, &leftArmDoneCb);
  }
  while (!g_right_arm_done || !g_left_arm_done)
  {
    ROS_INFO("waiting on arm server(s) to approach start of traj");
    ros::Duration(0.5).sleep();
    ros::spinOnce();
  }

  if (g_got_good_traj_right && (des_trajectory_right.points.size() > 1))
  {
    goal_right.trajectory = des_trajectory_right;
    g_right_arm_done = false;
    right_arm_action_client.sendGoal(goal_right, &rightArmDoneCb);
  }
  if (g_got_good_traj_left && (des_trajectory_left.points.size() > 1))
  {
    goal_left.trajectory = des_trajectory_left;
    g_left_arm_done = false;
    left_arm_action_client.sendGoal(goal_left, &leftArmDoneCb);
  }

  while (!g_right_arm_done || !g_left_arm_done)
  {
    ROS_INFO("waiting on arm server(s) to execute playfile(s)");
    ros::Duration(0.5).sleep();
    ros::spinOnce();
  }

  return 0;
}
