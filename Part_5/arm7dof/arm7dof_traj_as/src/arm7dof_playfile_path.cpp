

#include <ros/ros.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <arm7dof_traj_as/arm7dof_traj_as.h>
#include <arm7dof_traj_as/trajAction.h>

#define VECTOR_DIM 7

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

bool read_and_check_file(ifstream& infile, data_t& data)
{
  infile >> data;

  if (!infile.eof())
  {
    ROS_ERROR("error reading file !");
    return false;
  }

  ROS_INFO_STREAM("CSV file contains" << data.size() << " records");

  unsigned min_record_size = data[0].size();
  unsigned max_record_size = 0;
  for (unsigned n = 0; n < data.size(); n++)
  {
    if (max_record_size < data[n].size())
    {
      max_record_size = data[n].size();
    }
    if (min_record_size > data[n].size())
    {
      min_record_size = data[n].size();
    }
  }
  if (max_record_size > 7)
  {
    ROS_ERROR("bad input file");
    ROS_INFO_STREAM("The largest record has " << max_record_size << " fields.");
    return false;
  }
  if (min_record_size < 7)
  {
    ROS_ERROR("bad input file");
    ROS_INFO_STREAM("The smallest record has " << min_record_size << " fields.");
    return false;
  }

  return true;
}

bool g_traj_is_done = true;
void armDoneCb(const actionlib::SimpleClientGoalState& state, const arm7dof_traj_as::trajResultConstPtr& result)
{
  ROS_INFO(" armDoneCb: server responded with state [%s]", state.toString().c_str());
  ROS_INFO("got return val = %d", result->return_val);
  g_traj_is_done = true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "arm7dof_playfile_path");
  ros::NodeHandle nh;

  Eigen::VectorXd q_pre_pose;
  Eigen::VectorXd q_vec;
  std::vector<Eigen::VectorXd> des_path;
  trajectory_msgs::JointTrajectory des_trajectory;
  bool finished_before_timeout;

  data_t path_data;

  if (argc == 2)
  {
    ifstream infile_path(argv[1]);
    if (!infile_path)
    {
      ROS_ERROR("Error: path file could not be opened; halting");
      exit(1);
    }
    if (!read_and_check_file(infile_path, path_data))
    {
      exit(1);
    }

    infile_path.close();
  }
  else
  {
    ROS_ERROR("must have a path file name as command line argument");
    exit(1);
  }

  ROS_INFO("instantiating a traj streamer");

  Arm7dof_traj_streamer arm7dof_traj_streamer(&nh);

  ROS_INFO("warming up callbacks...");
  for (int i = 0; i < 100; i++)
  {
    ros::spinOnce();

    ros::Duration(0.01).sleep();
  }

  q_vec = arm7dof_traj_streamer.get_q_vec_Xd();

  ROS_INFO_STREAM("arm current state:" << q_vec.transpose());

  int npts = path_data.size();
  cout << "path file has " << npts << " points" << endl;
  des_path.push_back(q_vec);
  for (unsigned n = 0; n < npts; n++)
  {
    for (int i = 0; i < 7; i++)
    {
      q_vec[i] = path_data[n][i];
    }
    cout << "path pt " << n << ": " << q_vec.transpose() << endl;
    des_path.push_back(q_vec);
  }

  cout << "converting path to trajectory" << endl;

  arm7dof_traj_streamer.stuff_trajectory(des_path, des_trajectory);

  cout << "num pts in traj: " << des_trajectory.points.size() << endl;
  arm7dof_traj_as::trajGoal goal;
  goal.trajectory = des_trajectory;

  actionlib::SimpleActionClient<arm7dof_traj_as::trajAction> arm_action_client("trajActionServer", true);

  ROS_INFO("waiting for arm traj action server: ");
  bool server_exists = arm_action_client.waitForServer(ros::Duration(1.0));
  while (!server_exists)
  {
    ROS_WARN("waiting on server...");
    ros::spinOnce();
    ros::Duration(1.0).sleep();
    server_exists = arm_action_client.waitForServer(ros::Duration(1.0));
  }
  ROS_INFO("connected to arm trajectory action server");

  ROS_INFO("sending goal to arm: ");
  g_traj_is_done = false;
  arm_action_client.sendGoal(goal, &armDoneCb);
  while (!g_traj_is_done)
  {
    ros::spinOnce();
    ROS_INFO("waiting for traj completion...");
    ros::Duration(1.0).sleep();
  }
  ROS_INFO("trajectory is done; quitting");

  return 0;
}
