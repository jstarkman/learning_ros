

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <baxter_trajectory_streamer/baxter_trajectory_streamer.h>
#include <ros/ros.h>

#include <baxter_trajectory_streamer/trajAction.h>
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
  if (max_record_size > 8)
  {
    ROS_ERROR("bad input file");
    ROS_INFO_STREAM("The largest record has " << max_record_size << " fields.");
    return false;
  }
  if (min_record_size < 8)
  {
    ROS_ERROR("bad input file");
    ROS_INFO_STREAM("The smallest record has " << min_record_size << " fields.");
    return false;
  }

  return true;
}

void rightArmDoneCb(const actionlib::SimpleClientGoalState& state,
                    const baxter_trajectory_streamer::trajResultConstPtr& result)
{
  ROS_INFO(" rtArmDoneCb: server responded with state [%s]", state.toString().c_str());
  ROS_INFO("got return val = %d", result->return_val);
}

void leftArmDoneCb(const actionlib::SimpleClientGoalState& state,
                   const baxter_trajectory_streamer::trajResultConstPtr& result)
{
  ROS_INFO(" leftArmDoneCb: server responded with state [%s]", state.toString().c_str());
  ROS_INFO("got return val = %d", result->return_val);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "playfile_jointspace");
  ros::NodeHandle nh;

  Eigen::VectorXd q_pre_pose_right, q_pre_pose_left;
  Eigen::VectorXd q_vec_right_arm, q_vec_left_arm;
  std::vector<Eigen::VectorXd> des_path_right, des_path_left;
  trajectory_msgs::JointTrajectory des_trajectory_right, des_trajectory_left;

  data_t data_right, data_left;
  int num_arms_ctl = 0;

  if (argc == 2)
  {
    num_arms_ctl = 1;

    ifstream infile_right(argv[1]);
    if (!infile_right)
    {
      ROS_ERROR("Error: right-arm file could not be opened; halting");
      exit(1);
    }
    if (!read_and_check_file(infile_right, data_right))
    {
      exit(1);
    }

    infile_right.close();
  }
  else if (argc == 3)
  {
    num_arms_ctl = 2;

    ifstream infile_right(argv[1]);
    if (!infile_right)
    {
      ROS_ERROR("Error: right-arm file could not be opened; halting");
      exit(1);
    }
    if (!read_and_check_file(infile_right, data_right))
    {
      exit(1);
    }

    infile_right.close();

    ifstream infile_left(argv[2]);
    if (!infile_left)
    {
      ROS_ERROR("Error: left-arm file could not be opened; halting");
      exit(1);
    }
    if (!read_and_check_file(infile_left, data_left))
    {
      exit(1);
    }

    infile_left.close();
  }
  else
  {
    ROS_ERROR("must have one or two file names as command line arguments");
    exit(1);
  }

  ROS_INFO("setting pre-poses: ");
  q_pre_pose_right.resize(7);
  q_pre_pose_left.resize(7);

  ROS_INFO("instantiating a traj streamer");
  Baxter_traj_streamer baxter_traj_streamer(&nh);

  ROS_INFO("warming up callbacks...");
  for (int i = 0; i < 100; i++)
  {
    ros::spinOnce();

    ros::Duration(0.01).sleep();
  }

  q_vec_right_arm = baxter_traj_streamer.get_q_vec_right_arm_Xd();
  ROS_INFO_STREAM("right-arm current state:" << q_vec_right_arm.transpose());

  q_vec_left_arm = baxter_traj_streamer.get_q_vec_left_arm_Xd();
  ROS_INFO_STREAM("left-arm current state:" << q_vec_left_arm.transpose());

  des_trajectory_right.points.clear();
  des_trajectory_right.joint_names.clear();

  des_trajectory_right.header.stamp = ros::Time::now();

  des_trajectory_left.points.clear();
  des_trajectory_left.joint_names.clear();
  des_trajectory_left.header.stamp = ros::Time::now();

  trajectory_msgs::JointTrajectoryPoint trajectory_point;
  trajectory_point.positions.resize(7);
  double t_arrival;
  for (unsigned n = 0; n < data_right.size(); n++)
  {
    for (int i = 0; i < 7; i++)
    {
      trajectory_point.positions[i] = data_right[n][i];
    }
    t_arrival = data_right[n][7];
    trajectory_point.time_from_start = ros::Duration(t_arrival);
    des_trajectory_right.points.push_back(trajectory_point);
  }

  if (num_arms_ctl == 2)
  {
    for (unsigned n = 0; n < data_left.size(); n++)
    {
      for (int i = 0; i < 7; i++)
      {
        trajectory_point.positions[i] = data_left[n][i];
      }
      t_arrival = data_left[n][7];
      trajectory_point.time_from_start = ros::Duration(t_arrival);
      des_trajectory_left.points.push_back(trajectory_point);
    }
  }

  baxter_trajectory_streamer::trajGoal goal_right, goal_left;

  goal_right.trajectory = des_trajectory_right;
  goal_left.trajectory = des_trajectory_left;

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
    ros::Duration(1.0).sleep();
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
      ros::Duration(1.0).sleep();
      server_exists = left_arm_action_client.waitForServer(ros::Duration(1.0));
    }
    ROS_INFO("connected to left-arm action server");
  }
  if (num_arms_ctl == 1)
  {
    ROS_INFO("sending goal to right arm: ");
    right_arm_action_client.sendGoal(goal_right, &rightArmDoneCb);
  }
  else
  {
    ROS_INFO("sending goals to left and right arms: ");
    right_arm_action_client.sendGoal(goal_right, &rightArmDoneCb);
    left_arm_action_client.sendGoal(goal_left, &leftArmDoneCb);
  }
  return 0;
}
