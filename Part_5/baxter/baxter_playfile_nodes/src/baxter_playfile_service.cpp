

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <baxter_trajectory_streamer/baxter_trajectory_streamer.h>
#include <ros/ros.h>
#include <stdlib.h> /* getenv */

#include <baxter_playfile_nodes/playfileSrv.h>
#include <baxter_trajectory_streamer/trajAction.h>
#include <std_msgs/Int32.h>

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

string g_ros_ws_path;

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

std::string g_path_to_playfiles;
Baxter_traj_streamer* g_baxter_traj_streamer_ptr;
actionlib::SimpleActionClient<baxter_trajectory_streamer::trajAction>* g_left_arm_action_client_ptr;
actionlib::SimpleActionClient<baxter_trajectory_streamer::trajAction>* g_right_arm_action_client_ptr;

bool srv_callback(baxter_playfile_nodes::playfileSrvRequest& request,
                  baxter_playfile_nodes::playfileSrvResponse& response)
{
  ROS_INFO("srv_callback activated");
  std::string fname_full_path;
  int playfile_code = request.playfile_code;

  Eigen::VectorXd q_right_state, q_right_firstpoint, q_left_state, q_left_firstpoint;
  q_right_firstpoint.resize(7);
  q_left_firstpoint.resize(7);

  Vectorq7x1 q_vec_right_arm, q_vec_left_arm;
  baxter_trajectory_streamer::trajGoal goal_right, goal_left;

  std::vector<Eigen::VectorXd> des_path_right, des_path_left;

  trajectory_msgs::JointTrajectory des_trajectory_right, des_trajectory_left;
  trajectory_msgs::JointTrajectory approach_trajectory_right, approach_trajectory_left;
  trajectory_msgs::JointTrajectoryPoint trajectory_point0;

  q_vec_right_arm = g_baxter_traj_streamer_ptr->get_qvec_right_arm();
  q_right_state = q_vec_right_arm;
  q_vec_left_arm = g_baxter_traj_streamer_ptr->get_qvec_left_arm();
  q_left_state = q_vec_left_arm;
  des_path_right.clear();
  des_path_right.push_back(q_right_state);
  des_path_left.clear();
  des_path_left.push_back(q_left_state);

  g_got_good_traj_right = false;
  g_got_good_traj_left = false;
  switch (playfile_code)
  {
    case baxter_playfile_nodes::playfileSrvRequest::PRE_POSE:
      ROS_INFO("case PRE_POSE:  ");

      fname_full_path = g_path_to_playfiles + "pre_pose_right.jsp";

      if (0 == read_traj_file(fname_full_path.c_str(), des_trajectory_right))
      {
        ROS_INFO("read right-arm file OK");
        g_got_good_traj_right = true;
      }
      else
      {
        ROS_ERROR("could not read right-arm file %s", fname_full_path.c_str());

        response.return_code = baxter_playfile_nodes::playfileSrvResponse::DID_NOT_FIND_PLAYFILE;
        return true;
      }

      fname_full_path = g_path_to_playfiles + "pre_pose_left.jsp";
      if (0 == read_traj_file(fname_full_path.c_str(), des_trajectory_left))
      {
        ROS_INFO("read left-arm file OK");
        g_got_good_traj_left = true;
      }
      else
      {
        ROS_ERROR("could not read left-arm file");
        response.return_code = baxter_playfile_nodes::playfileSrvResponse::DID_NOT_FIND_PLAYFILE;
        return true;
      }

      response.return_code = baxter_playfile_nodes::playfileSrvResponse::FOUND_BOTH_ARMS_PLAYFILES;
      break;
    case baxter_playfile_nodes::playfileSrvRequest::DEMO_TRAJ:
      fname_full_path = g_path_to_playfiles + "baxter_r_arm_traj.jsp";

      ROS_INFO("case 1:  baxter_r_arm_traj.jsp and baxter_l_arm_traj.jsp");
      if (0 == read_traj_file(fname_full_path.c_str(), des_trajectory_right))
      {
        ROS_INFO("read right-arm file OK");
        g_got_good_traj_right = true;
      }
      else
      {
        ROS_ERROR("could not read right-arm file");
        response.return_code = baxter_playfile_nodes::playfileSrvResponse::DID_NOT_FIND_PLAYFILE;
        return true;
      }

      fname_full_path = g_path_to_playfiles + "baxter_l_arm_traj.jsp";
      if (0 == read_traj_file(fname_full_path.c_str(), des_trajectory_left))
      {
        ROS_INFO("read left-arm file OK");
        g_got_good_traj_left = true;
      }
      else
      {
        ROS_ERROR("could not read left-arm file");
        response.return_code = baxter_playfile_nodes::playfileSrvResponse::DID_NOT_FIND_PLAYFILE;
        return true;
      }
      response.return_code = baxter_playfile_nodes::playfileSrvResponse::FOUND_BOTH_ARMS_PLAYFILES;
      break;
    case baxter_playfile_nodes::playfileSrvRequest::SHY:

      ROS_INFO("case 2: shy");
      fname_full_path = g_path_to_playfiles + "shy.jsp";
      if (0 == read_traj_file(fname_full_path.c_str(), des_trajectory_right))
      {
        g_got_good_traj_right = true;
      }
      else
      {
        ROS_ERROR("could not read right-arm file");
        response.return_code = baxter_playfile_nodes::playfileSrvResponse::DID_NOT_FIND_PLAYFILE;
        return true;
      }
      response.return_code = baxter_playfile_nodes::playfileSrvResponse::FOUND_RIGHT_ARM_PLAYFILE;

      break;
    case baxter_playfile_nodes::playfileSrvRequest::HUG:
      ROS_INFO("case 3: hug");
      fname_full_path = g_path_to_playfiles + "hug.jsp";
      if (0 == read_traj_file(fname_full_path.c_str(), des_trajectory_right))
      {
        g_got_good_traj_right = true;
      }
      else
      {
        ROS_ERROR("could not read right-arm file");
        response.return_code = baxter_playfile_nodes::playfileSrvResponse::DID_NOT_FIND_PLAYFILE;
        return true;
      }
      response.return_code = baxter_playfile_nodes::playfileSrvResponse::FOUND_RIGHT_ARM_PLAYFILE;

      break;
    case baxter_playfile_nodes::playfileSrvRequest::SHAKE:
      ROS_INFO("case 4: shake");
      fname_full_path = g_path_to_playfiles + "shake.jsp";
      if (0 == read_traj_file(fname_full_path.c_str(), des_trajectory_right))
      {
        g_got_good_traj_right = true;
      }
      else
      {
        ROS_ERROR("could not read right-arm file");
        response.return_code = baxter_playfile_nodes::playfileSrvResponse::DID_NOT_FIND_PLAYFILE;
        return true;
      }
      response.return_code = baxter_playfile_nodes::playfileSrvResponse::FOUND_RIGHT_ARM_PLAYFILE;
      break;
    case baxter_playfile_nodes::playfileSrvRequest::STICK_EM_UP:
      ROS_INFO("case 5: stick_em_up");
      fname_full_path = g_path_to_playfiles + "stick_em_up.jsp";
      if (0 == read_traj_file(fname_full_path.c_str(), des_trajectory_right))
      {
        g_got_good_traj_right = true;
      }
      else
      {
        ROS_ERROR("could not read right-arm file");
        response.return_code = baxter_playfile_nodes::playfileSrvResponse::DID_NOT_FIND_PLAYFILE;
        return true;
      }
      response.return_code = baxter_playfile_nodes::playfileSrvResponse::FOUND_RIGHT_ARM_PLAYFILE;
      break;
    case baxter_playfile_nodes::playfileSrvRequest::WAVE:
      ROS_INFO("case 6:wave");
      fname_full_path = g_path_to_playfiles + "wave.jsp";
      if (0 == read_traj_file(fname_full_path.c_str(), des_trajectory_right))
      {
        g_got_good_traj_right = true;
      }
      else
      {
        ROS_ERROR("could not read right-arm file");
        response.return_code = baxter_playfile_nodes::playfileSrvResponse::DID_NOT_FIND_PLAYFILE;
        return true;
      }
      response.return_code = baxter_playfile_nodes::playfileSrvResponse::FOUND_RIGHT_ARM_PLAYFILE;
      break;
    default:
      ROS_INFO("unknown case");
      response.return_code = baxter_playfile_nodes::playfileSrvResponse::UNKNOWN_CASE;
      return true;
      break;
  }

  if (g_got_good_traj_right)
  {
    trajectory_point0 = des_trajectory_right.points[0];
    for (int i = 0; i < 7; i++)
    {
      q_right_firstpoint[i] = trajectory_point0.positions[i];
    }

    des_path_right.push_back(q_right_firstpoint);

    g_baxter_traj_streamer_ptr->stuff_trajectory_right_arm(des_path_right, approach_trajectory_right);
  }
  if (g_got_good_traj_left)
  {
    trajectory_point0 = des_trajectory_left.points[0];
    for (int i = 0; i < 7; i++)
    {
      q_left_firstpoint[i] = trajectory_point0.positions[i];
    }

    des_path_left.push_back(q_left_firstpoint);

    g_baxter_traj_streamer_ptr->stuff_trajectory_left_arm(des_path_left, approach_trajectory_left);
  }

  g_right_arm_done = true;

  if (g_got_good_traj_right)
  {
    goal_right.trajectory = approach_trajectory_right;
    g_right_arm_done = false;
    g_right_arm_action_client_ptr->sendGoal(goal_right, &rightArmDoneCb);
  }
  g_left_arm_done = true;
  if (g_got_good_traj_left)
  {
    goal_left.trajectory = approach_trajectory_left;
    g_left_arm_done = false;
    g_left_arm_action_client_ptr->sendGoal(goal_left, &leftArmDoneCb);
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
    g_right_arm_action_client_ptr->sendGoal(goal_right, &rightArmDoneCb);
  }
  if (g_got_good_traj_left && (des_trajectory_left.points.size() > 1))
  {
    goal_left.trajectory = des_trajectory_left;
    g_left_arm_done = false;
    g_left_arm_action_client_ptr->sendGoal(goal_left, &leftArmDoneCb);
  }

  while (!g_right_arm_done || !g_left_arm_done)
  {
    ROS_INFO("waiting on arm server(s) to execute playfile(s)");
    ros::Duration(0.5).sleep();
    ros::spinOnce();
  }
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "baxter_playfile_service");
  ros::NodeHandle n;
  std::string ros_ws_path = getenv("ROS_WORKSPACE");

  g_path_to_playfiles = ros_ws_path + "/src/learning_ros/Part_5/baxter/baxter_playfile_nodes/";
  ROS_INFO("using path to jsp files: %s", g_path_to_playfiles.c_str());

  cout << "instantiating a traj streamer" << endl;
  Baxter_traj_streamer baxter_traj_streamer(&n);

  cout << "warming up callbacks..." << endl;
  for (int i = 0; i < 100; i++)
  {
    ros::spinOnce();

    ros::Duration(0.01).sleep();
  }

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

  g_baxter_traj_streamer_ptr = &baxter_traj_streamer;
  g_left_arm_action_client_ptr = &left_arm_action_client;
  g_right_arm_action_client_ptr = &right_arm_action_client;

  ros::ServiceServer service = n.advertiseService("playfile_service", srv_callback);
  ROS_INFO("playfile_service is ready to accept requests");
  ros::spin();

  return 0;
}
