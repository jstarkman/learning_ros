

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <arm7dof_traj_as/arm7dof_traj_as.h>
#include <ros/ros.h>

#include <arm7dof_traj_as/trajAction.h>
using namespace std;
#define VECTOR_DIM 7

void armDoneCb(const actionlib::SimpleClientGoalState& state, const arm7dof_traj_as::trajResultConstPtr& result)
{
  ROS_INFO("armDoneCb: server responded with state [%s]", state.toString().c_str());
  ROS_INFO("got return val = %d", result->return_val);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "traj_action_client_node");
  ros::NodeHandle nh;

  Eigen::VectorXd q_pre_pose;
  Eigen::VectorXd q_vec;
  std::vector<Eigen::VectorXd> des_path;
  trajectory_msgs::JointTrajectory des_trajectory;

  cout << "setting pre-poses: " << endl;
  q_pre_pose.resize(7);
  q_pre_pose << 0, 1, 0, -2, 0, 1, 0;

  cout << "pre-pose: " << q_pre_pose.transpose() << endl;

  ROS_INFO("instantiating a traj streamer");

  Arm7dof_traj_streamer arm7dof_traj_streamer(&nh);

  cout << "warming up callbacks..." << endl;
  for (int i = 0; i < 100; i++)
  {
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }

  cout << "arm is at: " << arm7dof_traj_streamer.get_q_vec_Xd().transpose() << endl;

  q_vec = arm7dof_traj_streamer.get_q_vec_Xd();
  cout << "arm current state:" << q_vec.transpose() << endl;

  des_path.push_back(q_vec);
  des_path.push_back(q_pre_pose);

  cout << "stuffing traj: " << endl;

  arm7dof_traj_streamer.stuff_trajectory(des_path, des_trajectory);

  arm7dof_traj_as::trajGoal goal;

  goal.trajectory = des_trajectory;

  actionlib::SimpleActionClient<arm7dof_traj_as::trajAction> arm_action_client("trajActionServer", true);

  ROS_INFO("waiting for arm server: ");
  bool server_exists = arm_action_client.waitForServer(ros::Duration(1.0));
  while (!server_exists)
  {
    ROS_WARN("waiting on server...");
    ros::spinOnce();
    ros::Duration(1.0).sleep();
    server_exists = arm_action_client.waitForServer(ros::Duration(1.0));
  }
  ROS_INFO("connected to arm action server");

  ROS_INFO("sending goals to arm: ");
  arm_action_client.sendGoal(goal, &armDoneCb);
  bool finished_before_timeout = false;
  finished_before_timeout = arm_action_client.waitForResult(ros::Duration(10.0));

  int jnum;
  double j_ang;
  while (ros::ok())
  {
    cout << "enter jnt number (0 through 6; negative to quit): ";
    cin >> jnum;
    if (jnum < 0)
      return 0;
    cout << "enter angle command: ";
    cin >> j_ang;
    des_path.clear();
    des_path.push_back(q_pre_pose);
    q_pre_pose(jnum) = j_ang;
    des_path.push_back(q_pre_pose);
    arm7dof_traj_streamer.stuff_trajectory(des_path, des_trajectory);
    goal.trajectory = des_trajectory;
    arm_action_client.sendGoal(goal, &armDoneCb);
    finished_before_timeout = arm_action_client.waitForResult(ros::Duration(10.0));
  }

  return 0;
}
