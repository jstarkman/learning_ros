

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <baxter_trajectory_streamer/baxter_trajectory_streamer.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

using namespace std;
#define VECTOR_DIM 7

int g_done_count = 0;

void rightArmDoneCb(const actionlib::SimpleClientGoalState& state,
                    const control_msgs::FollowJointTrajectoryResultConstPtr& result)
{
  ROS_INFO(" rtArmDoneCb: server responded with state [%s]", state.toString().c_str());

  g_done_count++;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "jnt_traj_ctlr_client_node");
  ros::NodeHandle nh;

  Eigen::VectorXd q_goal_right;
  Eigen::VectorXd q_vec_right_arm, q_vec_left_arm;
  std::vector<Eigen::VectorXd> des_path_right, des_path_left;
  trajectory_msgs::JointTrajectory des_trajectory_right, des_trajectory_left;

  cout << "setting goal pose: " << endl;
  q_goal_right.resize(7);
  q_goal_right << 0, 0, 0, 0, 0, 0, 0;

  cout << "goal pose right arm: " << q_goal_right.transpose() << endl;

  Baxter_traj_streamer baxter_traj_streamer(&nh);

  cout << "warming up callbacks..." << endl;
  for (int i = 0; i < 100; i++)
  {
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }

  q_vec_right_arm = baxter_traj_streamer.get_q_vec_right_arm_Xd();
  cout << "right-arm current jnt positions:" << q_vec_right_arm.transpose() << endl;

  q_vec_left_arm = baxter_traj_streamer.get_q_vec_left_arm_Xd();
  cout << "left-arm current state:" << q_vec_left_arm.transpose() << endl;

  cout << "stuffing traj: " << endl;
  trajectory_msgs::JointTrajectoryPoint trajectory_point1;

  trajectory_msgs::JointTrajectory new_trajectory;
  trajectory_point1.positions.clear();

  new_trajectory.points.clear();
  new_trajectory.joint_names.clear();

  new_trajectory.joint_names.push_back("right_s0");
  new_trajectory.joint_names.push_back("right_s1");
  new_trajectory.joint_names.push_back("right_e0");
  new_trajectory.joint_names.push_back("right_e1");
  new_trajectory.joint_names.push_back("right_w0");
  new_trajectory.joint_names.push_back("right_w1");
  new_trajectory.joint_names.push_back("right_w2");

  trajectory_point1.time_from_start = ros::Duration(0);
  for (int i = 0; i < 7; i++)
  {
    trajectory_point1.positions.push_back(q_vec_right_arm[i]);
  }
  new_trajectory.points.push_back(trajectory_point1);

  for (int i = 0; i < 7; i++)
  {
    trajectory_point1.positions[i] = q_goal_right[i];
  }

  cout << "enter move duration, in seconds: ";
  double net_time;
  cin >> net_time;
  trajectory_point1.time_from_start = ros::Duration(net_time);
  new_trajectory.points.push_back(trajectory_point1);

  control_msgs::FollowJointTrajectoryGoal goal_right, goal_left;

  goal_right.trajectory = new_trajectory;

  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> right_arm_action_client(
      "robot/limb/right/follow_joint_trajectory", true);

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

  ROS_INFO("sending goal to right arm: ");
  right_arm_action_client.sendGoal(goal_right, &rightArmDoneCb);

  while (g_done_count < 1)
  {
    ROS_INFO("waiting to finish pre-pose..");
    ros::Duration(1.0).sleep();
  }

  ros::spinOnce();
  cout << "right arm is at: " << baxter_traj_streamer.get_q_vec_right_arm_Xd().transpose() << endl;

  return 0;
}
