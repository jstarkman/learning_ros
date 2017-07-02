

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <baxter_trajectory_streamer/baxter_trajectory_streamer.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ros/ros.h>

using namespace std;
#define VECTOR_DIM 7

int g_done_count = 0;
int g_done_move = true;

void rightArmDoneCb(const actionlib::SimpleClientGoalState& state,
                    const control_msgs::FollowJointTrajectoryResultConstPtr& result)
{
  ROS_INFO(" rtArmDoneCb: server responded with state [%s]", state.toString().c_str());
  g_done_move = true;

  g_done_count++;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "jnt_traj_ctlr_client_node");
  ros::NodeHandle nh;

  Eigen::VectorXd q_pre_pose_right, q_pre_pose_left;
  Eigen::VectorXd q_vec_right_arm, q_vec_left_arm;
  std::vector<Eigen::VectorXd> des_path_right, des_path_left;
  trajectory_msgs::JointTrajectory des_trajectory_right, des_trajectory_left;

  cout << "setting pre-poses: " << endl;
  q_pre_pose_right.resize(7);
  q_pre_pose_left.resize(7);
  q_pre_pose_right << -0.907528, -0.111813, 2.06622, 1.8737, -1.295, 2.00164, 0;

  q_pre_pose_left << 0.907528, 0.111813, -2.06622, 1.8737, 1.295, 2.00164, -2.87179;
  cout << "pre-pose right: " << q_pre_pose_right.transpose() << endl;

  ROS_INFO("instantiating a traj streamer");
  Baxter_traj_streamer baxter_traj_streamer(&nh);

  cout << "warming up callbacks..." << endl;
  for (int i = 0; i < 100; i++)
  {
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }

  q_vec_right_arm = baxter_traj_streamer.get_q_vec_right_arm_Xd();
  cout << "right-arm current state:" << q_vec_right_arm.transpose() << endl;

  q_vec_left_arm = baxter_traj_streamer.get_q_vec_left_arm_Xd();
  cout << "left-arm current state:" << q_vec_left_arm.transpose() << endl;

  des_path_right.push_back(q_vec_right_arm);
  des_path_right.push_back(q_pre_pose_right);

  des_path_left.push_back(q_vec_left_arm);
  des_path_left.push_back(q_pre_pose_left);

  cout << "stuffing traj: " << endl;

  baxter_traj_streamer.stuff_trajectory_right_arm(des_path_right, des_trajectory_right);
  baxter_traj_streamer.stuff_trajectory_left_arm(des_path_left, des_trajectory_left);

  control_msgs::FollowJointTrajectoryGoal goal_right, goal_left;

  goal_right.trajectory = des_trajectory_right;
  goal_left.trajectory = des_trajectory_left;

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
