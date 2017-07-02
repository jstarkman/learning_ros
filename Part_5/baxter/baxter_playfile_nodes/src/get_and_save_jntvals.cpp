

#include <baxter_trajectory_streamer/baxter_trajectory_streamer.h>
#include <ros/ros.h>
#include <fstream>
#include <iostream>

using namespace std;
#define VECTOR_DIM 7

int main(int argc, char** argv)
{
  ros::init(argc, argv, "traj_action_client_node");
  ros::NodeHandle nh;

  Eigen::VectorXd q_vec_right_arm, q_vec_left_arm;

  ofstream outfile_right, outfile_left;
  outfile_right.open("baxter_r_arm_angs.txt");
  outfile_left.open("baxter_l_arm_angs.txt");
  cout << "instantiating a traj streamer" << endl;

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

  int trigger = 2;
  int npts = 0;
  ROS_INFO("ready to sample and save joint angles to baxter_r_arm_angs.txt and baxter_l_arm_angs.txt");
  while (trigger > 0)
  {
    cout << "enter 1 for a snapshot, 0 to finish: ";
    cin >> trigger;
    if (trigger == 1)
    {
      trigger = 2;
      npts++;
      for (int i = 0; i < 10; i++)
      {
        ros::spinOnce();

        ros::Duration(0.01).sleep();
      }
      q_vec_right_arm = baxter_traj_streamer.get_q_vec_right_arm_Xd();
      cout << "right-arm current state:" << q_vec_right_arm.transpose() << endl;

      q_vec_left_arm = baxter_traj_streamer.get_q_vec_left_arm_Xd();
      cout << "left-arm current state:" << q_vec_left_arm.transpose() << endl;

      outfile_right << q_vec_right_arm[0] << ", " << q_vec_right_arm[1] << ", " << q_vec_right_arm[2] << ", "
                    << q_vec_right_arm[3] << ", " << q_vec_right_arm[4] << ", " << q_vec_right_arm[05] << ", "
                    << q_vec_right_arm[6] << ", " << npts << endl;
      outfile_left << q_vec_left_arm[0] << ", " << q_vec_left_arm[1] << ", " << q_vec_left_arm[2] << ", "
                   << q_vec_left_arm[3] << ", " << q_vec_left_arm[4] << ", " << q_vec_left_arm[05] << ", "
                   << q_vec_left_arm[6] << ", " << npts << endl;
    }
  }
  outfile_right.close();
  outfile_left.close();
  return 0;
}
