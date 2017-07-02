

#include <baxter_trajectory_streamer/baxter_trajectory_streamer.h>
#include <ros/ros.h>
#include <fstream>
#include <iostream>

using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "baxter_recorder_node");
  ros::NodeHandle nh;
  Eigen::VectorXd q_vec_right_arm, q_vec_left_arm;

  ofstream outfile_right, outfile_left;
  outfile_right.open("baxter_r_arm_traj.jsp");
  outfile_left.open("baxter_l_arm_traj.jsp");

  double dt_samp = 0.2;
  double dt_spin = 0.01;
  double dt_inc = 0.0;
  double arrival_time = 0.0;

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

  int ans;
  cout << "enter 1 to start capturing, then move arms in desired trajectory; control-C when done recording: ";
  cin >> ans;
  while (ros::ok())
  {
    ros::spinOnce();
    ros::Duration(dt_spin).sleep();
    dt_inc += dt_spin;
    if (dt_inc >= dt_samp)
    {
      arrival_time += dt_samp;
      dt_inc = 0.0;
      q_vec_right_arm = baxter_traj_streamer.get_q_vec_right_arm_Xd();
      q_vec_left_arm = baxter_traj_streamer.get_q_vec_left_arm_Xd();

      outfile_right << q_vec_right_arm[0] << ", " << q_vec_right_arm[1] << ", " << q_vec_right_arm[2] << ", "
                    << q_vec_right_arm[3] << ", " << q_vec_right_arm[4] << ", " << q_vec_right_arm[5] << ", "
                    << q_vec_right_arm[6] << ", " << arrival_time << endl;
      outfile_left << q_vec_left_arm[0] << ", " << q_vec_left_arm[1] << ", " << q_vec_left_arm[2] << ", "
                   << q_vec_left_arm[3] << ", " << q_vec_left_arm[4] << ", " << q_vec_left_arm[5] << ", "
                   << q_vec_left_arm[6] << ", " << arrival_time << endl;
    }
  }

  outfile_right.close();
  outfile_left.close();
  return 0;
}
