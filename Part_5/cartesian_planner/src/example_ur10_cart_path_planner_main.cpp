

#include <cartesian_planner/ur10_cartesian_planner.h>
#include <eigen3/Eigen/src/Geometry/Transform.h>
#include <ur_fk_ik/ur_kin.h>
#include <fstream>
#include <iostream>

const double z_des = 0.1;
const double y_des = 0.3;
const double x_start = 0.8;
const double x_end = -0.8;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "example_ur10_cart_path_planner_main");
  ros::NodeHandle nh;
  Eigen::VectorXd q_in_vecxd;

  Eigen::Matrix3d R_gripper_horiz, R_gripper_dn;
  Eigen::Vector3d gripper_n_des, gripper_t_des, gripper_b_des;
  Eigen::Vector3d flange_origin_start, flange_origin_end;
  bool found_path = false;

  ofstream outfile;
  outfile.open("ur10_poses.path");

  flange_origin_start << x_start, y_des, z_des;
  flange_origin_end << x_end, y_des, z_des;

  Eigen::Affine3d a_tool_start, a_tool_end;

  std::vector<Eigen::VectorXd> optimal_path;
  Eigen::VectorXd qvec;

  ROS_INFO("instantiating a cartesian planner object: ");

  CartTrajPlanner cartTrajPlanner;

  R_gripper_dn = cartTrajPlanner.get_R_gripper_down();

  a_tool_start.linear() = R_gripper_dn;
  a_tool_start.translation() = flange_origin_start;
  a_tool_end.linear() = R_gripper_dn;
  a_tool_end.translation() = flange_origin_end;

  found_path = cartTrajPlanner.cartesian_path_planner(a_tool_start, a_tool_end, optimal_path);
  int nsteps = optimal_path.size();
  cout << "there are " << nsteps << " layers in the computed optimal path" << endl;
  if (found_path)
  {
    ROS_INFO("found path");
    for (int istep = 0; istep < nsteps; istep++)
    {
      qvec = optimal_path[istep];
      cout << "qvec: " << qvec.transpose() << endl;
      for (int ijnt = 0; ijnt < NJNTS - 1; ijnt++)
      {
        cout << qvec[ijnt] << ", ";
        outfile << qvec[ijnt] << ", ";
      }
      cout << qvec[NJNTS - 1] << endl;
      outfile << qvec[NJNTS - 1] << endl;
    }
  }
  else
  {
    ROS_WARN("no path found");
  }

  outfile.close();
  return 0;
}
