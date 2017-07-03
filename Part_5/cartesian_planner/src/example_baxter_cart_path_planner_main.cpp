

#include <cartesian_planner/baxter_cartesian_planner.h>
#include <eigen3/Eigen/src/Geometry/Transform.h>
#include <fstream>
#include <iostream>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "example_baxter_cart_path_planner_main");
  ros::NodeHandle nh;
  Eigen::VectorXd q_in_vecxd;

  Eigen::Matrix3d R_flange_horiz;
  Eigen::Vector3d flange_n_des, flange_t_des, flange_b_des;
  Eigen::Vector3d flange_origin;
  bool found_path = false;

  Eigen::Affine3d a_toolflange_start, a_toolflange_end;

  std::vector<Eigen::VectorXd> optimal_path;

  ROS_INFO("instantiating a cartesian planner object: ");
  CartTrajPlanner cartTrajPlanner;

  ofstream outfile;
  outfile.open("approachable_poses.dat");
  flange_n_des << 0, 0, 1;
  double flange_theta = 0;
  double L_depart = 0.25;
  double x_des = 0.5;
  double y_des = 0.0;
  double z_des = 0.0;
  flange_b_des << cos(flange_theta), sin(flange_theta), 0;
  flange_t_des = flange_b_des.cross(flange_n_des);
  R_flange_horiz.col(0) = flange_n_des;
  R_flange_horiz.col(1) = flange_t_des;
  R_flange_horiz.col(2) = flange_b_des;

  a_toolflange_start.linear() = R_flange_horiz;
  a_toolflange_end.linear() = R_flange_horiz;
  for (x_des = 0.2; x_des < 1.5; x_des += 0.1)
  {
    for (y_des = -1.5; y_des <= 1.5; y_des += 0.1)
    {
      for (flange_theta = 0.0; flange_theta < 6.28; flange_theta += 0.2)
      {
        flange_b_des << cos(flange_theta), sin(flange_theta), 0;
        flange_t_des = flange_b_des.cross(flange_n_des);
        R_flange_horiz.col(0) = flange_n_des;
        R_flange_horiz.col(1) = flange_t_des;
        R_flange_horiz.col(2) = flange_b_des;
        a_toolflange_start.linear() = R_flange_horiz;
        a_toolflange_end.linear() = R_flange_horiz;

        flange_origin << x_des, y_des, z_des;
        a_toolflange_start.translation() = flange_origin;
        a_toolflange_end.translation() = flange_origin - L_depart * flange_b_des;

        found_path = cartTrajPlanner.cartesian_path_planner(a_toolflange_start, a_toolflange_end, optimal_path);

        if (found_path)
        {
          ROS_INFO("found path; x= %f, y= %f, flange_theta = %f", x_des, y_des, flange_theta);
          outfile << x_des << ", " << y_des << ", " << flange_theta << endl;
        }
        else
        {
          ROS_WARN("no path found; x= %f, y= %f, flange_theta = %f", x_des, y_des, flange_theta);
        }
      }
    }
  }
  outfile.close();
  ROS_INFO("results are in file approachable_poses.dat");
  return 0;
}
