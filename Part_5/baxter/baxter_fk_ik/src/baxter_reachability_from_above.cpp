#include <fstream>
#include "baxter_fk_ik/baxter_kinematics.hpp"

int main(int argc, char **argv)
{
  Eigen::Vector3d p;
  Eigen::Vector3d n_des, t_des, b_des;
  Vectorq6x1 q_in;
  q_in << 0, 0, 0, 0, 0, 0;

  Baxter_fwd_solver baxter_fwd_solver;
  Baxter_IK_solver baxter_ik_solver;

  b_des << 0, 0, -1;
  n_des << 0, 0, 1;
  t_des = b_des.cross(n_des);

  Eigen::Matrix3d R_des;
  R_des.col(0) = n_des;
  R_des.col(1) = t_des;
  R_des.col(2) = b_des;

  std::vector<Vectorq7x1> q_solns;
  Vectorq6x1 qsoln;

  Eigen::Affine3d a_tool_des;
  a_tool_des.linear() = R_des;

  double x_des, y_des, z_des;
  double x_min = 0.4;
  double x_max = 1.5;
  double y_min = -1.5;
  double y_max = 1.0;
  double z_low = 0.0;
  double z_high = 0.1;
  double dx = 0.05;
  double dy = 0.05;
  Eigen::Vector3d p_des;
  int nsolns;
  std::vector<Eigen::Vector3d> reachable, approachable;

  for (double x_des = x_min; x_des < x_max; x_des += dx)
  {
    for (double y_des = y_min; y_des < y_max; y_des += dy)
    {
      p_des[0] = x_des;
      p_des[1] = y_des;
      p_des[2] = z_low;
      a_tool_des.translation() = p_des;

      nsolns = baxter_ik_solver.ik_solve_approx_wrt_torso(a_tool_des, q_solns);

      if (nsolns > 0)
      {
        ROS_INFO("soln at x,y = %f, %f", p_des[0], p_des[1]);
        reachable.push_back(p_des);
      }
      p_des[2] = z_high;
      a_tool_des.translation() = p_des;

      nsolns = baxter_ik_solver.ik_solve_approx_wrt_torso(a_tool_des, q_solns);

      if (nsolns > 0)
      {
        ROS_INFO("soln at x,y = %f, %f", p_des[0], p_des[1]);
        approachable.push_back(p_des);
      }
    }
  }
  ROS_INFO("saving the results...");
  nsolns = reachable.size();
  std::ofstream outfile;
  outfile.open("reachable_x_y");

  for (int i = 0; i < nsolns; i++)
  {
    p_des = reachable[i];
    outfile << p_des[0] << ", " << p_des[1] << std::endl;
  }
  outfile.close();

  nsolns = approachable.size();
  std::ofstream outfile2;
  outfile2.open("approachable_x_y");

  for (int i = 0; i < nsolns; i++)
  {
    p_des = approachable[i];
    outfile2 << p_des[0] << ", " << p_des[1] << std::endl;
  }
  outfile2.close();

  return 0;
}
