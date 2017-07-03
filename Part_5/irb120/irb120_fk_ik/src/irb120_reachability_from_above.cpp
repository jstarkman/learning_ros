

#include <irb120_fk_ik/irb120_kinematics.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "irb120_reachability");
  Eigen::Vector3d p;
  Eigen::Vector3d n_des, t_des, b_des;
  Vectorq6x1 q_in;
  q_in << 0, 0, 0, 0, 0, 0;

  Irb120_fwd_solver irb120_fwd_solver;
  Irb120_IK_solver ik_solver;

  b_des << 1, 0, 0;
  t_des << 0, 1, 0;
  n_des = t_des.cross(b_des);

  Eigen::Matrix3d R_des;
  R_des.col(0) = n_des;
  R_des.col(1) = t_des;
  R_des.col(2) = b_des;

  Eigen::Affine3d a_tool;
  a_tool.linear() = R_des;
  a_tool.translation() << 0.3, 0.0, 0.3;

  Eigen::Affine3d A_fwd_DH = irb120_fwd_solver.fwd_kin_solve(q_in);

  std::cout << "q_in: " << q_in.transpose() << std::endl;
  std::cout << "A rot: " << std::endl;
  std::cout << A_fwd_DH.linear() << std::endl;
  std::cout << "A origin: " << A_fwd_DH.translation().transpose() << std::endl;

  int nsolns = ik_solver.ik_solve(A_fwd_DH);
  std::vector<Vectorq6x1> q6dof_solns;
  Vectorq6x1 qsoln;
  ik_solver.get_solns(q6dof_solns);
  std::cout << "IK solns: " << std::endl;
  for (int i = 0; i < nsolns; i++)
  {
    std::cout << (q6dof_solns[i]).transpose();
  }

  Eigen::Affine3d a_tool_des;

  a_tool_des.linear() = R_des;

  double x_des;
  while (true)
  {
    std::cout << std::endl << "enter x_des: ";
    std::cin >> x_des;

    p[0] = x_des;

    std::cout << "====  irb120 kinematics solver ====" << std::endl;
    int ans = 1;
    bool reachable_proposition;
    for (double z_des = 0.9; z_des > -0.4; z_des -= 0.1)
    {
      std::cout << std::endl;
      std::cout << "z=" << round(z_des * 10) << "  ";
      for (double y_des = -1.0; y_des < 1.0; y_des += 0.05)
      {
        p[1] = y_des;
        p[2] = z_des;
        a_tool_des.translation() = p;
        int nsolns = ik_solver.ik_solve(a_tool_des);
        std::cout << nsolns;
      }
    }
  }
  return 0;
}
