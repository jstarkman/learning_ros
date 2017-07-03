

#include <irb120_fk_ik/irb120_kinematics.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "irb120_kinematics_test_main");

  double euler_R = 01.2192;
  double euler_P = 0.9412;
  double euler_Y = 0.4226;
  Vectorq6x1 q_in;

  q_in << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6;

  Eigen::Affine3d a_tool;
  a_tool.linear() << 0, 0, -1, 0, 1, 0, 1, 0, 0;
  a_tool.translation() << 0.0, 0.0, 0.0;

  Irb120_fwd_solver irb120_fwd_solver;
  Irb120_IK_solver ik_solver;

  std::cout << "==== Test for irb120 kinematics solver ====" << std::endl;
  int ans = 1;
  bool reachable_proposition;
  while (ans)
  {
    double rval;
    for (int i = 0; i < 6; i++)
    {
      rval = ((double)rand()) / RAND_MAX;

      double qval = -M_PI + 2.0 * M_PI * rval;
      q_in[i] = qval;
    }
    reachable_proposition = ik_solver.fit_joints_to_range(q_in);
    if (reachable_proposition)
    {
      Eigen::Affine3d A_fwd_DH = irb120_fwd_solver.fwd_kin_solve(q_in);

      Eigen::Affine3d A_fwd_URDF = A_fwd_DH * a_tool;
      std::cout << "q_in: " << q_in.transpose() << std::endl;

      Eigen::Matrix3d R_flange = A_fwd_URDF.linear();
      Eigen::Matrix4d A_wrist;

      Eigen::Matrix3d R_hand;

      A_wrist = irb120_fwd_solver.get_wrist_frame();

      int nsolns = ik_solver.ik_solve(A_fwd_DH);
      std::cout << "number of IK solutions: " << nsolns << std::endl;

      std::vector<Vectorq6x1> q6dof_solns;
      ik_solver.get_solns(q6dof_solns);
      nsolns = q6dof_solns.size();
      double q_err;
      int i_min = -1;
      std::cout << "found " << nsolns << " solutions:" << std::endl;
      for (int i = 0; i < nsolns; i++)
      {
        Vectorq6x1 q_soln = q6dof_solns[i];
        ik_solver.fit_joints_to_range(q_soln);
        std::cout << q_soln.transpose() << std::endl;
        q6dof_solns[i] = q_soln;
        q_err = (q_in - q_soln).norm();
        if (q_err < 0.000001)
        {
          i_min = i;
        }
      }
      std::cout << "precise fit for soln " << i_min << std::endl << std::endl;
      std::cout << "des fwd kin wrist point: " << A_wrist(0, 3) << ", " << A_wrist(1, 3) << ", " << A_wrist(2, 3)
                << std::endl;
      std::cout << "fwd kin wrist points from these solutions:" << std::endl;
      for (int i = 0; i < nsolns; i++)
      {
        A_fwd_DH = irb120_fwd_solver.fwd_kin_solve(q6dof_solns[i]);
        A_wrist = irb120_fwd_solver.get_wrist_frame();
        std::cout << "fwd kin wrist point: " << A_wrist(0, 3) << ", " << A_wrist(1, 3) << ", " << A_wrist(2, 3)
                  << std::endl;
      }
      std::cout << "enter 1 to continue, 0 to stop: ";
      std::cin >> ans;
    }
  }
  return 0;
}
