

#include <rrbot/planar_3rbot_kinematics.h>
using namespace std;

Eigen::Matrix4d compute_A_of_DH(double q, double a, double d, double alpha)
{
  Eigen::Matrix4d A;
  Eigen::Matrix3d R;
  Eigen::Vector3d p;

  A = Eigen::Matrix4d::Identity();
  R = Eigen::Matrix3d::Identity();

  double cq = cos(q);
  double sq = sin(q);
  double sa = sin(alpha);
  double ca = cos(alpha);
  R(0, 0) = cq;
  R(0, 1) = -sq * ca;
  R(0, 2) = sq * sa;
  R(1, 0) = sq;
  R(1, 1) = cq * ca;
  R(1, 2) = -cq * sa;

  R(2, 1) = sa;
  R(2, 2) = ca;
  p(0) = a * cq;
  p(1) = a * sq;
  p(2) = d;
  A.block<3, 3>(0, 0) = R;
  A.col(3).head(3) = p;
  return A;
}

Planar_3rbot_fwd_solver::Planar_3rbot_fwd_solver()
{
  Eigen::Matrix3d R_base_link_wrt_world;
  Eigen::Vector3d O_base_link_wrt_world;

  A_base_link_wrt_world_ = Eigen::MatrixXd::Identity(4, 4);
  O_base_link_wrt_world(0) = base_to_frame0_dx;
  O_base_link_wrt_world(1) = base_to_frame0_dy;
  O_base_link_wrt_world(2) = base_to_frame0_dz;

  R_base_link_wrt_world(0, 0) = 0;
  R_base_link_wrt_world(1, 0) = 0;
  R_base_link_wrt_world(2, 0) = 1;

  R_base_link_wrt_world(0, 1) = -1;
  R_base_link_wrt_world(1, 1) = 0;
  R_base_link_wrt_world(2, 1) = 0;

  R_base_link_wrt_world(0, 2) = 0.0;
  R_base_link_wrt_world(1, 2) = -1.0;
  R_base_link_wrt_world(2, 2) = 0.0;

  A_base_link_wrt_world_.block<3, 1>(0, 3) = O_base_link_wrt_world;
  A_base_link_wrt_world_.block<3, 3>(0, 0) = R_base_link_wrt_world;
  A_base_link_wrt_world_inv_ = A_base_link_wrt_world_.inverse();
}

Eigen::Affine3d Planar_3rbot_fwd_solver::fwd_kin_flange_wrt_world_solve(Eigen::VectorXd q_vec)
{
  Eigen::Matrix4d M;
  M = fwd_kin_solve_(q_vec);
  Eigen::Affine3d A(M);
  return A;
}

Eigen::Matrix4d Planar_3rbot_fwd_solver::fwd_kin_solve_(Eigen::VectorXd q_vec)
{
  Eigen::Matrix4d A = Eigen::Matrix4d::Identity();

  Eigen::Matrix4d A_i_iminusi;
  Eigen::Matrix3d R;
  Eigen::Vector3d p;

  for (int i = 0; i < NJNTS; i++)
  {
    A_i_iminusi = compute_A_of_DH(q_vec[i] + DH_q_offsets[i], DH_a_params[i], DH_d_params[i], DH_alpha_params[i]);
    A_mats_[i] = A_i_iminusi;
  }

  A_mat_products_[0] = A_base_link_wrt_world_ * A_mats_[0];

  for (int i = 1; i < NJNTS; i++)
  {
    A_mat_products_[i] = A_mat_products_[i - 1] * A_mats_[i];
  }

  return A_mat_products_[NJNTS - 1];
}

Eigen::MatrixXd Planar_3rbot_fwd_solver::Jacobian(Eigen::VectorXd q_vec)
{
  Eigen::MatrixXd Jacobian(6, NJNTS);

  Eigen::MatrixXd Origins(3, NJNTS);
  Eigen::Matrix4d A4x4_flange;
  Eigen::MatrixXd J_ang(3, NJNTS), J_trans(3, NJNTS);
  Eigen::Vector3d zvec, Oi, wvec, rvec;

  A4x4_flange = fwd_kin_solve_(q_vec);
  wvec = A4x4_flange.block<3, 1>(0, 3);

  zvec = A_base_link_wrt_world_.block<3, 1>(0, 2);

  J_ang.block<3, 1>(0, 0) = zvec;

  Oi << A_base_link_wrt_world_.block<3, 1>(0, 3);
  Origins.block<3, 1>(0, 0) = Oi;
  for (int i = 1; i < NJNTS; i++)
  {
    zvec = A_mat_products_[i - 1].block<3, 1>(0, 2);
    J_ang.block<3, 1>(0, i) = zvec;
    Oi = A_mat_products_[i - 1].block<3, 1>(0, 3);
    Origins.block<3, 1>(0, i) = Oi;
  }

  for (int i = 0; i < NJNTS; i++)
  {
    zvec = J_ang.block<3, 1>(0, i);
    Oi = Origins.col(i);
    rvec = wvec - Oi;

    J_trans.block<3, 1>(0, i) = zvec.cross(rvec);
  }

  Jacobian.block<3, NJNTS>(0, 0) = J_trans;
  Jacobian.block<3, NJNTS>(3, 0) = J_ang;
  return Jacobian;
}

Planar_3rbot_IK_solver::Planar_3rbot_IK_solver()
{
  ROS_INFO("Planar_3rbot_IK_solver constructor");
}

int Planar_3rbot_IK_solver::ik_solve(double dq1_sample_res, Eigen::Affine3d desired_flange_pose_wrt_base,
                                     std::vector<Eigen::Vector3d> &q_solns)
{
  Eigen::Vector3d O_flange_wrt_world = desired_flange_pose_wrt_base.translation();
  q_solns.clear();
  double q1, q2, q3;
  Eigen::Vector3d q_soln_vec;
  int num_solns = 0;

  for (q1 = q_lower_limits[0]; q1 < q_upper_limits[0]; q1 += dq1_sample_res)
  {
    num_solns += solve_for_qsolns_given_q1(O_flange_wrt_world, q1, q_solns);
  }
  return num_solns;
}

int Planar_3rbot_IK_solver::solve_for_qsolns_given_q1(Eigen::Vector3d O_flange_wrt_world, double q1,
                                                      std::vector<Eigen::Vector3d> &q_solns)
{
  double q2, q3;
  Eigen::Vector3d q_soln_vec;
  int num_solns = 0;
  bool valid_q3 = false;
  bool valid_q2 = false;
  std::vector<double> q3_solns;
  if (!fit_q_to_range(q_lower_limits[0], q_upper_limits[0], q1))
  {
    ROS_ERROR("q1 arg is out of range!");
    return 0;
  }

  q3_solns.clear();

  valid_q3 = solve_for_j3_ang(O_flange_wrt_world, q1, q3_solns);
  if (!valid_q3)
    return 0;

  q3 = q3_solns[0];

  valid_q2 = solve_for_j2_ang(O_flange_wrt_world, q1, q3, q2);
  if (valid_q2)
  {
    q_soln_vec[0] = q1;
    q_soln_vec[1] = q2;
    q_soln_vec[2] = q3;
    q_solns.push_back(q_soln_vec);
    num_solns++;
  }

  if (q3_solns.size() > 1)
  {
    q3 = q3_solns[1];
    valid_q2 = solve_for_j2_ang(O_flange_wrt_world, q1, q3, q2);
    if (valid_q2)
    {
      q_soln_vec[0] = q1;
      q_soln_vec[1] = q2;
      q_soln_vec[2] = q3;
      q_solns.push_back(q_soln_vec);
      num_solns++;
    }
  }

  return num_solns;
}

bool Planar_3rbot_IK_solver::solve_for_j3_ang(Eigen::Vector3d O_flange_wrt_world, double q1,
                                              std::vector<double> &q3_solns)
{
  Eigen::Vector3d O_j2_wrt_world;

  Eigen::Matrix4d A3_wrt_2, A2_wrt_1, A1_wrt_0, A1_wrt_world;
  Eigen::Vector3d vec_j2_to_flange;
  A1_wrt_0 = compute_A_of_DH(q1, DH_a1, DH_d1, DH_alpha1);
  A1_wrt_world = A_base_link_wrt_world_ * A1_wrt_0;
  O_j2_wrt_world = A1_wrt_world.block<3, 1>(0, 3);

  vec_j2_to_flange = O_flange_wrt_world - O_j2_wrt_world;

  vec_j2_to_flange(1) = 0.0;

  double d_j2_to_flange = vec_j2_to_flange.norm();

  double den = 2.0 * DH_a2 * DH_a3;
  double num = d_j2_to_flange * d_j2_to_flange - (DH_a2) * (DH_a2)-DH_a3 * DH_a3;

  double c_ang = num / den;
  if (c_ang > 1.0)
  {
    ROS_WARN("flange out of reach at full j3 extension");
    return false;
  }
  double s_ang = sqrt(1 - c_ang * c_ang);
  double q3_a = atan2(s_ang, c_ang);
  q3_solns.clear();

  bool valid_soln = false;
  if (fit_q_to_range(q_lower_limits[2], q_upper_limits[2], q3_a))
  {
    valid_soln = true;
    q3_solns.push_back(q3_a);
  }
  double q3_b = atan2(-s_ang, c_ang);
  if (fit_q_to_range(q_lower_limits[2], q_upper_limits[2], q3_b))
  {
    valid_soln = true;
    q3_solns.push_back(q3_b);
  }
  return valid_soln;
}

bool Planar_3rbot_IK_solver::solve_for_j2_ang(Eigen::Vector3d O_flange_wrt_world, double q1, double q3, double &q2_soln)
{
  Eigen::Vector4d b_vec, y_vec, O_flange_4x1, O_flange_test;
  Eigen::Matrix4d A3_wrt_2, A2_wrt_1, A1_wrt_0;
  double fk_tol = 0.00001;
  A3_wrt_2 = compute_A_of_DH(q3, DH_a3, DH_d3, DH_alpha3);
  A1_wrt_0 = compute_A_of_DH(q1, DH_a1, DH_d1, DH_alpha1);

  b_vec = A3_wrt_2.block<4, 1>(0, 3);

  O_flange_4x1.block<3, 1>(0, 0) = O_flange_wrt_world;
  O_flange_4x1(3) = 1.0;

  y_vec = A1_wrt_0.inverse() * A_base_link_wrt_world_inv_ * O_flange_4x1;

  double K = y_vec(0);
  double A = b_vec(0) + DH_a2;
  double B = -b_vec(1);

  bool valid_solns = false;
  std::vector<double> q_solns;
  valid_solns = solve_K_eq_Acos_plus_Bsin(K, A, B, q_solns);
  if (!valid_solns)
    return false;

  double fk_err;

  q2_soln = q_solns[0];

  if (fit_q_to_range(q_lower_limits[0], q_upper_limits[0], q2_soln))
  {
    A2_wrt_1 = compute_A_of_DH(q2_soln, DH_a2, DH_d2, DH_alpha2);
    O_flange_test = A_base_link_wrt_world_ * A1_wrt_0 * A2_wrt_1 * b_vec;
    fk_err = (O_flange_test - O_flange_4x1).norm();

    if (fk_err < 0.00001)
    {
      return true;
    }
  }

  if (q_solns.size() > 1)
  {
    q2_soln = q_solns[1];

    if (fit_q_to_range(q_lower_limits[0], q_upper_limits[0], q2_soln))
    {
      A2_wrt_1 = compute_A_of_DH(q2_soln, DH_a2, DH_d2, DH_alpha2);
      O_flange_test = A_base_link_wrt_world_ * A1_wrt_0 * A2_wrt_1 * b_vec;
      fk_err = (O_flange_test - O_flange_4x1).norm();

      if (fk_err < 0.00001)
      {
        return true;
      }
    }
  }

  return false;
}

bool Planar_3rbot_IK_solver::fit_q_to_range(double q_min, double q_max, double &q)
{
  while (q < q_min)
  {
    q += 2.0 * M_PI;
  }
  while (q > q_max)
  {
    q -= 2.0 * M_PI;
  }

  if (q < q_min)
    return false;
  else
    return true;
}

bool Planar_3rbot_IK_solver::solve_K_eq_Acos_plus_Bsin(double K, double A, double B, std::vector<double> &q_solns)
{
  double r, cphi, sphi, phi, gamma;
  double KTOL = 0.000001;
  r = sqrt(A * A + B * B);
  phi = atan2(B, A);
  q_solns.clear();
  if (fabs(K) > fabs(r))
  {
    ROS_WARN("K/r is too large for a cos/sin soln");
    return false;
  }

  if (fabs(K) < KTOL)
  {
    ROS_WARN("K is too small for A,B,K trig soln: user error? ");
    return false;
  }
  gamma = acos(K / r);
  double soln1 = phi + gamma;
  double soln2 = phi - gamma;
  q_solns.push_back(soln1);
  q_solns.push_back(soln2);

  double q_soln;

  /* debug...


  double test = A * cos(soln1) + B * sin(soln1);

  if (fabs(test - K) < KTOL) {
      q_soln = soln1;

  }



  test = A * cos(soln2) + B * sin(soln2);
  if (fabs(test - K) < KTOL) {
      q_soln = soln2;

  }
   */

  return true;
}
