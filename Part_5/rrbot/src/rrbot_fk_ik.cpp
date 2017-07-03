

#include <rrbot/rrbot_kinematics.h>
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

Rrbot_fwd_solver::Rrbot_fwd_solver()
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

Eigen::Affine3d Rrbot_fwd_solver::fwd_kin_flange_wrt_world_solve(Eigen::VectorXd q_vec)
{
  Eigen::Matrix4d M;
  M = fwd_kin_solve_(q_vec);
  Eigen::Affine3d A(M);
  return A;
}

Eigen::Matrix4d Rrbot_fwd_solver::fwd_kin_solve_(Eigen::VectorXd q_vec)
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

Eigen::MatrixXd Rrbot_fwd_solver::Jacobian(Eigen::VectorXd q_vec)
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

Rrbot_IK_solver::Rrbot_IK_solver()
{
  ROS_INFO("Rrbot_IK_solver constructor");
}

int Rrbot_IK_solver::ik_solve(Eigen::Affine3d desired_flange_pose_wrt_base, std::vector<Eigen::Vector2d> &q_solns)
{
  Eigen::Vector3d O_flange_wrt_world = desired_flange_pose_wrt_base.translation();
  q_solns.clear();
  double q_elbow, q_shoulder;
  Eigen::Vector2d q_soln_vec;
  int num_solns = 0;
  bool valid_q_elbow = false;
  bool valid_q_shoulder = false;
  std::vector<double> q_elbow_solns;

  valid_q_elbow = solve_for_elbow_ang(O_flange_wrt_world, q_elbow_solns);
  if (!valid_q_elbow)
    return 0;

  q_elbow = q_elbow_solns[0];
  valid_q_shoulder = solve_for_shoulder_ang(O_flange_wrt_world, q_elbow, q_shoulder);
  if (valid_q_shoulder)
  {
    q_soln_vec[0] = q_shoulder;
    q_soln_vec[1] = q_elbow;
    q_solns.push_back(q_soln_vec);
    num_solns = 1;
  }

  if (q_elbow_solns.size() > 1)
  {
    q_elbow = q_elbow_solns[1];
    valid_q_shoulder = solve_for_shoulder_ang(O_flange_wrt_world, q_elbow, q_shoulder);
    if (valid_q_shoulder)
    {
      q_soln_vec[0] = q_shoulder;
      q_soln_vec[1] = q_elbow;
      q_solns.push_back(q_soln_vec);
      num_solns++;
    }
  }
  return num_solns;
}

bool Rrbot_IK_solver::solve_for_elbow_ang(Eigen::Vector3d O_flange_wrt_world, std::vector<double> &q_elbow_solns)
{
  Eigen::Vector3d O_shoulder_wrt_world;
  O_shoulder_wrt_world = A_base_link_wrt_world_.block<3, 1>(0, 3);

  Eigen::Vector3d vec_shoulder_to_flange;
  vec_shoulder_to_flange = O_flange_wrt_world - O_shoulder_wrt_world;

  vec_shoulder_to_flange(1) = 0.0;

  double d_shoulder_to_flange = vec_shoulder_to_flange.norm();

  double den = 2.0 * DH_a1 * DH_a2;
  double num = d_shoulder_to_flange * d_shoulder_to_flange - (DH_a1) * (DH_a1)-DH_a2 * DH_a2;

  double c_ang = num / den;
  if (c_ang > 1.0)
  {
    ROS_WARN("flange out of reach at full elbow extension");
    return false;
  }

  double s_ang = sqrt(1 - c_ang * c_ang);

  double q_elbow_a = atan2(s_ang, c_ang);

  q_elbow_solns.clear();

  bool valid_soln = false;
  if (fit_q_to_range(q_lower_limits[1], q_upper_limits[1], q_elbow_a))
  {
    valid_soln = true;
    q_elbow_solns.push_back(q_elbow_a);
  }
  double q_elbow_b = atan2(-s_ang, c_ang);
  if (fit_q_to_range(q_lower_limits[1], q_upper_limits[1], q_elbow_b))
  {
    valid_soln = true;
    q_elbow_solns.push_back(q_elbow_b);
  }
  return valid_soln;
}

bool Rrbot_IK_solver::solve_for_shoulder_ang(Eigen::Vector3d O_flange_wrt_world, double q_elbow, double &q_shoulder)
{
  Eigen::Vector4d b_vec, y_vec, O_flange_4x1, O_flange_test;
  Eigen::Matrix4d A2_wrt_1, A1_wrt_0;
  double fk_tol = 0.00001;
  A2_wrt_1 = compute_A_of_DH(q_elbow, DH_a2, DH_d2, DH_alpha2);

  b_vec = A2_wrt_1.block<4, 1>(0, 3);

  O_flange_4x1.block<3, 1>(0, 0) = O_flange_wrt_world;
  O_flange_4x1(3) = 1.0;

  y_vec = A_base_link_wrt_world_inv_ * O_flange_4x1;

  double K = y_vec(0);
  double A = b_vec(0) + DH_a1;
  double B = -b_vec(1);

  bool valid_solns = false;
  std::vector<double> q_solns;
  valid_solns = solve_K_eq_Acos_plus_Bsin(K, A, B, q_solns);
  if (!valid_solns)
    return false;

  double fk_err;

  q_shoulder = q_solns[0];

  if (fit_q_to_range(q_lower_limits[0], q_upper_limits[0], q_shoulder))
  {
    A1_wrt_0 = compute_A_of_DH(q_shoulder, DH_a1, DH_d1, DH_alpha1);
    O_flange_test = A_base_link_wrt_world_ * A1_wrt_0 * b_vec;
    fk_err = (O_flange_test - O_flange_4x1).norm();

    if (fk_err < 0.00001)
    {
      return true;
    }
  }

  if (q_solns.size() > 1)
  {
    q_shoulder = q_solns[1];

    if (fit_q_to_range(q_lower_limits[0], q_upper_limits[0], q_shoulder))
    {
      A1_wrt_0 = compute_A_of_DH(q_shoulder, DH_a1, DH_d1, DH_alpha1);
      O_flange_test = A_base_link_wrt_world_ * A1_wrt_0 * b_vec;
      fk_err = (O_flange_test - O_flange_4x1).norm();

      if (fk_err < 0.00001)
      {
        return true;
      }
    }
  }

  return false;
}

bool Rrbot_IK_solver::fit_q_to_range(double q_min, double q_max, double &q)
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

bool Rrbot_IK_solver::solve_K_eq_Acos_plus_Bsin(double K, double A, double B, std::vector<double> &q_solns)
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
