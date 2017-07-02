#include "arm7dof_fk_ik/arm7dof_kinematics.hpp"

#define ROS_WARN printf

using namespace std;

Eigen::Matrix4d compute_A_of_DH(int i, double q_ang)
{
  Eigen::Matrix4d A;
  Eigen::Matrix3d R;
  Eigen::Vector3d p;
  double a = DH_a_params[i];
  double d = DH_d_params[i];
  double alpha = DH_alpha_params[i];
  double q = q_ang + DH_q_offsets[i];

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

Arm7dof_fwd_solver::Arm7dof_fwd_solver()
{
  Eigen::Matrix3d R_hand;
  Eigen::Vector3d O_hand;

  O_hand(0) = Lx_hand;
  O_hand(1) = 0.0;
  O_hand(2) = Lz_hand;

  R_hand(0, 0) = cos(theta_yaw_hand);
  R_hand(0, 1) = -sin(theta_yaw_hand);
  R_hand(0, 2) = 0.0;
  R_hand(1, 0) = -R_hand(0, 1);
  R_hand(1, 1) = R_hand(0, 0);
  R_hand(1, 2) = 0.0;
  R_hand(2, 0) = 0.0;
  R_hand(2, 1) = 0.0;
  R_hand(2, 2) = 1.0;

  A_tool_wrt_flange_.linear() = R_hand;
  A_tool_wrt_flange_.translation() = O_hand;
  A_tool_wrt_flange_inv_ = A_tool_wrt_flange_.inverse();
}

Eigen::Affine3d Arm7dof_fwd_solver::fwd_kin_tool_wrt_base_solve(const Vectorq7x1 &q_vec)
{
  Eigen::Affine3d A_flange_wrt_base;
  Eigen::Affine3d A_tool_wrt_base;
  A_flange_wrt_base = fwd_kin_flange_wrt_base_solve(q_vec);
  A_tool_wrt_base = A_flange_wrt_base * A_tool_wrt_flange_;
  return A_tool_wrt_base;
}

Eigen::Affine3d Arm7dof_fwd_solver::fwd_kin_flange_wrt_base_solve(const Vectorq7x1 &q_vec)
{
  Eigen::Matrix4d M;
  M = fwd_kin_solve_(q_vec);
  Eigen::Affine3d A(M);
  return A;
}

Eigen::Vector3d Arm7dof_fwd_solver::get_wrist_point(const Vectorq7x1 &q_vec)
{
  fwd_kin_solve_(q_vec);
  Eigen::Affine3d A(A_mat_products_[5]);
  Eigen::Vector3d w;
  w = A.translation();
  return w;
}

Eigen::Vector3d Arm7dof_fwd_solver::get_wrist_point()
{
  Eigen::Affine3d A(A_mat_products_[5]);
  Eigen::Vector3d w;
  w = A.translation();
  return w;
}

Eigen::Vector3d Arm7dof_fwd_solver::get_wrist_coords_wrt_frame1(const Vectorq7x1 &q_vec)
{
  Eigen::Matrix4d A_shoulder_to_wrist;
  fwd_kin_solve_(q_vec);
  A_shoulder_to_wrist = A_mats_[1] * A_mats_[2] * A_mats_[3] * A_mats_[4];
  Eigen::Vector3d w_wrt_1 = A_shoulder_to_wrist.block<3, 1>(0, 3);
  return w_wrt_1;
}

Eigen::Vector3d Arm7dof_fwd_solver::get_wrist_coords_wrt_frame1()
{
  Eigen::Matrix4d A_shoulder_to_wrist;

  A_shoulder_to_wrist = A_mats_[1] * A_mats_[2] * A_mats_[3] * A_mats_[4];
  Eigen::Vector3d w_wrt_1 = A_shoulder_to_wrist.block<3, 1>(0, 3);
  return w_wrt_1;
}

Eigen::MatrixXd Arm7dof_fwd_solver::Jacobian(Eigen::VectorXd q_vec)
{
  Eigen::MatrixXd Jacobian(6, 7);

  Eigen::MatrixXd Origins(3, 7);
  Eigen::Matrix4d A4x4_flange;
  Eigen::MatrixXd J_ang(3, 7), J_trans(3, 7);
  Eigen::Vector3d zvec, Oi, wvec, rvec;

  A4x4_flange = fwd_kin_solve_(q_vec);
  wvec = A4x4_flange.block<3, 1>(0, 3);

  zvec << 0, 0, 1;

  J_ang.block<3, 1>(0, 0) = zvec;

  Oi << 0, 0, 0;
  Origins.block<3, 1>(0, 0) = Oi;
  for (int i = 1; i < 7; i++)
  {
    zvec = A_mat_products_[i - 1].block<3, 1>(0, 2);
    J_ang.block<3, 1>(0, i) = zvec;
    Oi = A_mat_products_[i - 1].block<3, 1>(0, 3);
    Origins.block<3, 1>(0, i) = Oi;
  }

  for (int i = 0; i < 7; i++)
  {
    zvec = J_ang.block<3, 1>(0, i);
    Oi = Origins.col(i);
    rvec = wvec - Oi;

    J_trans.block<3, 1>(0, i) = zvec.cross(rvec);
  }

  Jacobian.block<3, 7>(0, 0) = J_trans;
  Jacobian.block<3, 7>(3, 0) = J_ang;
  return Jacobian;
}

/*
Eigen::Matrix3d Arm7dof_fwd_solver::get_wrist_Jacobian_3x3(double q_s1, double q_humerus, double q_elbow, double
q_forearm) {
    Vectorq7x1 q_vec;
    for (int i=0;i<7;i++) q_vec(i)=0.0;
    q_vec(1) = q_s1;
    q_vec(2) = q_humerus;
    q_vec(3) = q_elbow;
    q_vec(4) = q_forearm;

    Eigen::Matrix4d A_mats_3dof[5];
    Eigen::Matrix4d A_mat_products_3dof[5];


    Eigen::Matrix4d Ai;
    Eigen::Matrix3d R;

    Eigen::Matrix3d Jw1_trans;
    Eigen::Matrix3d Jw1_ang;
    Eigen::Matrix3d Origins;

    Eigen::Vector3d zvec,rvec,wvec,Oi;


    for (int i=0;i<5;i++) {
        A_mats_3dof[i] = compute_A_of_DH(i+1, q_vec(i+1));
    }

    A_mat_products_3dof[0] = A_mats_3dof[0];


    for (int i=1;i<5;i++) {
        A_mat_products_3dof[i] = A_mat_products_3dof[i-1]*A_mats_3dof[i];
    }
    wvec = A_mat_products_3dof[4].block<3, 1>(0, 3);



    zvec<<0,0,1;
    Jw1_ang.block<3, 1>(0, 0) = zvec;
    Oi<<0,0,0;
    Origins.block<3, 1>(0, 0) = Oi;
    for (int i=1;i<3;i++) {
        zvec = A_mat_products_3dof[i-1].block<3, 1>(0, 2);
        Jw1_ang.block<3, 1>(0, i) = zvec;
        Oi = A_mat_products_3dof[i-1].block<3, 1>(0, 3);
        Origins.block<3, 1>(0, i) = Oi;
    }

    for (int i=0;i<3;i++) {
        zvec = Jw1_ang.block<3, 1>(0, i);
        Oi =Origins.block<3, 1>(0, i);
        rvec = wvec - Oi;



        Jw1_trans.block<3, 1>(0, i) = zvec.cross(rvec);

    }


    return Jw1_trans;
}
 */

Eigen::Matrix4d Arm7dof_fwd_solver::fwd_kin_solve_(const Vectorq7x1 &q_vec)
{
  // Eigen::Matrix4d A = Eigen::Matrix4d::Identity();

  Eigen::Matrix4d A_i_iminusi;
  Eigen::Matrix3d R;
  Eigen::Vector3d p;
  for (int i = 0; i < 7; i++)
  {
    A_i_iminusi = compute_A_of_DH(i, q_vec[i]);
    A_mats_[i] = A_i_iminusi;
  }

  A_mat_products_[0] = A_mats_[0];

  for (int i = 1; i < 7; i++)
  {
    A_mat_products_[i] = A_mat_products_[i - 1] * A_mats_[i];
  }

  return A_mat_products_[6];
}

Eigen::Matrix4d Arm7dof_fwd_solver::fwd_kin_solve_(Eigen::VectorXd q_vec)
{
  // Eigen::Matrix4d A = Eigen::Matrix4d::Identity();

  Eigen::Matrix4d A_i_iminusi;
  Eigen::Matrix3d R;
  Eigen::Vector3d p;
  for (int i = 0; i < 7; i++)
  {
    A_i_iminusi = compute_A_of_DH(i, q_vec[i]);
    A_mats_[i] = A_i_iminusi;
  }

  A_mat_products_[0] = A_mats_[0];

  for (int i = 1; i < 7; i++)
  {
    A_mat_products_[i] = A_mat_products_[i - 1] * A_mats_[i];
  }

  return A_mat_products_[6];
}

Arm7dof_IK_solver::Arm7dof_IK_solver()
{
}

Eigen::Vector3d Arm7dof_IK_solver::get_frame2_origin_of_shoulder_yaw(double q_yaw)
{
  // Eigen::Matrix4d A = Eigen::Matrix4d::Identity();

  Eigen::Matrix4d A_1_wrt_0, A_2_wrt_1, A_2_wrt_0;
  Eigen::Matrix3d R;
  Eigen::Vector3d p;
  A_1_wrt_0 = compute_A_of_DH(0, q_yaw);
  A_2_wrt_1 = compute_A_of_DH(1, 0.0);
  A_2_wrt_0 = A_1_wrt_0 * A_2_wrt_1;
  Eigen::Vector3d O2_wrt_base = A_2_wrt_0.block<3, 1>(0, 3);
  return O2_wrt_base;
}

bool Arm7dof_IK_solver::solve_for_elbow_ang(Eigen::Vector3d w_wrt_0, double q_yaw, std::vector<double> &q_elbow_solns)
{
  Eigen::Vector3d O2_wrt_base;
  O2_wrt_base = get_frame2_origin_of_shoulder_yaw(q_yaw);

  Eigen::Vector3d wrist_wrt_2;
  wrist_wrt_2 = w_wrt_0 - O2_wrt_base;

  double d25 = wrist_wrt_2.norm();

  double den = 2.0 * DH_d3 * DH_d5;
  double num = d25 * d25 - (DH_d4) * (DH_d4)-DH_d3 * DH_d3 - DH_d5 * DH_d5;

  double c4 = num / den;
  if (c4 > 1.0)
  {
    ROS_WARN("w_des out of reach at full elbow extension, q_yaw = %f", q_yaw);
    return false;
  }

  double s4 = sqrt(1 - c4 * c4);

  double q4a = atan2(s4, c4);

  q_elbow_solns.clear();

  bool valid_soln = false;
  if (fit_q_to_range(q_lower_limits[3], q_upper_limits[3], q4a))
  {
    valid_soln = true;
    q_elbow_solns.push_back(q4a);
  }
  double q4b = atan2(-s4, c4);
  if (fit_q_to_range(q_lower_limits[3], q_upper_limits[3], q4b))
  {
    valid_soln = true;
    q_elbow_solns.push_back(q4b);
  }
  return valid_soln;
}

bool Arm7dof_IK_solver::solve_K_eq_Acos_plus_Bsin(double K, double A, double B, std::vector<double> &q_solns)
{
  double r, phi, gamma;
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

  /*
  cout<<"K = "<<K<<endl;
  cout<<"soln1="<<soln1<<endl;
  double test = A*cos(soln1) + B*sin(soln1);
  cout<<"Acos(q1) + Bsin(q1) = "<<test<<endl;
  cout<<"soln2="<<soln2<<endl;
  test = A*cos(soln2) + B*sin(soln2);
  cout<<"Acos(q2) + Bsin(q2) = "<<test<<endl;
   */
  return true;
}

bool Arm7dof_IK_solver::fit_q_to_range(double q_min, double q_max, double &q)
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

bool Arm7dof_IK_solver::solve_for_humerus_ang(Eigen::Vector3d w_wrt_0, double q_yaw, double q_elbow,
                                              std::vector<double> &q_humerus_solns)
{
  Eigen::Matrix4d A_1_wrt_0, A_w_wrt_1;
  Eigen::Matrix3d R;
  Eigen::Vector3d p, w_wrt_1;
  double SOLN_TOL_Z = 0.001;
  A_1_wrt_0 = compute_A_of_DH(0, q_yaw);
  Eigen::Affine3d affine_1_wrt_0(A_1_wrt_0);
  w_wrt_1 = affine_1_wrt_0.inverse() * w_wrt_0;

  double w_z_wrt_1 = w_wrt_1[2];

  double A, B, K;
  K = w_z_wrt_1 - DH_d2;
  A = DH_d4;
  B = DH_d5 * sin(q_elbow);
  std::vector<double> q_humerus_temp;
  q_humerus_temp.clear();
  bool valid_soln = solve_K_eq_Acos_plus_Bsin(K, A, B, q_humerus_temp);
  if (!valid_soln)
    return false;

  valid_soln = false;

  q_humerus_solns.clear();
  double q_test = q_humerus_temp[0];
  bool does_fit = fit_q_to_range(q_lower_limits[2], q_upper_limits[2], q_test);
  if (does_fit)
  {
    double w_z_test = A * cos(q_test) + B * sin(q_test);

    double w_z_err = fabs(w_z_test - K);

    if (w_z_err < SOLN_TOL_Z)
    {
      q_humerus_solns.push_back(q_test);
      valid_soln = true;
    }
  }

  if (q_humerus_temp.size() > 1)
  {
    q_test = q_humerus_temp[1];
    does_fit = fit_q_to_range(q_lower_limits[2], q_upper_limits[2], q_test);
    if (does_fit)
    {
      double w_z_test = DH_d5 * sin(q_elbow) * sin(q_test) + DH_d4 * cos(q_test) + DH_d2;
      double w_z_err = fabs(w_z_test - w_z_wrt_1);

      if (w_z_err < SOLN_TOL_Z)
      {
        q_humerus_solns.push_back(q_test);
        valid_soln = true;
      }
    }
  }
  return valid_soln;
}

bool Arm7dof_IK_solver::solve_for_shoulder_pitch_ang(Eigen::Vector3d w_wrt_0, double q_yaw, double q_humerus,
                                                     double q_elbow, std::vector<double> &q_shoulder_solns)
{
  Eigen::Matrix4d A_1_wrt_0, A_w_wrt_1;
  Eigen::Matrix3d R;
  Eigen::Vector3d p, w_wrt_1;
  double SOLN_TOL_Z = 0.001;
  A_1_wrt_0 = compute_A_of_DH(0, q_yaw);
  Eigen::Affine3d affine_1_wrt_0(A_1_wrt_0);
  w_wrt_1 = affine_1_wrt_0.inverse() * w_wrt_0;

  double w_x_wrt_1 = w_wrt_1[0];

  double A, B, K;
  K = w_x_wrt_1;
  A = -DH_d5 * cos(q_humerus) * sin(q_elbow) + DH_d4 * sin(q_humerus);
  B = -DH_d5 * cos(q_elbow) - DH_d3;
  std::vector<double> q_shoulder_temp;
  q_shoulder_temp.clear();
  bool valid_soln = solve_K_eq_Acos_plus_Bsin(K, A, B, q_shoulder_temp);
  if (!valid_soln)
    return false;

  valid_soln = false;

  q_shoulder_solns.clear();
  double q_test = q_shoulder_temp[0];
  bool does_fit = fit_q_to_range(q_lower_limits[1], q_upper_limits[1], q_test);
  if (does_fit)
  {
    double w_x_test = A * cos(q_test) + B * sin(q_test);

    double w_x_err = fabs(w_x_test - K);

    if (w_x_err < SOLN_TOL_Z)
    {
      q_shoulder_solns.push_back(q_test);
      valid_soln = true;
    }
  }

  if (q_shoulder_temp.size() > 1)
  {
    q_test = q_shoulder_temp[1];
    does_fit = fit_q_to_range(q_lower_limits[1], q_upper_limits[1], q_test);
    if (does_fit)
    {
      double w_x_test = -DH_d5 * cos(q_test) * cos(q_humerus) * sin(q_elbow) + DH_d4 * cos(q_test) * sin(q_humerus) -
                        DH_d3 * sin(q_test);
      double w_x_err = fabs(w_x_test - w_x_wrt_1);

      if (w_x_err < SOLN_TOL_Z)
      {
        q_shoulder_solns.push_back(q_test);
        valid_soln = true;
      }
    }
  }
  return valid_soln;
}

bool Arm7dof_IK_solver::ik_wrist_solns_of_q0(Eigen::Vector3d wrist_pt, double q_yaw,
                                             std::vector<Eigen::VectorXd> &q_solns)
{
  bool valid_wrist_soln = false;
  bool valid_q_elbow = false;
  bool valid_q_humerus = false;
  bool valid_q_shoulder = false;
  double q_elbow, q_humerus, q_shoulder;
  std::vector<double> q_elbow_solns;
  std::vector<double> q_humerus_angs;
  std::vector<double> q_shoulder_pitch_angs;

  Eigen::VectorXd q_vec_test;
  Vectorq7x1 q_vec7x1;

  q_vec7x1 << 0, 0, 0, 0, 0, 0, 0;
  q_vec_test = q_vec7x1;

  q_elbow_solns.clear();
  q_solns.clear();

  valid_q_elbow = solve_for_elbow_ang(wrist_pt, q_yaw, q_elbow_solns);

  if (!valid_q_elbow)
    return false;

  Eigen::Vector3d wrist_pt_test_soln, w_soln_err;

  if (q_elbow_solns.size() > 0)
  {
    q_elbow = q_elbow_solns[0];

    q_vec_test[3] = q_elbow;
    q_humerus_angs.clear();
    valid_q_humerus = solve_for_humerus_ang(wrist_pt, q_yaw, q_elbow, q_humerus_angs);
    if (valid_q_humerus)
    {
      q_humerus = q_humerus_angs[0];

      q_shoulder_pitch_angs.clear();
      valid_q_shoulder = solve_for_shoulder_pitch_ang(wrist_pt, q_yaw, q_humerus, q_elbow, q_shoulder_pitch_angs);
      if (valid_q_shoulder)
      {
        q_shoulder = q_shoulder_pitch_angs[0];
        q_vec_test[0] = q_yaw;
        q_vec_test[1] = q_shoulder;
        q_vec_test[2] = q_humerus;
        q_vec_test[3] = q_elbow;
        wrist_pt_test_soln = get_wrist_point(q_vec_test);

        w_soln_err = wrist_pt - wrist_pt_test_soln;

        if (w_soln_err.norm() < W_ERR_TOL)
        {
          q_solns.push_back(q_vec_test);
          valid_wrist_soln = true;
        }
        else
          ROS_WARN("wrist soln err: %f", w_soln_err.norm());

        if (q_shoulder_pitch_angs.size() > 1)
        {
          q_shoulder = q_shoulder_pitch_angs[1];
          q_vec_test[1] = q_shoulder;
          wrist_pt_test_soln = get_wrist_point(q_vec_test);

          w_soln_err = wrist_pt - wrist_pt_test_soln;

          if (w_soln_err.norm() < W_ERR_TOL)
          {
            q_solns.push_back(q_vec_test);
            valid_wrist_soln = true;
          }
          else
            ROS_WARN("wrist soln err: %f", w_soln_err.norm());
        }
      }
      if (q_humerus_angs.size() > 1)
      {
        q_humerus = q_humerus_angs[1];
        q_vec_test[2] = q_humerus;

        valid_q_shoulder = solve_for_shoulder_pitch_ang(wrist_pt, q_yaw, q_humerus, q_elbow, q_shoulder_pitch_angs);
        if (valid_q_shoulder)
        {
          q_shoulder = q_shoulder_pitch_angs[0];
          q_vec_test[1] = q_shoulder;
          wrist_pt_test_soln = get_wrist_point(q_vec_test);

          w_soln_err = wrist_pt - wrist_pt_test_soln;
          if (w_soln_err.norm() < W_ERR_TOL)
          {
            q_solns.push_back(q_vec_test);
            valid_wrist_soln = true;
          }

          if (q_shoulder_pitch_angs.size() > 1)
          {
            q_shoulder = q_shoulder_pitch_angs[1];
            q_vec_test[1] = q_shoulder;
            wrist_pt_test_soln = get_wrist_point(q_vec_test);

            w_soln_err = wrist_pt - wrist_pt_test_soln;
            if (w_soln_err.norm() < W_ERR_TOL)
            {
              q_solns.push_back(q_vec_test);
              valid_wrist_soln = true;
            }
          }
        }
      }
    }

    if (q_elbow_solns.size() > 1)
    {
      q_elbow = q_elbow_solns[1];

      q_humerus_angs.clear();
      valid_q_humerus = solve_for_humerus_ang(wrist_pt, q_yaw, q_elbow, q_humerus_angs);
      if (valid_q_humerus)
      {
        q_humerus = q_humerus_angs[0];

        q_shoulder_pitch_angs.clear();

        valid_q_shoulder = solve_for_shoulder_pitch_ang(wrist_pt, q_yaw, q_humerus, q_elbow, q_shoulder_pitch_angs);
        if (valid_q_shoulder)
        {
          q_shoulder = q_shoulder_pitch_angs[0];
          q_vec_test[0] = q_yaw;
          q_vec_test[1] = q_shoulder;
          q_vec_test[2] = q_humerus;
          q_vec_test[3] = q_elbow;
          wrist_pt_test_soln = get_wrist_point(q_vec_test);

          w_soln_err = wrist_pt - wrist_pt_test_soln;
          if (w_soln_err.norm() < W_ERR_TOL)
          {
            q_solns.push_back(q_vec_test);
            valid_wrist_soln = true;
          }

          if (q_shoulder_pitch_angs.size() > 1)
          {
            q_shoulder = q_shoulder_pitch_angs[1];
            q_vec_test[1] = q_shoulder;
            wrist_pt_test_soln = get_wrist_point(q_vec_test);

            w_soln_err = wrist_pt - wrist_pt_test_soln;
            if (w_soln_err.norm() < W_ERR_TOL)
            {
              q_solns.push_back(q_vec_test);
              valid_wrist_soln = true;
            }
          }
        }
        if (q_humerus_angs.size() > 1)
        {
          q_humerus = q_humerus_angs[1];

          q_vec_test[2] = q_humerus;
          q_shoulder_pitch_angs.clear();
          valid_q_shoulder = solve_for_shoulder_pitch_ang(wrist_pt, q_yaw, q_humerus, q_elbow, q_shoulder_pitch_angs);
          if (valid_q_shoulder)
          {
            q_shoulder = q_shoulder_pitch_angs[0];
            q_vec_test[1] = q_shoulder;
            wrist_pt_test_soln = get_wrist_point(q_vec_test);

            w_soln_err = wrist_pt - wrist_pt_test_soln;
            if (w_soln_err.norm() < W_ERR_TOL)
            {
              q_solns.push_back(q_vec_test);
              valid_wrist_soln = true;
            }

            if (q_shoulder_pitch_angs.size() > 1)
            {
              q_shoulder = q_shoulder_pitch_angs[1];
              q_vec_test[1] = q_shoulder;
              wrist_pt_test_soln = get_wrist_point(q_vec_test);

              w_soln_err = wrist_pt - wrist_pt_test_soln;
              if (w_soln_err.norm() < W_ERR_TOL)
              {
                q_solns.push_back(q_vec_test);
                valid_wrist_soln = true;
              }
            }
          }
        }
      }
    }
  }
  return valid_wrist_soln;
}

Eigen::Vector3d Arm7dof_IK_solver::wrist_pnt_from_flange_frame(Eigen::Affine3d affine_flange_frame)
{
  Eigen::Vector3d flange_z_axis;
  Eigen::Vector3d flange_origin;
  Eigen::Vector3d wrist_pt;
  Eigen::Matrix3d R_flange = affine_flange_frame.linear();

  flange_origin = affine_flange_frame.translation();
  flange_z_axis = R_flange.col(2);
  wrist_pt = flange_origin - flange_z_axis * DH_d7;
  return wrist_pt;
}

bool Arm7dof_IK_solver::solve_spherical_wrist(Vectorq7x1 q_in, Eigen::Matrix3d R_des, std::vector<Vectorq7x1> &q_solns)
{
  bool is_singular = false;
  bool at_least_one_valid_soln = false;
  Eigen::Matrix4d A01, A12, A23, A04;
  Eigen::Matrix4d A45, A05, A56, A06;

  fwd_kin_solve_(q_in);
  A04 = A_mat_products_[3];
  Eigen::Vector3d n6, t6, b6;
  Eigen::Vector3d n5, t5, b5;
  Eigen::Vector3d n4, t4, b4;

  Eigen::Vector3d n_des, b_des;
  n4 = A04.col(0).head(3);
  t4 = A04.col(1).head(3);
  b4 = A04.col(2).head(3);
  b_des = R_des.col(2);
  n_des = R_des.col(0);
  b5 = b4.cross(b_des);
  double q4, q5, q6;
  Vectorq7x1 q_soln;
  q_solns.clear();

  if (b5.norm() <= 0.001)
  {
    q4 = 0;
    is_singular = true;
    ROS_WARN("WRIST SINGULARITY!");
  }
  else
  {
    double cq4 = b5.dot(-t4);
    double sq4 = b5.dot(n4);
    q4 = atan2(sq4, cq4);
  }

  while (q4 < q_upper_limits[4])
  {
    q4 += M_PI;
  }
  q4 -= M_PI;
  if (q4 < q_lower_limits[4])
  {
    ROS_WARN("no q4 solns");
    return false;
  }

  A45 = compute_A_of_DH(4, q4);
  A05 = A04 * A45;
  n5 = A05.col(0).head(3);
  t5 = A05.col(1).head(3);
  double cq5 = b_des.dot(t5);
  double sq5 = b_des.dot(-n5);
  q5 = atan2(sq5, cq5);

  A56 = compute_A_of_DH(5, q5);
  A06 = A05 * A56;
  n6 = A06.col(0).head(3);
  t6 = A06.col(1).head(3);

  double cq6 = n_des.dot(-n6);
  double sq6 = n_des.dot(-t6);
  q6 = atan2(sq6, cq6) + M_PI;

  if (fit_q_to_range(q_lower_limits[4], q_upper_limits[4], q4))
  {
    if (fit_q_to_range(q_lower_limits[5], q_upper_limits[5], q5))
    {
      if (fit_q_to_range(q_lower_limits[6], q_upper_limits[6], q6))
      {
        q_soln = q_in;
        q_soln[4] = q4;
        q_soln[5] = q5;
        q_soln[6] = q6;
        q_solns.push_back(q_soln);
        at_least_one_valid_soln = true;
      }
    }
  }

  q4 -= M_PI;
  q5 *= -1.0;
  q6 += M_PI;
  if (fit_q_to_range(q_lower_limits[4], q_upper_limits[4], q4))
  {
    if (fit_q_to_range(q_lower_limits[5], q_upper_limits[5], q5))
    {
      if (fit_q_to_range(q_lower_limits[6], q_upper_limits[6], q6))
      {
        q_soln = q_in;
        q_soln[4] = q4;
        q_soln[5] = q5;
        q_soln[6] = q6;
        q_solns.push_back(q_soln);
        at_least_one_valid_soln = true;
      }
    }
  }

  return is_singular;
}

int Arm7dof_IK_solver::ik_solve_given_qs0(Eigen::Affine3d const &desired_flange_pose_wrt_base, double q_yaw,
                                          std::vector<Vectorq7x1> &q_solns)
{
  Eigen::Matrix3d Rdes = desired_flange_pose_wrt_base.linear();
  Eigen::Vector3d wrist_pt = wrist_pnt_from_flange_frame(desired_flange_pose_wrt_base);
  std::vector<Vectorq7x1> q_solns_w_wrist;
  std::vector<Eigen::VectorXd> q_wristpt_solns;
  Vectorq7x1 q_soln;
  bool valid_wrist_solns = false;
  int num_solns = 0;

  valid_wrist_solns = ik_wrist_solns_of_q0(wrist_pt, q_yaw, q_wristpt_solns);
  if (!valid_wrist_solns)
  {
    ROS_WARN("no wrist-point solns for this pose and q_yaw= %f", q_yaw);
    return 0;
  }

  int n_wrist_pt_solns = q_wristpt_solns.size();

  for (int i = 0; i < n_wrist_pt_solns; i++)
  {
    q_soln = q_wristpt_solns[i];

    // bool singular_wrist =
    solve_spherical_wrist(q_soln, Rdes, q_solns_w_wrist);
    for (size_t j = 0; j < q_solns_w_wrist.size(); j++)
    {
      q_solns.push_back(q_solns_w_wrist[j]);
      num_solns++;
    }
  }

  return num_solns;
}

int Arm7dof_IK_solver::ik_solns_sampled_qs0(Eigen::Affine3d const &desired_flange_pose_wrt_base,
                                            std::vector<Vectorq7x1> &q_solns)
{
  q_solns.clear();
  int n7dof_solns;
  for (double q_yaw_samp = q_lower_limits[0]; q_yaw_samp < q_upper_limits[0]; q_yaw_samp += DQ_YAW)
  {
    n7dof_solns = ik_solve_given_qs0(desired_flange_pose_wrt_base, q_yaw_samp, q_solns);
    cout << "num solns found = " << n7dof_solns << " at q_yaw = " << q_yaw_samp << endl;
  }
  return q_solns.size();
}
