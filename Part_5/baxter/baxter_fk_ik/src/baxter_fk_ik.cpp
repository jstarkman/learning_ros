#include "baxter_fk_ik/baxter_kinematics.hpp"

Eigen::Matrix4d compute_A_of_DH(int i, double q_abb)
{
  Eigen::Matrix4d A;
  Eigen::Matrix3d R;
  Eigen::Vector3d p;
  double a = DH_a_params[i];
  double d = DH_d_params[i];
  double alpha = DH_alpha_params[i];
  double q = q_abb + DH_q_offsets[i];

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

Eigen::Matrix4d compute_A_of_DH_approx(int i, double q_abb)
{
  Eigen::Matrix4d A;
  Eigen::Matrix3d R;
  Eigen::Vector3d p;
  double a = DH_a_params_approx[i];
  double d = DH_d_params[i];
  double alpha = DH_alpha_params[i];
  double q = q_abb + DH_q_offsets[i];

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

Baxter_fwd_solver::Baxter_fwd_solver()
{
  A_rarm_mount_to_r_lower_forearm_ = Eigen::Matrix4d::Identity();
  A_rarm_mount_to_r_lower_forearm_(0, 3) = rmount_to_r_lower_forearm_x;
  A_rarm_mount_to_r_lower_forearm_(1, 3) = rmount_to_r_lower_forearm_y;
  A_rarm_mount_to_r_lower_forearm_(2, 3) = rmount_to_r_lower_forearm_z;
  Affine_rarm_mount_to_r_lower_forearm_ = A_rarm_mount_to_r_lower_forearm_;

  A_torso_to_rarm_mount_ = Eigen::Matrix4d::Identity();
  A_torso_to_rarm_mount_(0, 3) = torso_to_rmount_x;
  A_torso_to_rarm_mount_(1, 3) = torso_to_rmount_y;
  A_torso_to_rarm_mount_(2, 3) = torso_to_rmount_z;
  A_torso_to_rarm_mount_(0, 0) = cos(theta_z_arm_mount);
  A_torso_to_rarm_mount_(0, 0) = cos(theta_z_arm_mount);
  A_torso_to_rarm_mount_(1, 1) = cos(theta_z_arm_mount);
  A_torso_to_rarm_mount_(0, 1) = -sin(theta_z_arm_mount);
  A_torso_to_rarm_mount_(1, 0) = -A_torso_to_rarm_mount_(0, 1);
  Affine_torso_to_rarm_mount_ = A_torso_to_rarm_mount_;

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

Eigen::Affine3d Baxter_fwd_solver::fwd_kin_tool_wrt_r_arm_mount_solve(const Vectorq7x1& q_vec)
{
  Eigen::Affine3d A_flange_wrt_r_arm_mount;
  Eigen::Affine3d A_tool_wrt_r_arm_mount;
  A_flange_wrt_r_arm_mount = fwd_kin_flange_wrt_r_arm_mount_solve(q_vec);
  A_tool_wrt_r_arm_mount = A_flange_wrt_r_arm_mount * A_tool_wrt_flange_;
  return A_tool_wrt_r_arm_mount;
}

Eigen::Affine3d Baxter_fwd_solver::fwd_kin_tool_wrt_r_arm_mount_solve(const Vectorq7x1& q_vec,
                                                                      Eigen::Affine3d A_tool_wrt_flange)
{
  Eigen::Affine3d A_flange_wrt_r_arm_mount;
  Eigen::Affine3d A_tool_wrt_r_arm_mount;
  A_tool_wrt_flange_ = A_tool_wrt_flange;
  A_flange_wrt_r_arm_mount = fwd_kin_flange_wrt_r_arm_mount_solve(q_vec);
  A_tool_wrt_r_arm_mount = A_flange_wrt_r_arm_mount * A_tool_wrt_flange_;
  return A_tool_wrt_r_arm_mount;
}

Eigen::Affine3d Baxter_fwd_solver::fwd_kin_tool_wrt_r_arm_mount_solve_approx(const Vectorq7x1& q_vec)
{
  Eigen::Affine3d A_flange_wrt_r_arm_mount;
  Eigen::Affine3d A_tool_wrt_r_arm_mount;
  A_flange_wrt_r_arm_mount = fwd_kin_flange_wrt_r_arm_mount_solve_approx(q_vec);
  A_tool_wrt_r_arm_mount = A_flange_wrt_r_arm_mount * A_tool_wrt_flange_;
  return A_tool_wrt_r_arm_mount;
}

Eigen::Affine3d Baxter_fwd_solver::fwd_kin_tool_wrt_r_arm_mount_solve_approx(const Vectorq7x1& q_vec,
                                                                             Eigen::Affine3d A_tool_wrt_flange)
{
  A_tool_wrt_flange_ = A_tool_wrt_flange;
  Eigen::Affine3d A_flange_wrt_r_arm_mount;
  Eigen::Affine3d A_tool_wrt_r_arm_mount;
  A_flange_wrt_r_arm_mount = fwd_kin_flange_wrt_r_arm_mount_solve_approx(q_vec);
  A_tool_wrt_r_arm_mount = A_flange_wrt_r_arm_mount * A_tool_wrt_flange_;
  return A_tool_wrt_r_arm_mount;
}

Eigen::Affine3d Baxter_fwd_solver::fwd_kin_tool_wrt_torso_solve(const Vectorq7x1& q_vec)
{
  Eigen::Affine3d A_flange_wrt_torso;
  Eigen::Affine3d A_tool_wrt_torso;
  A_flange_wrt_torso = fwd_kin_flange_wrt_torso_solve(q_vec);
  A_tool_wrt_torso = A_flange_wrt_torso * A_tool_wrt_flange_;
  return A_tool_wrt_torso;
}

Eigen::Affine3d Baxter_fwd_solver::fwd_kin_tool_wrt_torso_solve(const Vectorq7x1& q_vec,
                                                                Eigen::Affine3d A_tool_wrt_flange)
{
  A_tool_wrt_flange_ = A_tool_wrt_flange;
  Eigen::Affine3d A_flange_wrt_torso;
  Eigen::Affine3d A_tool_wrt_torso;
  A_flange_wrt_torso = fwd_kin_flange_wrt_torso_solve(q_vec);
  A_tool_wrt_torso = A_flange_wrt_torso * A_tool_wrt_flange_;
  return A_tool_wrt_torso;
}

Eigen::Affine3d Baxter_fwd_solver::fwd_kin_flange_wrt_r_arm_mount_solve(const Vectorq7x1& q_vec)
{
  Eigen::Matrix4d M;
  M = fwd_kin_solve_(q_vec);
  Eigen::Affine3d A(M);
  return A;
}

Eigen::Affine3d Baxter_fwd_solver::fwd_kin_flange_wrt_r_arm_mount_solve_approx(const Vectorq7x1& q_vec)
{
  Eigen::Matrix4d M;
  M = fwd_kin_solve_approx_(q_vec);
  Eigen::Affine3d A(M);
  return A;
}

Eigen::Affine3d Baxter_fwd_solver::fwd_kin_flange_wrt_torso_solve(const Vectorq7x1& q_vec)
{
  Eigen::Matrix4d M;
  M = fwd_kin_solve_(q_vec);
  M = A_torso_to_rarm_mount_ * M;
  Eigen::Affine3d A(M);
  return A;
}

Eigen::Matrix4d Baxter_fwd_solver::get_wrist_frame()
{
  return A_mat_products_[5];
}

Eigen::Matrix4d Baxter_fwd_solver::get_shoulder_frame()
{
  return A_mat_products_[0];
}

Eigen::Matrix4d Baxter_fwd_solver::get_elbow_frame()
{
  return A_mat_products_[3];
}

Eigen::Matrix4d Baxter_fwd_solver::get_flange_frame()
{
  return A_mat_products_[6];
}

Eigen::Matrix4d Baxter_fwd_solver::get_shoulder_frame_approx()
{
  return A_mat_products_approx_[0];
}

Eigen::Matrix4d Baxter_fwd_solver::get_elbow_frame_approx()
{
  return A_mat_products_approx_[3];
}

Eigen::Matrix4d Baxter_fwd_solver::get_wrist_frame_approx()
{
  Eigen::Matrix4d A_wrist;
  A_wrist = A_mat_products_approx_[5];

  return A_wrist;
}

Eigen::Matrix4d Baxter_fwd_solver::get_flange_frame_approx()
{
  return A_mat_products_approx_[6];
}

Eigen::Vector3d Baxter_fwd_solver::get_wrist_coords_wrt_frame1(const Vectorq7x1& q_vec)
{
  Eigen::Matrix4d A_shoulder_to_wrist;
  fwd_kin_solve_(q_vec);
  A_shoulder_to_wrist = A_mats_[1] * A_mats_[2] * A_mats_[3] * A_mats_[4];
  Eigen::Vector3d w_wrt_1 = A_shoulder_to_wrist.block<3, 1>(0, 3);
  return w_wrt_1;
}
/* Wrist Jacobian:  somewhat odd; restricted to q_s1, q_humerus and q_elbow
 * return a 3x3 Jacobian relating dq to delta wrist point coords, w/rt q_s1, q_humerus and q_elbow*/

Eigen::Matrix3d Baxter_fwd_solver::get_wrist_Jacobian_3x3(double q_s1, double q_humerus, double q_elbow,
                                                          double q_forearm)
{
  Vectorq7x1 q_vec;
  for (int i = 0; i < 7; i++)
    q_vec(i) = 0.0;
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

  Eigen::Vector3d zvec, rvec, wvec, Oi;

  for (int i = 0; i < 5; i++)
  {
    A_mats_3dof[i] = compute_A_of_DH(i + 1, q_vec(i + 1));
  }

  A_mat_products_3dof[0] = A_mats_3dof[0];

  for (int i = 1; i < 5; i++)
  {
    A_mat_products_3dof[i] = A_mat_products_3dof[i - 1] * A_mats_3dof[i];
  }
  wvec = A_mat_products_3dof[4].block<3, 1>(0, 3);

  zvec << 0, 0, 1;
  Jw1_ang.block<3, 1>(0, 0) = zvec;
  Oi << 0, 0, 0;
  Origins.block<3, 1>(0, 0) = Oi;
  for (int i = 1; i < 3; i++)
  {
    zvec = A_mat_products_3dof[i - 1].block<3, 1>(0, 2);
    Jw1_ang.block<3, 1>(0, i) = zvec;
    Oi = A_mat_products_3dof[i - 1].block<3, 1>(0, 3);
    Origins.block<3, 1>(0, i) = Oi;
  }

  for (int i = 0; i < 3; i++)
  {
    zvec = Jw1_ang.block<3, 1>(0, i);
    Oi = Origins.block<3, 1>(0, i);
    rvec = wvec - Oi;

    Jw1_trans.block<3, 1>(0, i) = zvec.cross(rvec);
  }

  return Jw1_trans;
}

Eigen::Affine3d Baxter_fwd_solver::transform_affine_from_torso_frame_to_arm_mount_frame(Eigen::Affine3d pose_wrt_torso)
{
  /*
  Eigen::Affine3d desired_pose_wrt_arm_mount,desired_pose_wrt_arm_mount2;
  Eigen::Matrix3d R_hand_des_wrt_torso = pose_wrt_torso.linear();
  Eigen::Vector3d O_hand_des_wrt_torso = pose_wrt_torso.translation();
  Eigen::Vector3d O_hand_des_wrt_arm_mount;
  Eigen::Vector3d O_arm_mount_wrt_torso = A_torso_to_rarm_mount_.col(3).head(3);
  Eigen::Matrix3d R_arm_mount_wrt_torso = A_torso_to_rarm_mount_.block<3, 3>(0, 0);

  desired_pose_wrt_arm_mount.linear() = R_arm_mount_wrt_torso.transpose()*R_hand_des_wrt_torso;

  O_hand_des_wrt_arm_mount = R_arm_mount_wrt_torso.transpose()* O_hand_des_wrt_torso
          - R_arm_mount_wrt_torso.transpose()* O_arm_mount_wrt_torso;

  desired_pose_wrt_arm_mount.translation() = O_hand_des_wrt_arm_mount;
  std::cout<<"input pose w/rt torso: R"<<endl;
  std::cout<<pose_wrt_torso.linear()<<endl;
  std::cout<<"origin of des frame w/rt torso: "<<pose_wrt_torso.translation().transpose()<<endl;

  std::cout<<"input pose w/rt arm-mount frame: R"<<endl;
  std::cout<<desired_pose_wrt_arm_mount.linear()<<endl;
  std::cout<<"origin of des frame w/rt arm-mount frame: "<<desired_pose_wrt_arm_mount.translation().transpose()<<endl;


  desired_pose_wrt_arm_mount2 = Affine_torso_to_rarm_mount_.inverse()*pose_wrt_torso;
   std::cout<<"input pose w/rt arm-mount frame, method 2: R"<<endl;
  std::cout<<desired_pose_wrt_arm_mount2.linear()<<endl;
  std::cout<<"origin of des frame w/rt arm-mount frame, method 2:
  "<<desired_pose_wrt_arm_mount2.translation().transpose()<<endl;


  return desired_pose_wrt_arm_mount;
   * */
  return Affine_torso_to_rarm_mount_.inverse() * pose_wrt_torso;
}

Eigen::Matrix4d Baxter_fwd_solver::fwd_kin_solve_(const Vectorq7x1& q_vec)
{
  Eigen::Matrix4d A = Eigen::Matrix4d::Identity();

  Eigen::Matrix4d A_i_iminusi;
  Eigen::Matrix3d R;
  Eigen::Vector3d p;
  for (int i = 0; i < 7; i++)
  {
    A_i_iminusi = compute_A_of_DH(i, q_vec[i]);
    A_mats_[i] = A_i_iminusi;
  }

  A_mat_products_[0] = A_mats_[0];

  A_mat_products_[0] = A_rarm_mount_to_r_lower_forearm_ * A_mat_products_[0];
  for (int i = 1; i < 7; i++)
  {
    A_mat_products_[i] = A_mat_products_[i - 1] * A_mats_[i];
  }

  return A_mat_products_[6];
}

Eigen::Matrix4d Baxter_fwd_solver::fwd_kin_solve_approx_(const Vectorq7x1& q_vec)
{
  Eigen::Matrix4d A = Eigen::Matrix4d::Identity();

  Eigen::Matrix4d A_i_iminusi;
  Eigen::Matrix3d R;
  Eigen::Vector3d p;
  for (int i = 0; i < 7; i++)
  {
    A_i_iminusi = compute_A_of_DH_approx(i, q_vec[i]);
    A_mats_approx_[i] = A_i_iminusi;
  }

  A_mat_products_approx_[0] = A_mats_approx_[0];

  A_mat_products_approx_[0] = A_rarm_mount_to_r_lower_forearm_ * A_mat_products_approx_[0];

  for (int i = 1; i < 7; i++)
  {
    A_mat_products_approx_[i] = A_mat_products_approx_[i - 1] * A_mats_approx_[i];
  }

  return A_mat_products_approx_[6];
}

Baxter_IK_solver::Baxter_IK_solver()
{
  L_humerus_ = 0.37082;
  double L3 = DH_d3;
  double A3 = DH_a3;

  L_forearm_ = DH_d5;

  phi_shoulder_ = acos((-A3 * A3 + L_humerus_ * L_humerus_ + L3 * L3) / (2.0 * L3 * L_humerus_));
}

void Baxter_IK_solver::get_solns(std::vector<Vectorq7x1>& q_solns)
{
  q_solns = q_solns_fit;
}

Eigen::Vector3d Baxter_IK_solver::wrist_frame0_from_flange_wrt_rarm_mount(Eigen::Affine3d affine_flange_frame)
{
  Eigen::Vector3d flange_z_axis_wrt_arm_mount;
  Eigen::Vector3d flange_origin_wrt_arm_mount;
  Eigen::Vector3d wrist_pt_vec_wrt_arm_mount;
  Eigen::Matrix3d R_flange_wrt_arm_mount;

  R_flange_wrt_arm_mount = affine_flange_frame.linear();
  flange_origin_wrt_arm_mount = affine_flange_frame.translation();
  flange_z_axis_wrt_arm_mount = R_flange_wrt_arm_mount.col(2);
  wrist_pt_vec_wrt_arm_mount = flange_origin_wrt_arm_mount - flange_z_axis_wrt_arm_mount * DH_d7;

  return wrist_pt_vec_wrt_arm_mount;
}

Eigen::Vector3d Baxter_IK_solver::wrist_frame1_from_flange_wrt_rarm_mount(Eigen::Affine3d affine_flange_frame,
                                                                          Vectorq7x1 q_vec)
{
  return wrist_pt_wrt_frame1_of_flange_des_and_qs0(affine_flange_frame, q_vec);
}

Eigen::Vector3d Baxter_IK_solver::wrist_pt_wrt_frame1_of_flange_des_and_qs0(Eigen::Affine3d affine_flange_frame,
                                                                            Vectorq7x1 q_vec)
{
  Eigen::Vector3d flange_z_axis_wrt_arm_mount;
  Eigen::Vector3d flange_origin_wrt_arm_mount;
  Eigen::Vector3d wrist_pt_wrt_arm_frame1;
  Eigen::Vector3d wrist_pt_vec_wrt_arm_mount;
  Eigen::Matrix3d R_flange_wrt_arm_mount;

  R_flange_wrt_arm_mount = affine_flange_frame.linear();
  flange_origin_wrt_arm_mount = affine_flange_frame.translation();
  flange_z_axis_wrt_arm_mount = R_flange_wrt_arm_mount.col(2);
  wrist_pt_vec_wrt_arm_mount = flange_origin_wrt_arm_mount - flange_z_axis_wrt_arm_mount * DH_d7;

  Eigen::Affine3d A_fwd_DH_approx = fwd_kin_flange_wrt_r_arm_mount_solve_approx(q_vec);

  Eigen::Matrix4d A_shoulder_wrt_arm_mount = get_shoulder_frame_approx();

  Eigen::Matrix3d R_shoulder_wrt_arm_mount = A_shoulder_wrt_arm_mount.block<3, 3>(0, 0);

  Eigen::Vector3d p1_wrt_0 = A_shoulder_wrt_arm_mount.col(3).head(3);

  /*
      R10 = A10.block<3, 3>(0, 0);
  p1_wrt_0 = A10.col(3).head(3);

  w_wrt_1b = R10.transpose() * w_des - R10.transpose() * p1_wrt_0;
  */

  wrist_pt_wrt_arm_frame1 = R_shoulder_wrt_arm_mount.transpose() * wrist_pt_vec_wrt_arm_mount -
                            R_shoulder_wrt_arm_mount.transpose() * p1_wrt_0;

  return wrist_pt_wrt_arm_frame1;
}

Eigen::Vector3d Baxter_IK_solver::wrist_pt_from_flange_frame(Eigen::Affine3d affine_flange_frame)
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

bool Baxter_IK_solver::fit_q_to_range(double q_min, double q_max, double& q)
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

bool Baxter_IK_solver::fit_joints_to_range(Vectorq7x1& qvec)
{
  bool fits = true;
  bool does_fit;
  double q;
  for (int i = 0; i < 7; i++)
  {
    q = qvec[i];
    does_fit = fit_q_to_range(q_lower_limits[i], q_upper_limits[i], q);
    qvec[i] = q;
    fits = fits && does_fit;
  }
  if (fits)
    return true;
  else
    return false;
}

int Baxter_IK_solver::ik_solve(Eigen::Affine3d const& desired_hand_pose)
{
  return 0;
}

int Baxter_IK_solver::ik_solve_approx_wrt_torso(Eigen::Affine3d const& desired_flange_pose,
                                                std::vector<Vectorq7x1>& q_solns)
{
  Eigen::Affine3d desired_hand_pose_wrt_arm_mount = Affine_torso_to_rarm_mount_.inverse() * desired_flange_pose;
  int nsolns = ik_solve_approx(desired_hand_pose_wrt_arm_mount, q_solns);
  return nsolns;
}

int Baxter_IK_solver::ik_solve_approx_wrt_torso(Eigen::Affine3d const desired_tool_pose_wrt_torso,
                                                Eigen::Affine3d A_tool_wrt_flange, std::vector<Vectorq7x1>& q_solns)
{
  A_tool_wrt_flange_ = A_tool_wrt_flange;

  Eigen::Affine3d desired_flange_pose_wrt_torso = desired_tool_pose_wrt_torso * A_tool_wrt_flange_.inverse();

  int nsolns = ik_solve_approx(desired_flange_pose_wrt_torso, q_solns);
  return nsolns;
}

int Baxter_IK_solver::ik_wristpt_solve_approx_wrt_torso(Eigen::Affine3d const& desired_flange_pose_wrt_torso,
                                                        std::vector<Vectorq7x1>& q_solns)
{
  Eigen::Affine3d desired_flange_pose_wrt_arm_mount =
      Affine_torso_to_rarm_mount_.inverse() * desired_flange_pose_wrt_torso;
  return ik_wrist_solve_approx(desired_flange_pose_wrt_arm_mount, q_solns);
}

int Baxter_IK_solver::ik_solve_approx_wrt_torso_given_qs0(Eigen::Affine3d const& desired_hand_pose_wrt_torso,
                                                          double q_s0, std::vector<Vectorq7x1>& q_solns)
{
  Eigen::Affine3d desired_hand_pose_wrt_arm_mount = Affine_torso_to_rarm_mount_.inverse() * desired_hand_pose_wrt_torso;
  Eigen::Matrix3d Rdes = desired_hand_pose_wrt_arm_mount.linear();
  q_solns.clear();

  std::vector<Vectorq7x1> q_solns_of_qs0, q_solns_w_wrist;

  Vectorq7x1 q_soln;

  int nsolns = 0;
  bool reachable;
  reachable = compute_q123_solns(desired_hand_pose_wrt_arm_mount, q_s0, q_solns_of_qs0);

  for (int i = 0; i < q_solns_of_qs0.size(); i++)
  {
    q_soln = q_solns_of_qs0[i];

    solve_spherical_wrist(q_soln, Rdes, q_solns_w_wrist);
    for (int j = 0; j < q_solns_w_wrist.size(); j++)
    {
      q_solns.push_back(q_solns_w_wrist[j]);
    }
  }

  return q_solns.size();
}

int Baxter_IK_solver::ik_wrist_solve_approx(Eigen::Affine3d const& desired_flange_pose,
                                            std::vector<Vectorq7x1>& q_solns_123)
{
  double q_s0_ctr = compute_qs0_ctr(desired_flange_pose);

  double dqs0 = DQS0;
  double q_s0 = q_s0_ctr;
  std::vector<Vectorq7x1> q_solns_of_qs0, q_solns;
  Vectorq7x1 q_soln;
  int nsolns = 0;
  q_solns.clear();
  q_solns_123.clear();
  Eigen::Vector3d w_des_wrt_0 = wrist_frame0_from_flange_wrt_rarm_mount(desired_flange_pose);
  Eigen::Affine3d A_fwd_DH;
  Eigen::Matrix4d A_wrist;
  Eigen::Matrix3d Rdes = desired_flange_pose.linear();
  bool reachable = true;
  while (reachable)
  {
    reachable = compute_q123_solns(desired_flange_pose, q_s0, q_solns_of_qs0);

    if (reachable)
    {
      for (int i = 0; i < q_solns_of_qs0.size(); i++)
      {
        /*
        A_fwd_DH = fwd_kin_solve(q_solns_of_qs0[i]);
        A_wrist = get_wrist_frame();
        std::cout << "sln"<<i<<":  w_wrt_0 " << A_wrist(0, 3) << ", " << A_wrist(1, 3) << ", " << A_wrist(2, 3) <<
        std::endl;
          */
        q_solns.push_back(q_solns_of_qs0[i]);
      }
      q_s0 += dqs0;
    }
  }

  reachable = true;

  for (int i = q_solns_123.size() - 1; i >= 0; i--)
  {
    q_solns_123.push_back(q_solns[i]);
  }

  q_s0 = q_s0_ctr - dqs0;

  while (reachable)
  {
    reachable = compute_q123_solns(desired_flange_pose, q_s0, q_solns_of_qs0);
    if (reachable)
    {
      for (int i = 0; i < q_solns_of_qs0.size(); i++)
      {
        /*
        A_fwd_DH = fwd_kin_solve(q_solns_of_qs0[i]);
        A_wrist = get_wrist_frame();
        std::cout << "sln"<<i<<":  w_wrt_0 " << A_wrist(0, 3) << ", " << A_wrist(1, 3) << ", " << A_wrist(2, 3) <<
        std::endl;
         * */
        q_solns_123.push_back(q_solns_of_qs0[i]);
      }
      q_s0 -= dqs0;
    }
  }

  return q_solns_123.size();
}

int Baxter_IK_solver::ik_solve_approx(Eigen::Affine3d const& desired_flange_pose, std::vector<Vectorq7x1>& q_solns)
{
  double q_s0_ctr = compute_qs0_ctr(desired_flange_pose);

  double dqs0 = DQS0;
  double q_s0 = q_s0_ctr;
  std::vector<Vectorq7x1> q_solns_123, q_solns_of_qs0, q_solns_w_wrist;
  Vectorq7x1 q_soln;
  int nsolns = 0;
  q_solns.clear();
  q_solns_123.clear();

  Eigen::Vector3d w_des_wrt_0 = wrist_frame0_from_flange_wrt_rarm_mount(desired_flange_pose);

  Eigen::Affine3d A_fwd_DH;
  Eigen::Matrix4d A_wrist;
  Eigen::Matrix3d Rdes = desired_flange_pose.linear();
  bool reachable = true;
  while (reachable)
  {
    reachable = compute_q123_solns(desired_flange_pose, q_s0, q_solns_of_qs0);

    if (reachable)
    {
      for (int i = 0; i < q_solns_of_qs0.size(); i++)
      {
        /*
        A_fwd_DH = fwd_kin_solve(q_solns_of_qs0[i]);
        A_wrist = get_wrist_frame();
        std::cout << "sln"<<i<<":  w_wrt_0 " << A_wrist(0, 3) << ", " << A_wrist(1, 3) << ", " << A_wrist(2, 3) <<
        std::endl;
          */
        q_solns_123.push_back(q_solns_of_qs0[i]);
      }
      q_s0 += dqs0;
    }
  }

  reachable = true;

  for (int i = q_solns_123.size() - 1; i >= 0; i--)
  {
    q_soln = q_solns_123[i];
    solve_spherical_wrist(q_soln, Rdes, q_solns_w_wrist);
    for (int j = 0; j < q_solns_w_wrist.size(); j++)
    {
      q_solns.push_back(q_solns_w_wrist[j]);
    }
  }

  q_s0 = q_s0_ctr - dqs0;
  q_solns_123.clear();
  while (reachable)
  {
    reachable = compute_q123_solns(desired_flange_pose, q_s0, q_solns_of_qs0);
    if (reachable)
    {
      for (int i = 0; i < q_solns_of_qs0.size(); i++)
      {
        /*
        A_fwd_DH = fwd_kin_solve(q_solns_of_qs0[i]);
        A_wrist = get_wrist_frame();
        std::cout << "sln"<<i<<":  w_wrt_0 " << A_wrist(0, 3) << ", " << A_wrist(1, 3) << ", " << A_wrist(2, 3) <<
        std::endl;
         * */
        q_solns_123.push_back(q_solns_of_qs0[i]);
      }
      q_s0 -= dqs0;
    }
  }

  reachable = true;

  for (int i = 0; i < q_solns_123.size(); i++)
  {
    q_soln = q_solns_123[i];
    solve_spherical_wrist(q_soln, Rdes, q_solns_w_wrist);
    for (int j = 0; j < q_solns_w_wrist.size(); j++)
    {
      q_solns.push_back(q_solns_w_wrist[j]);
    }
  }

  return q_solns.size();
}

int Baxter_IK_solver::ik_solve_approx_elbow_orbit_from_flange_pose_wrt_torso(
    Eigen::Affine3d const& desired_flange_pose_wrt_torso, std::vector<std::vector<Eigen::VectorXd> >& path_options)
{
  Eigen::Affine3d desired_flange_pose_wrt_rarm_mount =
      Affine_torso_to_rarm_mount_.inverse() * desired_flange_pose_wrt_torso;
  Eigen::Matrix3d Rdes = desired_flange_pose_wrt_rarm_mount.linear();
  double q_s0_ctr = compute_qs0_ctr(desired_flange_pose_wrt_rarm_mount);
  double dqs0 = DQS0;
  double q_s0 = q_s0_ctr;

  std::vector<Eigen::VectorXd> single_layer_nodes;
  std::vector<std::vector<Eigen::VectorXd> > path_options_reverse;
  Eigen::VectorXd node;
  int nsolns;
  std::vector<Vectorq7x1> q_solns_of_qs0;
  path_options.clear();

  bool reachable = true;

  while (reachable)
  {
    nsolns = ik_solve_approx_wrt_torso_given_qs0(desired_flange_pose_wrt_torso, q_s0, q_solns_of_qs0);
    if (nsolns == 0)
      reachable = false;
    if (reachable)
    {
      single_layer_nodes.clear();
      for (int i = 0; i < nsolns; i++)
      {
        node = q_solns_of_qs0[i];
        single_layer_nodes.push_back(node);
      }
      path_options_reverse.push_back(single_layer_nodes);
      q_s0 += dqs0;
    }
  }

  for (int i = path_options_reverse.size() - 1; i >= 0; i--)
  {
    path_options.push_back(path_options_reverse[i]);
  }

  reachable = true;

  q_s0 = q_s0_ctr - dqs0;
  while (reachable)
  {
    nsolns = ik_solve_approx_wrt_torso_given_qs0(desired_flange_pose_wrt_torso, q_s0, q_solns_of_qs0);
    if (nsolns == 0)
      reachable = false;
    if (reachable)
    {
      single_layer_nodes.clear();
      for (int i = 0; i < nsolns; i++)
      {
        node = q_solns_of_qs0[i];
        single_layer_nodes.push_back(node);
      }
      path_options.push_back(single_layer_nodes);
      q_s0 -= dqs0;
    }
  }

  return path_options.size();
}

int Baxter_IK_solver::ik_solve_approx_elbow_orbit_plus_qdot_s0_from_flange_pose_wrt_torso(
    Vectorq7x1 q_start, std::vector<std::vector<Eigen::VectorXd> >& path_options)
{
  Eigen::Affine3d desired_flange_pose_wrt_torso = fwd_kin_flange_wrt_torso_solve(q_start);

  Eigen::Affine3d desired_flange_pose_wrt_rarm_mount =
      Affine_torso_to_rarm_mount_.inverse() * desired_flange_pose_wrt_torso;
  Eigen::Matrix3d Rdes = desired_flange_pose_wrt_rarm_mount.linear();

  double dqs0 = DQS0;
  double q_s0 = q_start[0];

  std::vector<Eigen::VectorXd> single_layer_nodes;
  Eigen::VectorXd node;
  int nsolns;
  std::vector<Vectorq7x1> q_solns_of_qs0;

  path_options.clear();
  node = q_start;
  single_layer_nodes.clear();
  single_layer_nodes.push_back(node);
  path_options.clear();
  path_options.push_back(single_layer_nodes);
  bool reachable = true;

  while (reachable)
  {
    q_s0 += dqs0;

    nsolns = ik_solve_approx_wrt_torso_given_qs0(desired_flange_pose_wrt_torso, q_s0, q_solns_of_qs0);
    if (nsolns == 0)
      reachable = false;
    if (reachable)
    {
      single_layer_nodes.clear();
      for (int i = 0; i < nsolns; i++)
      {
        node = q_solns_of_qs0[i];
        single_layer_nodes.push_back(node);
      }
      path_options.push_back(single_layer_nodes);
    }
  }

  return path_options.size();
}

double Baxter_IK_solver::precise_soln_q123(Eigen::Affine3d const& desired_flange_pose, Vectorq7x1 q123_approx,
                                           Vectorq7x1& q123_precise)
{
  double q_s0 = q123_approx[0];

  Eigen::Vector3d w_approx, w_err, dq123, wrist_pt_wrt_right_arm_frame1;
  Eigen::Matrix3d Jw3x3, Jw3x3_inv;

  wrist_pt_wrt_right_arm_frame1 = wrist_pt_wrt_frame1_of_flange_des_and_qs0(desired_flange_pose, q123_approx);
  Jw3x3 = get_wrist_Jacobian_3x3(q123_approx[1], q123_approx[2], q123_approx[3], q123_approx[4]);

  Jw3x3_inv = Jw3x3.inverse();

  q123_precise = q123_approx;
  int jiter = 0;
  double w_err_norm = 1.0;
  while ((jiter < MAX_JINV_ITERS) && (w_err_norm > W_ERR_TOL))
  {
    w_approx = get_wrist_coords_wrt_frame1(q123_precise);
    w_err = wrist_pt_wrt_right_arm_frame1 - w_approx;
    w_err_norm = w_err.norm();

    dq123 = Jw3x3_inv * w_err;
    if (dq123.norm() > DQ_ITER_MAX)
    {
      dq123 /= DQ_ITER_MAX;
    }

    for (int i = 1; i < 4; i++)
    {
      q123_precise[i] += dq123[i - 1];
    }
    jiter++;
  }
  return w_err_norm;
}

double Baxter_IK_solver::compute_qs0_ctr(Eigen::Affine3d const& desired_flange_pose)
{
  bool reachable = false;
  Eigen::Vector3d w_wrt_1;
  Vectorq7x1 soln1_vec;
  soln1_vec(0) = 0.0;
  w_wrt_1 = wrist_frame1_from_flange_wrt_rarm_mount(desired_flange_pose, soln1_vec);

  double r_perp = sqrt(w_wrt_1(1) * w_wrt_1(1) + w_wrt_1(0) * w_wrt_1(0));
  double q_s0_ctr = atan2(w_wrt_1(2), r_perp);

  soln1_vec(0) = q_s0_ctr;

  w_wrt_1 = wrist_frame1_from_flange_wrt_rarm_mount(desired_flange_pose, soln1_vec);

  return q_s0_ctr;
}

bool Baxter_IK_solver::compute_q123_solns(Eigen::Affine3d const& desired_flange_pose, double q_s0,
                                          std::vector<Vectorq7x1>& q_solns)
{
  q_solns.clear();
  Vectorq7x1 soln1_vec;
  Vectorq7x1 soln2_vec;

  for (int i = 0; i < 7; i++)
  {
    soln1_vec[i] = 0.0;
    soln2_vec[i] = 0.0;
  }
  soln1_vec[0] = q_s0;
  soln2_vec[0] = q_s0;
  double q_elbow;
  double q_humerus[2];
  double q_s1[2];
  double q_s1_temp;
  bool does_fit;
  bool reachable = false;
  bool at_least_one_valid_soln = false;
  Eigen::Vector3d wrist_pt_wrt_right_arm_frame1;
  /* constrain q_s0 limits */
  if (q_s0 > q_upper_limits[0])
  {
    return false;
  }
  if (q_s0 < q_lower_limits[0])
  {
    return false;
  }

  wrist_pt_wrt_right_arm_frame1 = wrist_frame1_from_flange_wrt_rarm_mount(desired_flange_pose, soln1_vec);

  reachable = solve_for_elbow_ang(wrist_pt_wrt_right_arm_frame1, q_elbow);
  if (!reachable)
  {
    return false;
  }

  does_fit = fit_q_to_range(q_lower_limits[3], q_upper_limits[3], q_elbow);
  if (!does_fit)
  {
    return false;
  }

  soln1_vec[3] = q_elbow;
  soln2_vec[3] = q_elbow;

  reachable = solve_for_humerus_ang(wrist_pt_wrt_right_arm_frame1, q_elbow, q_humerus);
  if (!reachable)
  {
    return false;
  }
  soln1_vec[2] = q_humerus[0];
  soln2_vec[2] = q_humerus[1];

  does_fit = fit_q_to_range(q_lower_limits[2], q_upper_limits[2], q_humerus[0]);
  if (does_fit)
  {
    reachable = solve_for_s1_ang(wrist_pt_wrt_right_arm_frame1, q_elbow, q_humerus[0], q_s1_temp);

    if (reachable)
    {
      if (fit_q_to_range(q_lower_limits[1], q_upper_limits[1], q_s1_temp))
      {
        soln1_vec[1] = q_s1_temp;
        q_solns.push_back(soln1_vec);
        at_least_one_valid_soln = true;
      }

      else
      {
      }
    }
  }
  else
  {
  }

  does_fit = fit_q_to_range(q_lower_limits[2], q_upper_limits[2], q_humerus[1]);
  if (does_fit)
  {
    reachable = solve_for_s1_ang(wrist_pt_wrt_right_arm_frame1, q_elbow, q_humerus[1], q_s1_temp);

    if (reachable)
    {
      if (fit_q_to_range(q_lower_limits[1], q_upper_limits[1], q_s1_temp))
      {
        soln2_vec[1] = q_s1_temp;
        q_solns.push_back(soln2_vec);
        at_least_one_valid_soln = true;
      }
      else
      {
      }
    }
  }
  else
  {
  }

  return at_least_one_valid_soln;
}

bool Baxter_IK_solver::solve_for_elbow_ang(Eigen::Vector3d w_wrt_1, double& q_elbow)
{
  double r_goal = w_wrt_1.norm();

  double acos_arg =
      (L_humerus_ * L_humerus_ + L_forearm_ * L_forearm_ - r_goal * r_goal) / (2.0 * L_humerus_ * L_forearm_);
  if ((fabs(acos_arg) > 1.0) || (r_goal > r_goal_max))
  {
    return false;
  }

  double eta = acos(acos_arg);

  double q_elbow_nom = M_PI - eta;

  q_elbow = q_elbow_nom + phi_shoulder_;

  /*
  ROS_INFO("test:");
  Eigen::Matrix4d    A_12 = compute_A_of_DH_approx(1, 0);
  Eigen::Matrix4d    A_23 = compute_A_of_DH_approx(2, 0);
  Eigen::Matrix4d    A_34 = compute_A_of_DH_approx(3, q_elbow);
  Eigen::Matrix4d    A_45 = compute_A_of_DH_approx(4, 0);
  Eigen::Matrix4d    A_15 = A_12*A_23*A_34*A_45;
  std::cout<<"A_15(q_elbow) = "<<endl;
  std::cout<<A_15<<endl;
  Eigen::Vector3d p_fwd_wrt_1 = A_15.col(3).head(3);
  double r_soln = p_fwd_wrt_1.norm();
  std::cout<<"r_soln = "<<r_soln<<endl;
  */
  return true;
}

bool Baxter_IK_solver::solve_for_humerus_ang(Eigen::Vector3d w_wrt_1, double q_elbow, double q_humerus[2])
{
  double r_arm = DH_a3 + sin(q_elbow) * L_forearm_;
  double z_offset = w_wrt_1(2);

  if (fabs(z_offset) > r_arm)
  {
    return false;
  }
  q_humerus[0] = asin(z_offset / r_arm);
  q_humerus[1] = M_PI - q_humerus[0];

  /*
  ROS_INFO("test:");
  Eigen::Matrix4d    A_12 = compute_A_of_DH_approx(1, 0);
  Eigen::Matrix4d    A_23 = compute_A_of_DH_approx(2, q_humerus[0]);
  Eigen::Matrix4d    A_34 = compute_A_of_DH_approx(3, q_elbow);
  Eigen::Matrix4d    A_45 = compute_A_of_DH_approx(4, 0);
  Eigen::Matrix4d    A_15 = A_12*A_23*A_34*A_45;
  std::cout<<"A_15(q_humerus[0],q_elbow) = "<<endl;
  std::cout<<A_15<<endl;
  A_23 = compute_A_of_DH_approx(2, q_humerus[1]);
  A_15 = A_12*A_23*A_34*A_45;
  std::cout<<"A_15(q_humerus[1],q_elbow) = "<<endl;
  std::cout<<A_15<<endl;
   * */
  return true;
}

bool Baxter_IK_solver::solve_for_s1_ang(Eigen::Vector3d w_wrt_1, double q_elbow, double q_humerus, double& q_s1)
{
  double r_reach = w_wrt_1.norm();

  double r_perp = DH_a3 + sin(q_elbow) * L_forearm_;
  double Lw_sqd = r_reach * r_reach - r_perp * r_perp;

  if (Lw_sqd < 0)
  {
    return false;
  }
  double Lw = sqrt(Lw_sqd);

  double b = Lw;
  double c = w_wrt_1(0);

  double a = r_perp * cos(q_humerus);
  double alpha = atan2(b, a);
  double rtemp = sqrt(a * a + b * b);

  double cos_of_arg = c / rtemp;
  if (fabs(cos_of_arg) > 1.0)
  {
    ROS_WARN("solve_for_s1_ang: logic problem w/ alpha");
    return false;
  }
  double ang_arg = acos(cos_of_arg);
  double q_s1a = alpha - ang_arg;

  double q_s1b = alpha + ang_arg;

  double LHSax = r_perp * cos(q_s1a) * cos(q_humerus) + Lw * sin(q_s1a);
  double LHSbx = r_perp * cos(q_s1b) * cos(q_humerus) + Lw * sin(q_s1b);
  double RHSx = w_wrt_1(0);

  double LHSay = r_perp * sin(q_s1a) * cos(q_humerus) - Lw * cos(q_s1a);
  double LHSby = r_perp * sin(q_s1b) * cos(q_humerus) - Lw * cos(q_s1b);
  double RHSy = w_wrt_1(1);

  double erra = fabs(LHSax - RHSx) + fabs(LHSay - RHSy);
  double errb = fabs(LHSbx - RHSx) + fabs(LHSby - RHSy);

  double err_qs = erra;
  q_s1 = q_s1a;
  if (errb < erra)
  {
    q_s1 = q_s1b;
    err_qs = errb;
  }

  q_s1 -= DH_q_offsets[1];

  /*
  ROS_INFO("test fwd kin:");
  Eigen::Matrix4d    A_12 = compute_A_of_DH_approx(1, q_s1);
  Eigen::Matrix4d    A_23 = compute_A_of_DH_approx(2, q_humerus);
  Eigen::Matrix4d    A_34 = compute_A_of_DH_approx(3, q_elbow);
  Eigen::Matrix4d    A_45 = compute_A_of_DH_approx(4, 0);
  Eigen::Matrix4d    A_15 = A_12*A_23*A_34*A_45;
  std::cout<<"A_15(q_s1,q_humerus,q_elbow) = "<<endl;
  std::cout<<A_15<<endl;
   * */

  return true;
}

bool Baxter_IK_solver::solve_spherical_wrist(Vectorq7x1 q_in, Eigen::Matrix3d R_des, std::vector<Vectorq7x1>& q_solns)
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

  if (b5.norm() <= 0.000001)
  {
    q4 = 0;
    is_singular = true;
  }
  else
  {
    double cq4 = b5.dot(-t4);
    double sq4 = b5.dot(n4);
    q4 = atan2(sq4, cq4);
  }

  if (q4 > M_PI)
  {
    q4 -= 2 * M_PI;
  }
  if (q4 < 0.0)
  {
    q4 += M_PI;
  }

  A45 = compute_A_of_DH(4, q4);
  A05 = A04 * A45;
  n5 = A05.col(0).head(3);
  t5 = A05.col(1).head(3);
  double cq5 = b_des.dot(t5);
  double sq5 = b_des.dot(-n5);
  q5 = atan2(sq5, cq5) + M_PI;

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

bool Baxter_IK_solver::update_spherical_wrist(Vectorq7x1 q_in, Eigen::Matrix3d R_des, Vectorq7x1& q_precise)
{
  std::vector<Vectorq7x1> q_solns;
  solve_spherical_wrist(q_in, R_des, q_solns);
  int nsolns = q_solns.size();
  ROS_INFO("update_spherical_wrist: num solns for wrist = %d", nsolns);
  if (nsolns == 0)
  {
    q_precise = q_in;
    return false;
  }
  if (nsolns == 2)
  {
    double err1 = (q_in - q_solns[0]).norm();
    double err2 = (q_in - q_solns[1]).norm();
    if (err1 < err2)
    {
      q_precise = q_solns[0];
    }
    else
    {
      q_precise = q_solns[1];
    }
    return true;
  }

  if (nsolns == 1)
  {
    q_precise = q_solns[0];
    double err = (q_in - q_precise).norm();
    if (err > 1.0)
    {
      ROS_WARN("update_spherical_wrist: likely poor fit");
      return false;
    }
    return true;
  }
  ROS_WARN("update_spherical_wrist: unexpected case");
  return false;
}

bool Baxter_IK_solver::improve_7dof_soln(Eigen::Affine3d const& desired_flange_pose_wrt_arm_mount, Vectorq7x1 q_in,
                                         Vectorq7x1& q_7dof_precise)
{
  Eigen::Matrix3d R_flange_wrt_right_arm_mount = desired_flange_pose_wrt_arm_mount.linear();
  Vectorq7x1 q123_precise;

  double w_err_norm = precise_soln_q123(desired_flange_pose_wrt_arm_mount, q_in, q123_precise);

  bool valid = update_spherical_wrist(q123_precise, R_flange_wrt_right_arm_mount, q_7dof_precise);
  return valid;
}

bool Baxter_IK_solver::improve_7dof_soln_wrt_torso(Eigen::Affine3d const& desired_flange_pose_wrt_torso,
                                                   Vectorq7x1 q_in, Vectorq7x1& q_7dof_precise)
{
  Eigen::Affine3d desired_flange_pose_wrt_right_arm_mount =
      Affine_torso_to_rarm_mount_.inverse() * desired_flange_pose_wrt_torso;

  Eigen::Vector3d des_origin, origin_from_q_approx, origin_refined;
  des_origin = desired_flange_pose_wrt_torso.translation();

  bool valid = improve_7dof_soln(desired_flange_pose_wrt_right_arm_mount, q_in, q_7dof_precise);
  if (!valid)
  {
    ROS_WARN("improved_7dof_soln returned not-valid!");
  }

  Eigen::Affine3d orig_affine = fwd_kin_flange_wrt_torso_solve(q_in);
  origin_from_q_approx = orig_affine.translation();
  double err1 = (origin_from_q_approx - des_origin).norm();

  Eigen::Affine3d refined_affine = fwd_kin_flange_wrt_torso_solve(q_7dof_precise);
  origin_refined = refined_affine.translation();

  double err2 = (origin_refined - des_origin).norm();
  if (err2 > err1)
  {
    ROS_WARN("IK improvement failed; retaining q_approx");
    q_7dof_precise = q_in;
  }

  return valid;
}