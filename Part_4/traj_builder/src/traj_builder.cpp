#include "traj_builder/traj_builder.hpp"

TrajBuilder::TrajBuilder()
{
  dt_ = default_dt;

  accel_max_ = default_accel_max;
  alpha_max_ = default_alpha_max;
  speed_max_ = default_speed_max;
  omega_max_ = default_omega_max;
  path_move_tol_ = default_path_move_tol;

  halt_twist_.linear.x = 0.0;
  halt_twist_.linear.y = 0.0;
  halt_twist_.linear.z = 0.0;
  halt_twist_.angular.x = 0.0;
  halt_twist_.angular.y = 0.0;
  halt_twist_.angular.z = 0.0;
}

double TrajBuilder::min_dang(double dang)
{
  while (dang > M_PI)
    dang -= 2.0 * M_PI;
  while (dang < -M_PI)
    dang += 2.0 * M_PI;
  return dang;
}

double TrajBuilder::sat(double x)
{
  if (x > 1.0)
  {
    return 1.0;
  }
  if (x < -1.0)
  {
    return -1.0;
  }
  return x;
}

double TrajBuilder::sgn(double x)
{
  if (x > 0.0)
  {
    return 1.0;
  }
  if (x < 0.0)
  {
    return -1.0;
  }
  return 0.0;
}

double TrajBuilder::convertPlanarQuat2Psi(geometry_msgs::msg::Quaternion quaternion)
{
  double quat_z = quaternion.z;
  double quat_w = quaternion.w;
  double psi = 2.0 * atan2(quat_z, quat_w);
  return psi;
}

geometry_msgs::msg::Quaternion TrajBuilder::convertPlanarPsi2Quaternion(double psi)
{
  geometry_msgs::msg::Quaternion quaternion;
  quaternion.x = 0.0;
  quaternion.y = 0.0;
  quaternion.z = sin(psi / 2.0);
  quaternion.w = cos(psi / 2.0);
  return quaternion;
}

geometry_msgs::msg::PoseStamped TrajBuilder::xyPsi2PoseStamped(double x, double y, double psi)
{
  geometry_msgs::msg::PoseStamped poseStamped;
  poseStamped.pose.orientation = convertPlanarPsi2Quaternion(psi);
  poseStamped.pose.position.x = x;
  poseStamped.pose.position.y = y;
  poseStamped.pose.position.z = 0.0;
  return poseStamped;
}

void TrajBuilder::build_trapezoidal_spin_traj(geometry_msgs::msg::PoseStamped start_pose,
                                              geometry_msgs::msg::PoseStamped end_pose,
                                              std::vector<nav_msgs::msg::Odometry> &vec_of_states)
{
  // double x_start = start_pose.pose.position.x;
  // double y_start = start_pose.pose.position.y;
  // double x_end = end_pose.pose.position.x;
  // double y_end = end_pose.pose.position.y;
  // double dx = x_end - x_start;
  // double dy = y_end - y_start;
  double psi_start = convertPlanarQuat2Psi(start_pose.pose.orientation);
  double psi_end = convertPlanarQuat2Psi(end_pose.pose.orientation);
  double dpsi = min_dang(psi_end - psi_start);
  double t_ramp = omega_max_ / alpha_max_;
  double ramp_up_dist = 0.5 * alpha_max_ * t_ramp * t_ramp;
  double cruise_distance = fabs(dpsi) - 2.0 * ramp_up_dist;
  int npts_ramp = round(t_ramp / dt_);
  nav_msgs::msg::Odometry des_state;
  des_state.header = start_pose.header;
  des_state.pose.pose = start_pose.pose;
  des_state.twist.twist = halt_twist_;

  double t = 0.0;
  double accel = sgn(dpsi) * alpha_max_;
  double omega_des = 0.0;
  double psi_des = psi_start;
  for (int i = 0; i < npts_ramp; i++)
  {
    t += dt_;
    omega_des = accel * t;
    des_state.twist.twist.angular.z = omega_des;

    psi_des = psi_start + 0.5 * accel * t * t;
    des_state.pose.pose.orientation = convertPlanarPsi2Quaternion(psi_des);
    vec_of_states.push_back(des_state);
  }

  omega_des = sgn(dpsi) * omega_max_;
  des_state.twist.twist.angular.z = sgn(dpsi) * omega_max_;
  double t_cruise = cruise_distance / omega_max_;
  int npts_cruise = round(t_cruise / dt_);
  for (int i = 0; i < npts_cruise; i++)
  {
    psi_des += omega_des * dt_;
    des_state.pose.pose.orientation = convertPlanarPsi2Quaternion(psi_des);
    vec_of_states.push_back(des_state);
  }

  for (int i = 0; i < npts_ramp; i++)
  {
    omega_des -= accel * dt_;
    des_state.twist.twist.angular.z = omega_des;
    psi_des += omega_des * dt_;
    des_state.pose.pose.orientation = convertPlanarPsi2Quaternion(psi_des);
    vec_of_states.push_back(des_state);
  }

  des_state.pose.pose = end_pose.pose;
  des_state.twist.twist = halt_twist_;
  vec_of_states.push_back(des_state);
}

void TrajBuilder::build_spin_traj(geometry_msgs::msg::PoseStamped start_pose, geometry_msgs::msg::PoseStamped end_pose,
                                  std::vector<nav_msgs::msg::Odometry> &vec_of_states)
{
  // double x_start = start_pose.pose.position.x;
  // double y_start = start_pose.pose.position.y;
  // double x_end = end_pose.pose.position.x;
  // double y_end = end_pose.pose.position.y;
  // double dx = x_end - x_start;
  // double dy = y_end - y_start;
  double psi_start = convertPlanarQuat2Psi(start_pose.pose.orientation);
  double psi_end = convertPlanarQuat2Psi(end_pose.pose.orientation);
  double dpsi = min_dang(psi_end - psi_start);
  ROS_INFO("rotational spin distance = %f\n", dpsi);
  double ramp_up_time = omega_max_ / alpha_max_;
  double ramp_up_dist = 0.5 * alpha_max_ * ramp_up_time * ramp_up_time;

  if (fabs(dpsi) < 2.0 * ramp_up_dist)
  {
    build_triangular_spin_traj(start_pose, end_pose, vec_of_states);
  }
  else
  {
    build_trapezoidal_spin_traj(start_pose, end_pose, vec_of_states);
  }
}

void TrajBuilder::build_travel_traj(geometry_msgs::msg::PoseStamped start_pose,
                                    geometry_msgs::msg::PoseStamped end_pose,
                                    std::vector<nav_msgs::msg::Odometry> &vec_of_states)
{
  double x_start = start_pose.pose.position.x;
  double y_start = start_pose.pose.position.y;
  double x_end = end_pose.pose.position.x;
  double y_end = end_pose.pose.position.y;
  double dx = x_end - x_start;
  double dy = y_end - y_start;
  double trip_len = sqrt(dx * dx + dy * dy);
  double ramp_up_dist = 0.5 * speed_max_ * speed_max_ / alpha_max_;
  ROS_INFO("trip len = %f", trip_len);
  if (trip_len < 2.0 * ramp_up_dist)
  {
    build_triangular_travel_traj(start_pose, end_pose, vec_of_states);
  }
  else
  {
    build_trapezoidal_travel_traj(start_pose, end_pose, vec_of_states);
  }
}

void TrajBuilder::build_trapezoidal_travel_traj(geometry_msgs::msg::PoseStamped start_pose,
                                                geometry_msgs::msg::PoseStamped end_pose,
                                                std::vector<nav_msgs::msg::Odometry> &vec_of_states)
{
  double x_start = start_pose.pose.position.x;
  double y_start = start_pose.pose.position.y;
  double x_end = end_pose.pose.position.x;
  double y_end = end_pose.pose.position.y;
  double dx = x_end - x_start;
  double dy = y_end - y_start;
  double psi_des = atan2(dy, dx);
  double trip_len = sqrt(dx * dx + dy * dy);
  double t_ramp = speed_max_ / accel_max_;
  double ramp_up_dist = 0.5 * accel_max_ * t_ramp * t_ramp;
  double cruise_distance = trip_len - 2.0 * ramp_up_dist;
  ROS_INFO("t_ramp =%f", t_ramp);
  ROS_INFO("ramp-up dist = %f", ramp_up_dist);
  ROS_INFO("cruise distance = %f", cruise_distance);

  nav_msgs::msg::Odometry des_state;
  des_state.header = start_pose.header;
  des_state.pose.pose = start_pose.pose;
  des_state.twist.twist = halt_twist_;
  int npts_ramp = round(t_ramp / dt_);
  double x_des = x_start;
  double y_des = y_start;
  double speed_des = 0.0;
  des_state.twist.twist.angular.z = 0.0;
  des_state.pose.pose.orientation = convertPlanarPsi2Quaternion(psi_des);

  double t = 0.0;

  for (int i = 0; i < npts_ramp; i++)
  {
    t += dt_;
    speed_des = accel_max_ * t;
    des_state.twist.twist.linear.x = speed_des;

    x_des = x_start + 0.5 * accel_max_ * t * t * cos(psi_des);
    y_des = y_start + 0.5 * accel_max_ * t * t * sin(psi_des);
    des_state.pose.pose.position.x = x_des;
    des_state.pose.pose.position.y = y_des;
    vec_of_states.push_back(des_state);
  }

  speed_des = speed_max_;
  des_state.twist.twist.linear.x = speed_des;
  double t_cruise = cruise_distance / speed_max_;
  int npts_cruise = round(t_cruise / dt_);
  ROS_INFO("t_cruise = %f; npts_cruise = %d\n", t_cruise, npts_cruise);
  for (int i = 0; i < npts_cruise; i++)
  {
    x_des += speed_des * dt_ * cos(psi_des);
    y_des += speed_des * dt_ * sin(psi_des);
    des_state.pose.pose.position.x = x_des;
    des_state.pose.pose.position.y = y_des;
    vec_of_states.push_back(des_state);
  }

  for (int i = 0; i < npts_ramp; i++)
  {
    speed_des -= accel_max_ * dt_;
    des_state.twist.twist.linear.x = speed_des;
    x_des += speed_des * dt_ * cos(psi_des);
    y_des += speed_des * dt_ * sin(psi_des);
    des_state.pose.pose.position.x = x_des;
    des_state.pose.pose.position.y = y_des;
    vec_of_states.push_back(des_state);
  }

  des_state.pose.pose = end_pose.pose;

  des_state.pose.pose.orientation = convertPlanarPsi2Quaternion(psi_des);
  des_state.twist.twist = halt_twist_;
  vec_of_states.push_back(des_state);
}

void TrajBuilder::build_triangular_travel_traj(geometry_msgs::msg::PoseStamped start_pose,
                                               geometry_msgs::msg::PoseStamped end_pose,
                                               std::vector<nav_msgs::msg::Odometry> &vec_of_states)
{
  double x_start = start_pose.pose.position.x;
  double y_start = start_pose.pose.position.y;
  double x_end = end_pose.pose.position.x;
  double y_end = end_pose.pose.position.y;
  double dx = x_end - x_start;
  double dy = y_end - y_start;
  double psi_des = atan2(dy, dx);
  nav_msgs::msg::Odometry des_state;
  des_state.header = start_pose.header;
  des_state.pose.pose = start_pose.pose;
  des_state.twist.twist = halt_twist_;
  double trip_len = sqrt(dx * dx + dy * dy);
  double t_ramp = sqrt(trip_len / accel_max_);
  int npts_ramp = round(t_ramp / dt_);
  // double v_peak = accel_max_ * t_ramp;
  // double d_vel = alpha_max_ * dt_;

  double x_des = x_start;
  double y_des = y_start;
  double speed_des = 0.0;
  des_state.twist.twist.angular.z = 0.0;
  des_state.pose.pose.orientation = convertPlanarPsi2Quaternion(psi_des);

  double t = 0.0;

  for (int i = 0; i < npts_ramp; i++)
  {
    t += dt_;
    speed_des = accel_max_ * t;
    des_state.twist.twist.linear.x = speed_des;

    x_des = x_start + 0.5 * accel_max_ * t * t * cos(psi_des);
    y_des = y_start + 0.5 * accel_max_ * t * t * sin(psi_des);
    des_state.pose.pose.position.x = x_des;
    des_state.pose.pose.position.y = y_des;
    vec_of_states.push_back(des_state);
  }

  for (int i = 0; i < npts_ramp; i++)
  {
    speed_des -= accel_max_ * dt_;
    des_state.twist.twist.linear.x = speed_des;
    x_des += speed_des * dt_ * cos(psi_des);
    y_des += speed_des * dt_ * sin(psi_des);
    des_state.pose.pose.position.x = x_des;
    des_state.pose.pose.position.y = y_des;
    vec_of_states.push_back(des_state);
  }

  des_state.pose.pose = end_pose.pose;

  des_state.pose.pose.orientation = convertPlanarPsi2Quaternion(psi_des);
  des_state.twist.twist = halt_twist_;
  vec_of_states.push_back(des_state);
}

void TrajBuilder::build_triangular_spin_traj(geometry_msgs::msg::PoseStamped start_pose,
                                             geometry_msgs::msg::PoseStamped end_pose,
                                             std::vector<nav_msgs::msg::Odometry> &vec_of_states)
{
  nav_msgs::msg::Odometry des_state;
  des_state.header = start_pose.header;
  des_state.pose.pose = start_pose.pose;
  des_state.twist.twist = halt_twist_;
  vec_of_states.push_back(des_state);
  double psi_start = convertPlanarQuat2Psi(start_pose.pose.orientation);
  double psi_end = convertPlanarQuat2Psi(end_pose.pose.orientation);
  double dpsi = min_dang(psi_end - psi_start);
  ROS_INFO("spin traj: psi_start = %f; psi_end = %f; dpsi= %f\n", psi_start, psi_end, dpsi);
  double t_ramp = sqrt(fabs(dpsi) / alpha_max_);
  int npts_ramp = round(t_ramp / dt_);
  double psi_des = psi_start;
  double omega_des = 0.0;

  double t = 0.0;
  double accel = sgn(dpsi) * alpha_max_;

  for (int i = 0; i < npts_ramp; i++)
  {
    t += dt_;
    omega_des = accel * t;
    des_state.twist.twist.angular.z = omega_des;

    psi_des = psi_start + 0.5 * accel * t * t;
    des_state.pose.pose.orientation = convertPlanarPsi2Quaternion(psi_des);
    vec_of_states.push_back(des_state);
  }

  for (int i = 0; i < npts_ramp; i++)
  {
    omega_des -= accel * dt_;
    des_state.twist.twist.angular.z = omega_des;
    psi_des += omega_des * dt_;
    des_state.pose.pose.orientation = convertPlanarPsi2Quaternion(psi_des);
    vec_of_states.push_back(des_state);
  }

  des_state.pose.pose = end_pose.pose;
  des_state.twist.twist = halt_twist_;
  vec_of_states.push_back(des_state);
}

void TrajBuilder::build_braking_traj(geometry_msgs::msg::PoseStamped start_pose,
                                     std::vector<nav_msgs::msg::Odometry> &vec_of_states)
{
  /* FIXME implement this */
}

void TrajBuilder::build_point_and_go_traj(geometry_msgs::msg::PoseStamped start_pose,
                                          geometry_msgs::msg::PoseStamped end_pose,
                                          std::vector<nav_msgs::msg::Odometry> &vec_of_states)
{
  ROS_INFO("building point-and-go trajectory");
  nav_msgs::msg::Odometry bridge_state;
  geometry_msgs::msg::PoseStamped bridge_pose;
  vec_of_states.clear();
  ROS_INFO("building rotational trajectory");
  double x_start = start_pose.pose.position.x;
  double y_start = start_pose.pose.position.y;
  double x_end = end_pose.pose.position.x;
  double y_end = end_pose.pose.position.y;
  double dx = x_end - x_start;
  double dy = y_end - y_start;
  double des_psi = atan2(dy, dx);
  ROS_INFO("desired heading to subgoal = %f", des_psi);

  bridge_pose = start_pose;
  bridge_pose.pose.orientation = convertPlanarPsi2Quaternion(des_psi);
  ROS_INFO("building reorientation trajectory");
  build_spin_traj(start_pose, bridge_pose, vec_of_states);

  ROS_INFO("building translational trajectory");
  build_travel_traj(bridge_pose, end_pose, vec_of_states);
}
