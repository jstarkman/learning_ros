#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64.hpp"
#include "xform_utils/xform_utils.hpp"

#define ROS_INFO printf

const double MAIN_DT = 0.01;
const double K_YAW = 0.1;
const double K_GPS = 0.002;
const double L_MOVE = 0.1;

XformUtils xform_utils;

double g_omega_z_imu = 0;
bool g_imu_good = false;

double g_x_gps = 0;
double g_y_gps = 0;
bool g_gps_good = false;

bool g_odom_good = false;
double g_odom_speed = 0;

double g_true_yaw = 0;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("gps_localizer");

  geometry_msgs::msg::PoseStamped pose_estimate;
  std_msgs::msg::Float64 yaw_msg;
  geometry_msgs::msg::Quaternion quat_est;
  // double dx_odom_est = 0;
  // double dy_odom_est = 0;
  double dl_odom_est = 0;

  double yaw_est = 0;
  double x_est = 0;
  double y_est = 0;
  double x_est_old = 0;
  double y_est_old = 0;

  rclcpp::WallRate timer(1 / MAIN_DT);
  auto gps_subscriber =
      node->create_subscription<geometry_msgs::msg::Pose>("gazebo_mobot_noisy_pose",
                                                          [](geometry_msgs::msg::Pose::SharedPtr gazebo_pose) {
                                                            g_x_gps = gazebo_pose->position.x;
                                                            g_y_gps = gazebo_pose->position.y;
                                                            g_gps_good = true;
                                                          },
                                                          rmw_qos_profile_default);

  auto imu_subscriber =
      node->create_subscription<sensor_msgs::msg::Imu>("/imu_data",
                                                       [](const sensor_msgs::msg::Imu::SharedPtr imu_rcvd) {
                                                         g_omega_z_imu = imu_rcvd->angular_velocity.z;
                                                         g_imu_good = true;
                                                       },
                                                       rmw_qos_profile_default);

  auto odom_subscriber =
      node->create_subscription<nav_msgs::msg::Odometry>("/drifty_odom",
                                                         [](const nav_msgs::msg::Odometry::SharedPtr odom_rcvd) {
                                                           g_odom_speed = odom_rcvd->twist.twist.linear.x;
                                                           g_odom_good = true;
                                                         },
                                                         rmw_qos_profile_default);

  auto true_state_subscriber = node->create_subscription<geometry_msgs::msg::Pose>(
      "gazebo_mobot_pose",
      [](const geometry_msgs::msg::Pose::SharedPtr gazebo_pose) {
        g_true_yaw = xform_utils.convertPlanarQuat2Phi(gazebo_pose->orientation);
      },
      rmw_qos_profile_default);

  auto localization_publisher =
      node->create_publisher<geometry_msgs::msg::PoseStamped>("/mobot_localization", rmw_qos_profile_default);
  auto yaw_publisher = node->create_publisher<std_msgs::msg::Float64>("/yaw_estimate", rmw_qos_profile_default);
  auto true_yaw_publisher = node->create_publisher<std_msgs::msg::Float64>("/true_yaw", rmw_qos_profile_default);

  ROS_INFO("warm up callbacks: ");
  ROS_INFO("waiting on gps: ");

  rclcpp::WallRate hundredMillisecondSleep(10);

  while (!g_gps_good)
  {
    rclcpp::spin_some(node);
    hundredMillisecondSleep.sleep();
  }
  builtin_interfaces::msg::Time t0;
  t0.sec = (double)std::chrono::system_clock::now().time_since_epoch().count();
  pose_estimate.header.stamp = t0;
  pose_estimate.pose.position.x = g_x_gps;
  pose_estimate.pose.position.y = g_y_gps;
  pose_estimate.pose.position.z = 0;

  ROS_INFO("got gps info; wait for imu:");
  while (!g_imu_good)
  {
    rclcpp::spin_some(node);
    hundredMillisecondSleep.sleep();
  }
  ROS_INFO("imu is good; wait for odom:");
  while (!g_odom_good)
  {
    rclcpp::spin_some(node);
    hundredMillisecondSleep.sleep();
  }
  double dang_gps = 0;
  double dang_odom = 0;
  double dx_odom, dy_odom;
  double delta_odom_x = 0;
  double delta_odom_y = 0;
  double yaw_err = 0;
  double move_dist = 0;

  x_est = g_x_gps;
  x_est_old = x_est;
  y_est = g_y_gps;
  y_est_old = y_est;
  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);

    x_est = (1 - K_GPS) * x_est + K_GPS * g_x_gps;
    y_est = (1 - K_GPS) * y_est + K_GPS * g_y_gps;
    dl_odom_est = MAIN_DT * g_odom_speed;
    move_dist += dl_odom_est;
    yaw_est += MAIN_DT * g_omega_z_imu;
    if (yaw_est < -M_PI)
      yaw_est += 2.0 * M_PI;
    if (yaw_est > M_PI)
      yaw_est -= 2.0 * M_PI;

    dx_odom = dl_odom_est * cos(yaw_est);
    dy_odom = dl_odom_est * sin(yaw_est);
    x_est += dx_odom;
    y_est += dy_odom;
    delta_odom_x += dx_odom;
    delta_odom_y += dy_odom;
    if (fabs(move_dist) > L_MOVE)
    {
      dang_gps = atan2((y_est - y_est_old), (x_est - x_est_old));

      dang_odom = atan2(delta_odom_y, delta_odom_x);

      yaw_err = dang_gps - dang_odom;
      if (yaw_err > M_PI)
        yaw_err -= 2.0 * M_PI;
      if (yaw_err < -M_PI)
        yaw_err += 2.0 * M_PI;

      yaw_est += K_YAW * yaw_err;

      y_est_old = y_est;
      x_est_old = x_est;
      move_dist = 0;
      delta_odom_y = 0;
      delta_odom_x = 0;
    }

    yaw_msg.data = yaw_est;
    yaw_publisher->publish(yaw_msg);

    yaw_msg.data = g_true_yaw;
    true_yaw_publisher->publish(yaw_msg);

    builtin_interfaces::msg::Time t0;
    t0.sec = (double)std::chrono::system_clock::now().time_since_epoch().count();
    pose_estimate.header.stamp = t0;
    pose_estimate.pose.position.x = x_est;
    pose_estimate.pose.position.y = y_est;

    quat_est = xform_utils.convertPlanarPsi2Quaternion(yaw_est);
    pose_estimate.pose.orientation = quat_est;
    localization_publisher->publish(pose_estimate);

    timer.sleep();
  }
  return 0;
}