#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <math.h>
#include <stdio.h>
#include <string.h>

const double R_LEFT_WHEEL = 0.3302 / 2.0;
const double R_RIGHT_WHEEL = R_LEFT_WHEEL + 0.005;

const double TRACK = 0.560515;

const double wheel_ang_sham_init = -1000000.0;
bool joints_states_good = false;

rclcpp::subscription::Subscription<sensor_msgs::msg::JointState>::SharedPtr g_joint_state_subscriber;
rclcpp::publisher::Publisher<nav_msgs::msg::Odometry>::SharedPtr g_drifty_odom_pub;

nav_msgs::msg::Odometry::SharedPtr g_drifty_odom;

double g_new_left_wheel_ang, g_old_left_wheel_ang;
double g_new_right_wheel_ang, g_old_right_wheel_ang;
double g_t_new, g_t_old, g_dt;
double g_odom_psi;

geometry_msgs::msg::Quaternion convertPlanarPsi2Quaternion(double psi)
{
  geometry_msgs::msg::Quaternion quaternion;
  quaternion.x = 0.0;
  quaternion.y = 0.0;
  quaternion.z = sin(psi / 2.0);
  quaternion.w = cos(psi / 2.0);
  return quaternion;
}

void joint_state_CB(const sensor_msgs::msg::JointState::SharedPtr joint_states)
{
  double dtheta_right, dtheta_left, ds, dpsi;
  int n_joints = joint_states->name.size();
  int ijnt;
  int njnts_found = 0;

  g_old_left_wheel_ang = g_new_left_wheel_ang;
  g_old_right_wheel_ang = g_new_right_wheel_ang;
  g_t_old = g_t_new;
  g_t_new = (double)std::chrono::system_clock::now().time_since_epoch().count();
  g_dt = g_t_new - g_t_old;

  for (ijnt = 0; ijnt < n_joints; ijnt++)
  {
    std::string joint_name(joint_states->name[ijnt]);
    if (joint_name.compare("left_wheel_joint") == 0)
    {
      g_new_left_wheel_ang = joint_states->position[ijnt];
      njnts_found++;
    }
    if (joint_name.compare("right_wheel_joint") == 0)
    {
      g_new_right_wheel_ang = joint_states->position[ijnt];
      njnts_found++;
    }
  }
  if (njnts_found < 2)
  {
  }
  else
  {
  }
  if (!joints_states_good)
  {
    if (g_new_left_wheel_ang > wheel_ang_sham_init / 2.0)
    {
      joints_states_good = true;
      g_old_left_wheel_ang = g_new_left_wheel_ang;
      g_old_right_wheel_ang = g_new_right_wheel_ang;
    }
  }
  if (joints_states_good)
  {
    dtheta_left = g_new_left_wheel_ang - g_old_left_wheel_ang;
    dtheta_right = g_new_right_wheel_ang - g_old_right_wheel_ang;
    ds = 0.5 * (dtheta_left * R_LEFT_WHEEL + dtheta_right * R_RIGHT_WHEEL);
    dpsi = dtheta_right * R_RIGHT_WHEEL / TRACK - dtheta_left * R_LEFT_WHEEL / TRACK;

    g_drifty_odom->pose.pose.position.x += ds * cos(g_odom_psi);
    g_drifty_odom->pose.pose.position.y += ds * sin(g_odom_psi);
    g_odom_psi += dpsi;

    g_drifty_odom->pose.pose.orientation = convertPlanarPsi2Quaternion(g_odom_psi);

    if (g_dt > 0.0005)
    {
      g_drifty_odom->twist.twist.linear.x = ds / g_dt;
      g_drifty_odom->twist.twist.angular.z = dpsi / g_dt;
    }
    builtin_interfaces::msg::Time t0;
    t0.sec = g_t_new;
    g_drifty_odom->header.stamp = t0;
    g_drifty_odom_pub->publish(g_drifty_odom);
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("drifty_odom_publisher");

  g_new_left_wheel_ang = wheel_ang_sham_init;
  g_old_left_wheel_ang = wheel_ang_sham_init;
  g_new_right_wheel_ang = wheel_ang_sham_init;
  g_old_right_wheel_ang = wheel_ang_sham_init;
  g_t_new = (double)std::chrono::system_clock::now().time_since_epoch().count();
  g_t_old = g_t_new;

  g_drifty_odom = std::make_shared<nav_msgs::msg::Odometry>();

  g_drifty_odom->child_frame_id = "base_link";
  g_drifty_odom->header.frame_id = "drifty_odom";
  builtin_interfaces::msg::Time t0;
  t0.sec = g_t_new;
  g_drifty_odom->header.stamp = t0;
  g_drifty_odom->pose.pose.position.x = 0.0;
  g_drifty_odom->pose.pose.position.y = 0.0;
  g_drifty_odom->pose.pose.position.z = 0.0;
  g_drifty_odom->pose.pose.orientation.x = 0.0;
  g_drifty_odom->pose.pose.orientation.y = 0.0;
  g_drifty_odom->pose.pose.orientation.z = 0.0;
  g_drifty_odom->pose.pose.orientation.w = 1.0;

  g_drifty_odom->twist.twist.linear.x = 0.0;
  g_drifty_odom->twist.twist.linear.y = 0.0;
  g_drifty_odom->twist.twist.linear.z = 0.0;
  g_drifty_odom->twist.twist.angular.x = 0.0;
  g_drifty_odom->twist.twist.angular.y = 0.0;
  g_drifty_odom->twist.twist.angular.z = 0.0;

  g_drifty_odom_pub = node->create_publisher<nav_msgs::msg::Odometry>("drifty_odom", rmw_qos_profile_default);
  g_joint_state_subscriber =
      node->create_subscription<sensor_msgs::msg::JointState>("joint_states", joint_state_CB, rmw_qos_profile_default);

  rclcpp::spin(node);
}
