#ifndef MOBOT_NL_STEERING_H_
#define MOBOT_NL_STEERING_H_

#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float32.hpp"

#define ROS_INFO printf

const double UPDATE_RATE = 100.0;
const double K_PSI = 5.0;
const double K_LAT_ERR_THRESH = 3.0;

const double MAX_SPEED = 1.0;
const double MAX_OMEGA = 1.0;

class SteeringController
{
public:
  SteeringController(rclcpp::Node::SharedPtr node);
  void mobot_nl_steering();
  static double psi_strategy(double);
  double omega_cmd_fnc(double psi_strategy, double psi_state, double psi_path);
  static double convertPlanarQuat2Phi(geometry_msgs::msg::Quaternion quaternion);
  static double min_dang(double dang);
  static double sat(double x);

private:
  rclcpp::Node::SharedPtr node_;

  rclcpp::subscription::Subscription<nav_msgs::msg::Odometry>::SharedPtr des_state_subscriber_;
  rclcpp::subscription::Subscription<geometry_msgs::msg::Pose>::SharedPtr current_state_subscriber_;
  rclcpp::publisher::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
  rclcpp::publisher::Publisher<std_msgs::msg::Float32>::SharedPtr lat_err_publisher_;
  rclcpp::publisher::Publisher<std_msgs::msg::Float32>::SharedPtr heading_publisher_;
  rclcpp::publisher::Publisher<std_msgs::msg::Float32>::SharedPtr heading_cmd_publisher_;

  geometry_msgs::msg::Twist twist_cmd_;

  double psi_cmd_;
  double lateral_err_;

  double current_speed_des_;
  double current_omega_des_;

  double state_x_;
  double state_y_;
  double state_psi_;
  double state_speed_;
  double state_omega_;

  geometry_msgs::msg::Quaternion state_quat_;

  double des_state_x_;
  double des_state_y_;
  double des_state_psi_;

  double des_state_speed_;
  double des_state_omega_;

  geometry_msgs::msg::Quaternion des_state_quat_;
  geometry_msgs::msg::Pose des_state_pose_;

  void initializeSubscribers();
  void initializePublishers();

  void gazeboPoseCallback(const geometry_msgs::msg::Pose::SharedPtr gazebo_pose);
  void desStateCallback(const nav_msgs::msg::Odometry::SharedPtr des_state_rcvd);
};

#endif
