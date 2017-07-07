#ifndef PUB_DES_STATE_H_
#define PUB_DES_STATE_H_

#include <queue>
#include <vector>

#include "mobot_pub_des_state/srv/path.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "traj_builder/traj_builder.hpp"  //has almost all the headers we need

#include "rcutils/logging_macros.h"
#define ROS_INFO RCUTILS_LOG_INFO
#define ROS_WARN RCUTILS_LOG_WARN

// https://stackoverflow.com/a/15764679
template <typename T>
void ignore(T &&)
{
}

// constants and parameters:
const double dt = 0.02;  // send desired-state messages at fixed rate, e.g. 0.02 sec = 50Hz
// dynamic parameters: should be tuned for target system
const double accel_max = 0.5;       // 1m/sec^2
const double alpha_max = 0.2;       // rad/sec^2
const double speed_max = 1.0;       // 1 m/sec
const double omega_max = 1.0;       // 1 rad/sec
const double path_move_tol = 0.01;  // if path points are within 1cm, fuggidaboutit

// define some mode keywords
enum SubgoalState
{
  E_STOPPED,
  DONE_W_SUBGOAL,
  PURSUING_SUBGOAL,
  HALTING
};
// const int E_STOPPED = 0;
// const int DONE_W_SUBGOAL = 1;
// const int PURSUING_SUBGOAL = 2;
// const int HALTING = 3;

class DesStatePublisher
{
private:
  rclcpp::node::Node::SharedPtr node_;

  nav_msgs::msg::Path path_;
  std::vector<nav_msgs::msg::Odometry> des_state_vec_;
  nav_msgs::msg::Odometry des_state_;
  nav_msgs::msg::Odometry halt_state_;
  nav_msgs::msg::Odometry seg_end_state_;
  nav_msgs::msg::Odometry seg_start_state_;
  nav_msgs::msg::Odometry current_des_state_;
  geometry_msgs::msg::Twist halt_twist_;
  geometry_msgs::msg::PoseStamped start_pose_;
  geometry_msgs::msg::PoseStamped end_pose_;
  geometry_msgs::msg::PoseStamped current_pose_;
  std_msgs::msg::Float64 float_msg_;
  double des_psi_;
  std::queue<geometry_msgs::msg::PoseStamped> path_queue_;

  SubgoalState motion_mode_;
  bool e_stop_trigger_;
  bool e_stop_reset_;
  int traj_pt_i_;
  int npts_traj_;
  double dt_;

  // dynamic parameters: should be tuned for target system
  double accel_max_;
  double alpha_max_;
  double speed_max_;
  double omega_max_;
  double path_move_tol_;

  rclcpp::service::Service<std_srvs::srv::Trigger>::SharedPtr estop_service_;
  rclcpp::service::Service<std_srvs::srv::Trigger>::SharedPtr estop_clear_service_;
  rclcpp::service::Service<std_srvs::srv::Trigger>::SharedPtr flush_path_queue_;
  rclcpp::service::Service<mobot_pub_des_state::srv::Path>::SharedPtr append_path_;

  rclcpp::publisher::Publisher<nav_msgs::msg::Odometry>::SharedPtr desired_state_publisher_;
  rclcpp::publisher::Publisher<std_msgs::msg::Float64>::SharedPtr des_psi_publisher_;

  TrajBuilder trajBuilder_;

  void initializePublishers();
  void initializeServices();

public:
  DesStatePublisher(rclcpp::node::Node::SharedPtr node);
  SubgoalState get_motion_mode()
  {
    return motion_mode_;
  }
  void set_motion_mode(SubgoalState mode)
  {
    motion_mode_ = mode;
  }
  bool get_estop_trigger()
  {
    return e_stop_trigger_;
  }
  void reset_estop_trigger()
  {
    e_stop_trigger_ = false;
  }
  void set_init_pose(double x, double y, double psi);
  void pub_next_state();
  void append_path_queue(geometry_msgs::msg::PoseStamped pose)
  {
    path_queue_.push(pose);
  }
  void append_path_queue(double x, double y, double psi)
  {
    path_queue_.push(trajBuilder_.xyPsi2PoseStamped(x, y, psi));
  }
};
#endif