#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64.hpp"
#include "traj_builder/traj_builder.hpp"

geometry_msgs::msg::Twist g_halt_twist;
nav_msgs::msg::Odometry g_end_state;
nav_msgs::msg::Odometry g_start_state;
geometry_msgs::msg::PoseStamped g_start_pose;
geometry_msgs::msg::PoseStamped g_end_pose;

void do_inits()
{
  g_halt_twist.linear.x = 0.0;
  g_halt_twist.linear.y = 0.0;
  g_halt_twist.linear.z = 0.0;
  g_halt_twist.angular.x = 0.0;
  g_halt_twist.angular.y = 0.0;
  g_halt_twist.angular.z = 0.0;

  g_start_state.twist.twist = g_halt_twist;
  g_start_state.pose.pose.position.x = 0;
  g_start_state.pose.pose.position.y = 0;
  g_start_state.pose.pose.position.z = 0;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::node::Node::make_shared("des_state_publisher");
  double dt = 0.02;

  auto des_state_publisher = node->create_publisher<nav_msgs::msg::Odometry>("/desState", rmw_qos_profile_default);
  auto des_psi_publisher = node->create_publisher<std_msgs::msg::Float64>("/desPsi", rmw_qos_profile_default);
  auto twist_commander = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", rmw_qos_profile_default);

  rclcpp::WallRate looprate(1 / dt);
  TrajBuilder trajBuilder;
  trajBuilder.set_dt(dt);
  trajBuilder.set_alpha_max(1.0);

  double psi_start = 0.0;
  double psi_end = 0.0;
  g_start_state.pose.pose.orientation = trajBuilder.convertPlanarPsi2Quaternion(psi_start);
  g_end_state = g_start_state;
  g_end_state.pose.pose.orientation = trajBuilder.convertPlanarPsi2Quaternion(psi_end);

  g_start_pose.pose.position.x = 0.0;
  g_start_pose.pose.position.y = 0.0;
  g_start_pose.pose.position.z = 0.0;
  g_start_pose.pose.orientation = trajBuilder.convertPlanarPsi2Quaternion(psi_start);
  g_end_pose = g_start_pose;

  g_end_pose.pose.orientation = trajBuilder.convertPlanarPsi2Quaternion(psi_end);
  g_end_pose.pose.position.x = 5.0;
  g_end_pose.pose.position.y = 0.0;

  double des_psi;
  std_msgs::msg::Float64 psi_msg;
  std::vector<nav_msgs::msg::Odometry> vec_of_states;

  nav_msgs::msg::Odometry des_state;
  nav_msgs::msg::Odometry last_state;
  geometry_msgs::msg::PoseStamped last_pose;

  builtin_interfaces::msg::Time t0;

  /*timing test: takes about 1.3 msec to compute a point-and-go trajectory
  ROS_INFO("timing test: constructing 10000 point-and-go trajectories");
  for (int ipaths=0;ipaths<10000;ipaths++) {
      trajBuilder.build_point_and_go_traj(g_start_pose, g_end_pose, vec_of_states);
  }
  ROS_INFO("done computing trajectories");
   * */

  while (rclcpp::ok())
  {
    ROS_INFO("building traj from start to end");
    trajBuilder.build_point_and_go_traj(g_start_pose, g_end_pose, vec_of_states);
    ROS_INFO("publishing desired states and open-loop cmd_vel");
    for (int i = 0; i < vec_of_states.size(); i++)
    {
      t0.sec = (double)std::chrono::system_clock::now().time_since_epoch().count();
      des_state.header.stamp = t0;
      des_state = vec_of_states[i];
      des_state_publisher->publish(des_state);
      des_psi = trajBuilder.convertPlanarQuat2Psi(des_state.pose.pose.orientation);
      psi_msg.data = des_psi;
      des_psi_publisher->publish(psi_msg);
      twist_commander->publish(des_state.twist.twist);

      looprate.sleep();
    }
    ROS_INFO("building traj from end to start");
    last_state = vec_of_states.back();
    last_pose.header = last_state.header;
    last_pose.pose = last_state.pose.pose;
    trajBuilder.build_point_and_go_traj(last_pose, g_start_pose, vec_of_states);
    for (int i = 0; i < vec_of_states.size(); i++)
    {
      des_state = vec_of_states[i];
      t0.sec = (double)std::chrono::system_clock::now().time_since_epoch().count();
      des_state.header.stamp = t0;
      des_state_publisher->publish(des_state);
      des_psi = trajBuilder.convertPlanarQuat2Psi(des_state.pose.pose.orientation);
      psi_msg.data = des_psi;
      des_psi_publisher->publish(psi_msg);
      twist_commander->publish(des_state.twist.twist);
      looprate.sleep();
    }
    last_state = vec_of_states.back();
    g_start_pose.header = last_state.header;
    g_start_pose.pose = last_state.pose.pose;
  }
}
