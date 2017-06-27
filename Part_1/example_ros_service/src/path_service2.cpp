#include "nav_msgs/msg/path.hpp"
#include <math.h>
#include <iostream>
#include <string>
#include "example_ros_service/srv/path_srv.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

/* ugly hack until rosconsole works */
#define ROS_INFO printf

const double g_move_speed = 1.0;
const double g_spin_speed = 1.0;
const double g_sample_dt = 0.01;
const double g_dist_tol = 0.01;

geometry_msgs::msg::Twist g_twist_cmd;
rclcpp::publisher::Publisher<geometry_msgs::msg::Twist>::SharedPtr g_twist_commander;
geometry_msgs::msg::Pose g_current_pose;

double sgn(double x);
double min_spin(double spin_angle);
double convertPlanarQuat2Phi(geometry_msgs::msg::Quaternion quaternion);
geometry_msgs::msg::Quaternion convertPlanarPhi2Quaternion(double phi);

void do_halt();
void do_move(double distance);
void do_spin(double spin_ang);

double sgn(double x)
{
  if (x > 0.0)
  {
    return 1.0;
  }
  else if (x < 0.0)
  {
    return -1.0;
  }
  else
  {
    return 0.0;
  }
}

double min_spin(double spin_angle)
{
  if (spin_angle > M_PI)
  {
    spin_angle -= 2.0 * M_PI;
  }
  if (spin_angle < -M_PI)
  {
    spin_angle += 2.0 * M_PI;
  }
  return spin_angle;
}

double convertPlanarQuat2Phi(geometry_msgs::msg::Quaternion quaternion)
{
  double quat_z = quaternion.z;
  double quat_w = quaternion.w;
  double phi = 2.0 * atan2(quat_z, quat_w);
  return phi;
}

geometry_msgs::msg::Quaternion convertPlanarPhi2Quaternion(double phi)
{
  geometry_msgs::msg::Quaternion quaternion;
  quaternion.x = 0.0;
  quaternion.y = 0.0;
  quaternion.z = std::sin(phi / 2.0);
  quaternion.w = std::cos(phi / 2.0);
  return quaternion;
}

void do_spin(double spin_ang)
{
  rclcpp::rate::WallRate loop_timer(1 / g_sample_dt);
  double timer = 0.0;
  double final_time = fabs(spin_ang) / g_spin_speed;
  g_twist_cmd.angular.z = sgn(spin_ang) * g_spin_speed;
  while (timer < final_time)
  {
    g_twist_commander->publish(g_twist_cmd);
    timer += g_sample_dt;
    loop_timer.sleep();
  }
  do_halt();
}

void do_move(double distance)
{
  rclcpp::rate::WallRate loop_timer(1 / g_sample_dt);
  double timer = 0.0;
  double final_time = fabs(distance) / g_move_speed;
  g_twist_cmd.angular.z = 0.0;
  g_twist_cmd.linear.x = sgn(distance) * g_move_speed;
  while (timer < final_time)
  {
    g_twist_commander->publish(g_twist_cmd);
    timer += g_sample_dt;
    loop_timer.sleep();
  }
  do_halt();
}

void do_halt()
{
  rclcpp::rate::WallRate loop_timer(1 / g_sample_dt);
  g_twist_cmd.angular.z = 0.0;
  g_twist_cmd.linear.x = 0.0;
  for (int i = 0; i < 10; i++)
  {
    g_twist_commander->publish(g_twist_cmd);
    std::cout << "sent halt command" << std::endl;
    loop_timer.sleep();
  }
}

void get_yaw_and_dist(geometry_msgs::msg::Pose current_pose, geometry_msgs::msg::Pose goal_pose, double &dist,
                      double &heading)
{
  dist = 0.0;
  if (dist < g_dist_tol)
  {
    heading = convertPlanarQuat2Phi(goal_pose.orientation);
  }
  else
  {
    heading = 0.0;
  }
}

void callback(const std::shared_ptr<rmw_request_id_t> request_header,
              const std::shared_ptr<example_ros_service::srv::PathSrv::Request> request,
              std::shared_ptr<example_ros_service::srv::PathSrv::Response> response)
{
  (void)request_header;
  std::cout << "callback activated" << std::endl;
  double yaw_desired, yaw_current, travel_distance, spin_angle;
  geometry_msgs::msg::Pose pose_desired;
  int npts = request->nav_path.poses.size();
  std::cout << "received path request with " << npts << " poses" << std::endl;

  for (int i = 0; i < npts; i++)
  {
    pose_desired = request->nav_path.poses[i].pose;

    yaw_desired = convertPlanarQuat2Phi(pose_desired.orientation);

    ROS_INFO("pose %d: desired yaw = %f\n", i, yaw_desired);
    yaw_current = convertPlanarQuat2Phi(g_current_pose.orientation);
    spin_angle = yaw_desired - yaw_current;
    spin_angle = min_spin(spin_angle);
    do_spin(spin_angle);

    g_current_pose.orientation = pose_desired.orientation;

    do_move(1.0);
  }
}

void do_inits(rclcpp::node::Node::SharedPtr node)
{
  g_twist_cmd.linear.x = 0.0;
  g_twist_cmd.linear.y = 0.0;
  g_twist_cmd.linear.z = 0.0;
  g_twist_cmd.angular.x = 0.0;
  g_twist_cmd.angular.y = 0.0;
  g_twist_cmd.angular.z = 0.0;

  g_current_pose.position.x = 0.0;
  g_current_pose.position.y = 0.0;
  g_current_pose.position.z = 0.0;

  g_current_pose.orientation.x = 0.0;
  g_current_pose.orientation.y = 0.0;
  g_current_pose.orientation.z = 0.0;
  g_current_pose.orientation.w = 1.0;

  auto pub = node->create_publisher<geometry_msgs::msg::Twist>("/robot0/cmd_vel", rmw_qos_profile_default);
  g_twist_commander = pub;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::node::Node::make_shared("path_service");
  auto service = node->create_service<example_ros_service::srv::PathSrv>("path_service", callback);
  do_inits(node);

  ROS_INFO("Ready to accept paths.");
  rclcpp::spin(node);

  return 0;
}
