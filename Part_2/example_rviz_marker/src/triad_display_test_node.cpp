#define _USE_MATH_DEFINES
#include <math.h>
#include <chrono>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

geometry_msgs::msg::Quaternion convertPlanarPsi2Quaternion(double psi)
{
  geometry_msgs::msg::Quaternion quaternion;
  quaternion.x = 0.0;
  quaternion.y = 0.0;
  quaternion.z = sin(psi / 2.0);
  quaternion.w = cos(psi / 2.0);
  return (quaternion);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::node::Node::make_shared("triad_display_test_node");
  auto pose_publisher =
      node->create_publisher<geometry_msgs::msg::PoseStamped>("triad_display_pose", rmw_qos_profile_default);

  auto desired_triad_pose = std::make_shared<geometry_msgs::msg::PoseStamped>();

  desired_triad_pose->pose.position.x = 0.0;
  desired_triad_pose->pose.position.y = 0.0;
  desired_triad_pose->pose.position.z = 0.0;
  desired_triad_pose->pose.orientation.x = 0.0;
  desired_triad_pose->pose.orientation.y = 0.0;
  desired_triad_pose->pose.orientation.z = 0.0;
  desired_triad_pose->pose.orientation.w = 1.0;
  desired_triad_pose->header.stamp = rclcpp::Time::now();
  desired_triad_pose->header.frame_id = "world";

  double amp = 1.0;
  double phase = 0.0;
  double omega = 1.0;
  double vz = 0.05;
  double dt = 0.01;
  double x, y, z = 0;

  rclcpp::WallRate loop_timer(1 / dt);

  while (rclcpp::ok())
  {
    phase += omega * dt;
    if (phase > 2.0 * M_PI)
    {
      phase -= 2.0 * M_PI;
    }
    x = amp * sin(phase);
    y = amp * cos(phase);
    z += vz * dt;
    desired_triad_pose->pose.position.x = x;
    desired_triad_pose->pose.position.y = y;
    desired_triad_pose->pose.position.z = z;
    desired_triad_pose->pose.orientation = convertPlanarPsi2Quaternion(-phase);
    desired_triad_pose->header.stamp = rclcpp::Time::now();

    pose_publisher->publish(desired_triad_pose);
    loop_timer.sleep();
  }
}
