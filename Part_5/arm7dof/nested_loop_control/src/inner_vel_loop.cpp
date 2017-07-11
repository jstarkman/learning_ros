#define _USE_MATH_DEFINES
#include <math.h>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <iostream>
#include <string>
#include <vector>
#include "arm7dof_fk_ik/arm7dof_kinematics.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

const double q0_dot_max = 0.17;
const double q1_dot_max = 0.17;
const double q2_dot_max = 0.3;
const double q3_dot_max = 0.3;
const double q4_dot_max = 0.4;
const double q5_dot_max = 0.4;
const double q6_dot_max = 0.4;

const double qdot_max_vec[] = {
  q0_dot_max, q1_dot_max, q2_dot_max, q3_dot_max, q4_dot_max, q5_dot_max, q6_dot_max
};  // values per URDF

Eigen::VectorXd g_q_des;
Eigen::VectorXd g_q_vec_actual, g_qdot_vec_actual;
Vectorq7x1 g_q_actual_7x1;

rclcpp::publisher::Publisher<std_msgs::msg::Float64>::SharedPtr g_j0_pub;
rclcpp::publisher::Publisher<std_msgs::msg::Float64>::SharedPtr g_j1_pub;
rclcpp::publisher::Publisher<std_msgs::msg::Float64>::SharedPtr g_j2_pub;
rclcpp::publisher::Publisher<std_msgs::msg::Float64>::SharedPtr g_j3_pub;
rclcpp::publisher::Publisher<std_msgs::msg::Float64>::SharedPtr g_j4_pub;
rclcpp::publisher::Publisher<std_msgs::msg::Float64>::SharedPtr g_j5_pub;
rclcpp::publisher::Publisher<std_msgs::msg::Float64>::SharedPtr g_j6_pub;

void qDesCB(const std_msgs::msg::Float64MultiArray::SharedPtr q_des_msg)
{
  for (int i = 0; i < 7; i++)
  {
    g_q_des[i] = q_des_msg->data[i];
  }
  std::cout << "qDesCB: " << g_q_des.transpose() << std::endl;
}

void sat_qdot(Eigen::VectorXd& q_dot_cmd_vec)
{
  for (int i = 0; i < 7; i++)
  {
    q_dot_cmd_vec[i] = std::min(q_dot_cmd_vec[i], qdot_max_vec[i]);
    q_dot_cmd_vec[i] = std::max(q_dot_cmd_vec[i], -qdot_max_vec[i]);
  }
}

void jointStatesCb(const sensor_msgs::msg::JointState::SharedPtr js_msg)
{
  for (int i = 0; i < 7; i++)
  {
    g_q_vec_actual[i] = js_msg->position[i + 1];
    g_qdot_vec_actual[i] = js_msg->velocity[i + 1];
  }
}

void send_qdot_cmds(Eigen::VectorXd qdot_cmd_vec)
{
  std_msgs::msg::Float64 cmd_msg;
  cmd_msg.data = qdot_cmd_vec[0];
  g_j0_pub->publish(cmd_msg);

  cmd_msg.data = qdot_cmd_vec[1];
  g_j1_pub->publish(cmd_msg);

  cmd_msg.data = qdot_cmd_vec[2];
  g_j2_pub->publish(cmd_msg);

  cmd_msg.data = qdot_cmd_vec[3];
  g_j3_pub->publish(cmd_msg);

  cmd_msg.data = qdot_cmd_vec[4];
  g_j4_pub->publish(cmd_msg);

  cmd_msg.data = qdot_cmd_vec[5];
  g_j5_pub->publish(cmd_msg);

  cmd_msg.data = qdot_cmd_vec[6];
  g_j6_pub->publish(cmd_msg);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::node::Node::make_shared("inner_vel_loop");

  Eigen::VectorXd q_ddot_vec, q_dot_cmd_vec;
  Eigen::VectorXd q_vec_err = Eigen::VectorXd::Zero(7, 1);
  Eigen::MatrixXd Kq_on_H = Eigen::MatrixXd::Zero(7, 7);
  Eigen::MatrixXd B_on_H = Eigen::MatrixXd::Identity(7, 7);
  Eigen::MatrixXd I7x7 = Eigen::MatrixXd::Identity(7, 7);

  std::cout << "set Kq_on_H vals..." << std::endl;
  /*
Kq_on_H(1,1) = 1.0;
Kq_on_H(2,2) = 1.0;
Kq_on_H(3,3) = 1.0;
Kq_on_H(4,4) = 1.0;
Kq_on_H(5,5) = 1.0;
Kq_on_H(6,6) = 1.0;*/
  Kq_on_H = 1.0 * I7x7;

  B_on_H = 1.0 * I7x7;

  double dt = 0.001;
  rclcpp::WallRate naptime(1 / dt);

  g_q_des = q_vec_err;
  for (int i = 0; i < 7; i++)
  {
    g_q_des[i] = 0.0;
  }

  g_q_des[1] = 1.0;
  g_q_des[3] = -2.0;
  g_q_des[5] = 1.0;
  g_q_vec_actual = g_q_des;
  g_qdot_vec_actual = q_vec_err;

  q_ddot_vec = q_vec_err;
  q_dot_cmd_vec = q_vec_err;
  q_vec_err = g_q_des - g_q_vec_actual;

  g_j0_pub = node->create_publisher<std_msgs::msg::Float64>("/arm7dof/joint0_velocity_controller/command",
                                                            rmw_qos_profile_default);
  g_j1_pub = node->create_publisher<std_msgs::msg::Float64>("/arm7dof/joint1_velocity_controller/command",
                                                            rmw_qos_profile_default);
  g_j2_pub = node->create_publisher<std_msgs::msg::Float64>("/arm7dof/joint2_velocity_controller/command",
                                                            rmw_qos_profile_default);
  g_j3_pub = node->create_publisher<std_msgs::msg::Float64>("/arm7dof/joint3_velocity_controller/command",
                                                            rmw_qos_profile_default);
  g_j4_pub = node->create_publisher<std_msgs::msg::Float64>("/arm7dof/joint4_velocity_controller/command",
                                                            rmw_qos_profile_default);
  g_j5_pub = node->create_publisher<std_msgs::msg::Float64>("/arm7dof/joint5_velocity_controller/command",
                                                            rmw_qos_profile_default);
  g_j6_pub = node->create_publisher<std_msgs::msg::Float64>("/arm7dof/joint6_velocity_controller/command",
                                                            rmw_qos_profile_default);

  auto joint_state_sub = node->create_subscription<sensor_msgs::msg::JointState>("arm7dof/joint_states", jointStatesCb,
                                                                                 rmw_qos_profile_default);
  auto q_des_sub = node->create_subscription<std_msgs::msg::Float64MultiArray>("qdes_attractor_vec", qDesCB,
                                                                               rmw_qos_profile_default);

  std::cout << "starting position control loop using inner velocity control loop." << std::endl;
  std::cout << "Listening for desired joint values on topic qdes_attractor_vec." << std::endl;

  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);

    q_vec_err = g_q_des - g_q_vec_actual;

    q_ddot_vec = Kq_on_H * q_vec_err - B_on_H * g_qdot_vec_actual;
    q_dot_cmd_vec += q_ddot_vec * dt;
    sat_qdot(q_dot_cmd_vec);
    q_dot_cmd_vec += q_ddot_vec * dt;
    sat_qdot(q_dot_cmd_vec);
    send_qdot_cmds(q_dot_cmd_vec);

    rclcpp::spin_some(node);
    naptime.sleep();
  }
}
