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

Eigen::VectorXd g_q_des;
Eigen::VectorXd g_q_vec_actual, g_qdot_vec_actual;
Vectorq7x1 g_q_actual_7x1;

Eigen::VectorXd J_transpose_f;
Eigen::Vector3d g_f_sensor;
double g_force_z = 0.0;
Eigen::VectorXd g_wrench;

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
    q_dot_cmd_vec[i] = std::min(q_dot_cmd_vec[i], g_qdot_max_vec[i]);
    q_dot_cmd_vec[i] = std::max(q_dot_cmd_vec[i], -g_qdot_max_vec[i]);
  }
}

void jointStatesCb(const sensor_msgs::msg::JointState::SharedPtr js_msg)
{
  for (int i = 0; i < 7; i++)
  {
    g_q_vec_actual[i] = js_msg->position[i + 1];
    g_qdot_vec_actual[i] = js_msg->velocity[i + 1];
    g_q_actual_7x1[i] = g_q_vec_actual[i];
  }
}

void ft_sensor_CB(const geometry_msgs::msg::WrenchStamped::SharedPtr ft)
{
  g_force_z = ft->wrench.force.z;

  g_f_sensor[0] = ft->wrench.force.x;
  g_f_sensor[1] = ft->wrench.force.y;
  g_f_sensor[2] = ft->wrench.force.z;
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
  auto node = rclcpp::node::Node::make_shared("NAC_control_loop");

  Arm7dof_IK_solver arm7dof_ik_solver;
  Arm7dof_fwd_solver arm7dof_fwd_solver;

  Eigen::VectorXd q_ddot_vec, q_dot_cmd_vec;
  Eigen::VectorXd q_vec_err = Eigen::VectorXd::Zero(7, 1);
  Eigen::MatrixXd Kq_on_H = Eigen::MatrixXd::Zero(7, 7);
  Eigen::MatrixXd H_inv = Eigen::MatrixXd::Zero(7, 7);
  Eigen::MatrixXd B_on_H = Eigen::MatrixXd::Identity(7, 7);
  Eigen::MatrixXd B_jnts = Eigen::MatrixXd::Identity(7, 7);
  Eigen::MatrixXd I7x7 = Eigen::MatrixXd::Identity(7, 7);
  Eigen::VectorXd wrench_6x1 = Eigen::VectorXd::Zero(6, 1);
  Eigen::MatrixXd Jacobian;
  Eigen::MatrixXd J_dx_dq = Eigen::MatrixXd::Zero(3, 7);
  Eigen::Vector3d endpt_wrt_base, endpt_attractor, f_attractor, f_net, v_cartesian;
  Eigen::Affine3d affine_flange;
  Eigen::Matrix3d K_cartesian = Eigen::Matrix3d::Identity(3, 3);
  Eigen::Matrix3d B_cartesian = Eigen::Matrix3d::Identity(3, 3);

  Eigen::Matrix3d R_des = Eigen::Matrix3d::Identity(3, 3);

  std::cout << "set Kq_on_H vals..." << std::endl;
  Kq_on_H(0, 0) = 1.0;
  std::cout << "more vals..." << std::endl;

  H_inv(0, 0) = 1 / 20.0;
  H_inv(1, 1) = 1 / 20.0;
  H_inv(2, 2) = 1 / 10.0;
  H_inv(3, 3) = 1 / 10.0;
  H_inv(4, 4) = 1 / 3.0;
  B_jnts = 10 * B_jnts;
  B_jnts(1, 1) = 100;
  B_jnts(2, 2) = 100;
  /*
  Kq_on_H(1,1) = 1.0;
  Kq_on_H(2,2) = 1.0;
  Kq_on_H(3,3) = 1.0;
  Kq_on_H(4,4) = 1.0;
  Kq_on_H(5,5) = 1.0;
  Kq_on_H(6,6) = 1.0;*/
  Kq_on_H = 1.0 * I7x7;

  B_on_H = 1.0 * I7x7;

  /*
  cout<<"g_qdot_max_vec..."<<endl;
  g_qdot_max_vec= q_vec_err;
  cout<<"assign vel limits..."<<endl;
  g_qdot_max_vec[0] = q0_dot_max;
  g_qdot_max_vec[1] = q1_dot_max;
  g_qdot_max_vec[2] = q2_dot_max;
  g_qdot_max_vec[3] = q3_dot_max;
  g_qdot_max_vec[4] = q4_dot_max;
  g_qdot_max_vec[5] = q5_dot_max;
  g_qdot_max_vec[6] = q6_dot_max;
  cout<<"so far, so good..."<<endl;
  */
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

  endpt_attractor << 0, 0, 1.5;
  g_q_vec_actual = g_q_des;
  g_qdot_vec_actual = q_vec_err;

  K_cartesian(0, 0) = 100.0;
  K_cartesian(1, 1) = 100.0;
  K_cartesian(2, 2) = 10.0;
  B_cartesian = 10 * B_cartesian;
  const double K_vel_p = 20.0;

  q_ddot_vec = q_vec_err;
  q_dot_cmd_vec = q_vec_err;
  J_transpose_f = q_vec_err;
  q_vec_err = g_q_des - g_q_vec_actual;

  g_wrench = wrench_6x1;

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
  auto ft_sensor_sub = node->create_subscription<geometry_msgs::msg::WrenchStamped>("ft_sensor_topic", ft_sensor_CB,
                                                                                    rmw_qos_profile_default);

  std::cout << "starting loop..." << std::endl;
  bool is_singular = false;
  std::vector<Vectorq7x1> q_solns;
  Vectorq7x1 q_soln, q_vec_err_wrist;
  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);

    Jacobian = arm7dof_fwd_solver.Jacobian(g_q_vec_actual);
    J_dx_dq = Jacobian.block<3, 7>(0, 0);
    affine_flange = arm7dof_fwd_solver.fwd_kin_flange_wrt_base_solve(g_q_vec_actual);

    endpt_wrt_base = affine_flange.translation();
    std::cout << "endpt_wrt_base: " << endpt_wrt_base.transpose() << std::endl;
    std::cout << "g_f_sensor: " << g_f_sensor.transpose() << std::endl;
    v_cartesian = J_dx_dq * g_qdot_vec_actual;
    f_attractor = K_cartesian * (endpt_attractor - endpt_wrt_base) - B_cartesian * v_cartesian;
    std::cout << "f_attractor: " << f_attractor.transpose() << std::endl;
    f_net = f_attractor + g_f_sensor;
    std::cout << "f_net: " << f_net.transpose() << std::endl;
    J_transpose_f = J_dx_dq.transpose() * f_net;

    q_ddot_vec = H_inv * (J_transpose_f - B_on_H * g_qdot_vec_actual);

    q_dot_cmd_vec += q_ddot_vec * dt;

    is_singular = arm7dof_ik_solver.solve_spherical_wrist(g_q_actual_7x1, R_des, q_solns);
    std::cout << "there are " << q_solns.size() << " wrist solns" << std::endl;
    if (q_solns.size() > 0)
    {
      q_vec_err_wrist = q_solns[0] - g_q_actual_7x1;
      if (q_solns.size() > 1)
      {
        double q_err1 = q_vec_err_wrist.norm();
        double q_err2 = (q_solns[1] - g_q_actual_7x1).norm();
        if (q_err2 < q_err1)
        {
          q_vec_err_wrist = q_solns[1] - g_q_actual_7x1;
        }
      }
      std::cout << "wrist ang err: " << q_vec_err_wrist.transpose() << std::endl;
      for (int i = 4; i < 7; i++)
      {
        q_dot_cmd_vec[i] = K_vel_p * q_vec_err_wrist[i];
      }
    }

    sat_qdot(q_dot_cmd_vec);
    q_dot_cmd_vec += q_ddot_vec * dt;
    sat_qdot(q_dot_cmd_vec);
    send_qdot_cmds(q_dot_cmd_vec);

    rclcpp::spin_some(node);
    naptime.sleep();
  }
}
