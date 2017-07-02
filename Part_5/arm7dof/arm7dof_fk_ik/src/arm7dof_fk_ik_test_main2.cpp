#include "arm7dof_fk_ik/arm7dof_kinematics.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#define ROS_INFO printf
#define ROS_WARN printf

Vectorq7x1 g_q_vec;

double q123_err(Eigen::VectorXd q1, Eigen::VectorXd q2)
{
  double esqd = 0.0;
  for (int i = 0; i < 4; i++)
  {
    esqd += (q1(i) - q2(i)) * (q1(i) - q2(i));
  }
  return sqrt(esqd);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::node::Node::make_shared("arm7dof_fk_ik_test");
  auto joint_state_sub = node->create_subscription<sensor_msgs::msg::JointState>(
      "arm7dof/joint_states", 1, [](const sensor_msgs::msg::JointState::SharedPtr js_msg) {
        for (int i = 0; i < 7; i++)
        {
          g_q_vec[i] = js_msg->position[i];
        }
      });

  g_q_vec << 0, 0, 0, 0, 0, 0, 0;
  Vectorq7x1 q_vec_test, q_vec_err;
  double q_fit_best = 100.0;
  double q_fit_err;
  q_vec_test << 0, 0, 0, 0, 0, 0, 0;

  Eigen::Affine3d affine_flange, affine_test, affine_prod;
  Eigen::Matrix3d R_flange;
  Arm7dof_IK_solver arm7dof_ik_solver;
  Arm7dof_fwd_solver arm7dof_fwd_solver;
  Eigen::Vector3d wrist_pt_wrt_frame1, wrist_pt, wrist_pt_test_soln, w_soln_err;
  bool valid_q_elbow = false;
  std::vector<Eigen::VectorXd> q_solns;

  std::vector<Vectorq7x1> q_solns_7dof;
  double q_yaw;
  int n7dof_solns;
  while (rclcpp::ok())
  {
    ROS_INFO("angs: %f, %f, %f, %f, %f, %f, %f", g_q_vec[0], g_q_vec[1], g_q_vec[2], g_q_vec[3], g_q_vec[4], g_q_vec[5],
             g_q_vec[6]);
    q_vec_test = g_q_vec;
    q_yaw = g_q_vec[0];

    affine_flange = arm7dof_fwd_solver.fwd_kin_flange_wrt_base_solve(g_q_vec);
    std::cout << "flange origin: " << affine_flange.translation().transpose() << std::endl;
    std::cout << "R" << std::endl;
    std::cout << affine_flange.linear() << std::endl;
    Eigen::Quaterniond quat(affine_flange.linear());
    ROS_INFO("quat: %f, %f, %f, %f", quat.x(), quat.y(), quat.z(), quat.w());

    int ntrials = 1;
    double dq_yaw = 2.0 * M_PI / 50;
    ROS_WARN("starting q_yaw sampling, %d iters at dq_yaw = %f...", ntrials, dq_yaw);
    std::cout << "------------------------------------------------------------------" << std::endl;
    for (int niters = 0; niters < ntrials; niters++)
    {
      q_solns_7dof.clear();

      arm7dof_ik_solver.ik_solns_sampled_qs0(affine_flange, q_solns_7dof);

      /*
      for (double q_yaw_samp = 0.0; q_yaw_samp < 2.0 * M_PI; q_yaw_samp += dq_yaw) {

          n7dof_solns= arm7dof_ik_solver.ik_solve_given_qs0(affine_flange, q_yaw_samp, q_solns_7dof);
          std::cout << "num solns found = " << n7dof_solns << " at q_yaw = " << q_yaw_samp << std::endl;
      }
      */
      int nsolns_all_qyaw = q_solns_7dof.size();
      ROS_INFO("num solns over q0 samples = %d", nsolns_all_qyaw);
    }
    ROS_WARN("finished iters");
    rclcpp::spin_some(node);
    rclcpp::WallRate(1.0).sleep();
  }
}
