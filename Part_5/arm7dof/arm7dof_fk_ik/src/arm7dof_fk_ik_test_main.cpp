#include "arm7dof_fk_ik/arm7dof_kinematics.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

Vectorq7x1 g_q_vec;

std::vector<int> g_joint_indices;
std::string g_arm7dof_jnt_names[] = { "joint0", "joint1", "joint2", "joint3", "joint4", "joint5", "joint6" };
const int arm7dof_NJNTS = 7;

void map_arm_joint_indices(std::vector<std::string> joint_names, std::vector<int> &joint_indices)
{
  joint_indices.clear();

  int index;
  int n_jnts = joint_names.size();
  std::cout << "num jnt names = " << n_jnts << std::endl;
  std::string j_name;

  for (int j = 0; j < arm7dof_NJNTS; j++)
  {
    j_name = g_arm7dof_jnt_names[j];
    for (int i = 0; i < n_jnts; i++)
    {
      if (j_name.compare(joint_names[i]) == 0)
      {
        index = i;
        joint_indices.push_back(index);
        break;
      }
    }
  }
  /*
 std::cout<<"indices of arm joints: "<<std::endl;
 for (int i=0;i<arm7dof_NJNTS;i++) {
     std::cout<<joint_indices_[i]<<", ";
 }
   * */
}

double q123_err(Eigen::VectorXd q1, Eigen::VectorXd q2)
{
  double esqd = 0.0;
  for (int i = 0; i < 4; i++)
  {
    esqd += (q1(i) - q2(i)) * (q1(i) - q2(i));
  }
  return sqrt(esqd);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::node::Node::make_shared("arm7dof_fk_ik_test");
  auto joint_state_sub = node->create_subscription<sensor_msgs::msg::JointState>(
      "arm7dof/joint_states", 1, [](const sensor_msgs::msg::JointState::SharedPtr js_msg) {
        std::vector<int> joint_indices;
        map_arm_joint_indices(js_msg->name, joint_indices);
        for (int i = 0; i < arm7dof_NJNTS; i++)
        {
          g_q_vec[i] = js_msg->position[joint_indices[i]];
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
  // bool valid_q_elbow = false;
  std::vector<Eigen::VectorXd> q_solns;

  std::vector<Vectorq7x1> q_solns_7dof;
  double q_yaw;
  // int n7dof_solns;
  // int ans;
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
    affine_test = arm7dof_fwd_solver.get_frame0();
    std::cout << "O0: " << affine_test.translation().transpose() << std::endl;
    affine_test = arm7dof_fwd_solver.get_frame1();
    std::cout << "O1: " << affine_test.translation().transpose() << std::endl;
    affine_test = arm7dof_fwd_solver.get_frame2();
    std::cout << "O2: " << affine_test.translation().transpose() << std::endl;
    affine_test = arm7dof_fwd_solver.get_frame3();
    std::cout << "O3: " << affine_test.translation().transpose() << std::endl;
    affine_test = arm7dof_fwd_solver.get_frame4();
    std::cout << "O4: " << affine_test.translation().transpose() << std::endl;
    affine_test = arm7dof_fwd_solver.get_frame5();
    std::cout << "O5: " << affine_test.translation().transpose() << std::endl;
    affine_test = arm7dof_fwd_solver.get_frame6();
    std::cout << "O6: " << affine_test.translation().transpose() << std::endl;

    // JAS seems unnecessary, but might be important
    // std::cout << "enter 1: ";
    // std::cin >> ans;

    std::cout << "calling ik_solve_given_qs0" << std::endl;
    arm7dof_ik_solver.ik_solve_given_qs0(affine_flange, q_yaw, q_solns_7dof);
    int n7dof_solns = q_solns_7dof.size();
    std::cout << "num solns found = " << n7dof_solns << std::endl;
    q_fit_best = 100.0;
    for (int isoln = 0; isoln < n7dof_solns; isoln++)
    {
      q_vec_test = q_solns_7dof[isoln];
      std::cout << "q_test: " << q_vec_test.transpose() << std::endl;
      q_fit_err = (q_vec_test - g_q_vec).norm();
      std::cout << "q_fit_err = " << q_fit_err << std::endl;
      if (q_fit_err < q_fit_best)
        q_fit_best = q_fit_err;

      affine_test = arm7dof_fwd_solver.fwd_kin_flange_wrt_base_solve(q_vec_test);
      std::cout << "flange origin: " << affine_test.translation().transpose() << std::endl;
      affine_prod = affine_test.inverse() * affine_flange;
      std::cout << "R_prod: " << std::endl;
      std::cout << affine_prod.linear() << std::endl;
      std::cout << "origin_err.norm(): " << affine_prod.translation().norm() << std::endl;
    }
    std::cout << "q_fit_best = " << q_fit_best << std::endl;
    rclcpp::spin_some(node);
    rclcpp::WallRate(1.0).sleep();
  }
}
