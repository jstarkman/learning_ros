

#include <eigen3/Eigen/src/Geometry/Quaternion.h>
#include <sensor_msgs/JointState.h>
#include <ur_fk_ik/ur_kin.h>

Eigen::VectorXd g_q_vec;
using namespace std;
#define VECTOR_DIM 6

vector<int> g_arm_joint_indices;
vector<string> g_ur_jnt_names;

double sgn(double x)
{
  if (x > 0.0)
    return 1.0;
  if (x < 0.0)
    return -1.0;
  return 0.0;
}

void map_arm_joint_indices(vector<string> joint_names)
{
  g_arm_joint_indices.clear();
  int index;
  int n_jnts = VECTOR_DIM;

  std::string j_name;

  for (int j = 0; j < VECTOR_DIM; j++)
  {
    j_name = g_ur_jnt_names[j];
    for (int i = 0; i < n_jnts; i++)
    {
      if (j_name.compare(joint_names[i]) == 0)
      {
        index = i;

        g_arm_joint_indices.push_back(index);
        break;
      }
    }
  }
  cout << "indices of arm joints: " << endl;
  for (int i = 0; i < VECTOR_DIM; i++)
  {
    cout << g_arm_joint_indices[i] << ", ";
  }
  cout << endl;
}

void jointStatesCb(const sensor_msgs::JointState& js_msg)
{
  if (g_arm_joint_indices.size() < 1)
  {
    int njnts = js_msg.position.size();
    ROS_INFO("finding joint mappings for %d jnts", njnts);
    map_arm_joint_indices(js_msg.name);
  }
  for (int i = 0; i < VECTOR_DIM; i++)
  {
    g_q_vec[i] = js_msg.position[g_arm_joint_indices[i]];
  }
  cout << "CB: q_vec_right_arm: " << g_q_vec.transpose() << endl;
}

void set_ur_jnt_names()
{
  g_ur_jnt_names.push_back("shoulder_pan_joint");
  g_ur_jnt_names.push_back("shoulder_lift_joint");
  g_ur_jnt_names.push_back("elbow_joint");
  g_ur_jnt_names.push_back("wrist_1_joint");
  g_ur_jnt_names.push_back("wrist_2_joint");
  g_ur_jnt_names.push_back("wrist_3_joint");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ur10_kinematics_test_main");
  ros::NodeHandle nh;

  double euler_R = 01.2192;
  double euler_P = 0.9412;
  double euler_Y = 0.4226;
  Eigen::VectorXd q_in;
  Eigen::Matrix4d A61;
  q_in.resize(NJNTS);
  g_q_vec.resize(NJNTS);

  q_in << 0.1, -0.2, 0.3, 0.4, 0.5, 0.6;
  g_q_vec = q_in;

  Eigen::Affine3d a_tool;
  a_tool.linear() << 0, 0, -1, 0, 1, 0, 1, 0, 0;
  a_tool.translation() << 0.0, 0.0, 0.0;

  UR10FwdSolver fwd_solver;
  UR10IkSolver ik_solver;
  set_ur_jnt_names();
  ros::Subscriber joint_state_sub = nh.subscribe("/joint_states", 1, jointStatesCb);
  cout << "warming up callbacks..." << endl;
  while (g_arm_joint_indices.size() < 1)
  {
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }
  std::cout << "==== Test for UR10  kinematics solver ====" << std::endl;
  Eigen::VectorXd q_soln;
  q_soln.resize(6);

  while (ros::ok())
  {
    double rval;
    for (int i = 0; i < NJNTS; i++)
    {
      q_in[i] = g_q_vec[i];
    }

    Eigen::Affine3d A_fwd_DH = fwd_solver.fwd_kin_solve(q_in);

    std::cout << "q_in: " << q_in.transpose() << std::endl;
    std::cout << "A rot: " << std::endl;
    std::cout << A_fwd_DH.linear() << std::endl;
    std::cout << "A origin: " << A_fwd_DH.translation().transpose() << std::endl;
    Eigen::Matrix3d R_flange = A_fwd_DH.linear();
    Eigen::Matrix4d A_wrist;
    Eigen::Quaterniond quat(R_flange);
    std::cout << "quat: " << quat.x() << ", " << quat.y() << ", " << quat.z() << ", " << quat.w() << endl;

    Eigen::Matrix3d R_hand;

    A_wrist = fwd_solver.get_wrist_frame();

    std::vector<Eigen::VectorXd> q6dof_solns;
    int nsolns = ik_solver.ik_solve(A_fwd_DH, q6dof_solns);

    nsolns = q6dof_solns.size();
    std::cout << "number of IK solutions: " << nsolns << std::endl;
    double q_err;
    int i_min = -1;
    std::cout << "found " << nsolns << " solutions:" << std::endl;
    for (int i = 0; i < nsolns; i++)
    {
      Eigen::VectorXd q_soln = q6dof_solns[i];

      std::cout << q_soln.transpose() << std::endl;
      q6dof_solns[i] = q_soln;
      q_err = (q_in - q_soln).norm();
      if (q_err < 0.000001)
      {
        i_min = i;
      }
    }
    std::cout << "precise fit for soln " << i_min << std::endl << std::endl;

    for (int i = 0; i < nsolns; i++)
    {
      A_fwd_DH = fwd_solver.fwd_kin_solve(q6dof_solns[i]);

      Eigen::VectorXd q_soln = q6dof_solns[i];
      std::cout << "q: " << q_soln.transpose() << std::endl;
      Eigen::Matrix3d R_flange = A_fwd_DH.linear();
      Eigen::Matrix4d A_wrist;
      Eigen::Quaterniond quat(R_flange);
      std::cout << "   quat: " << quat.x() << ", " << quat.y() << ", " << quat.z() << ", " << quat.w() << endl;

      cout << "   fwd kin flange origin: " << A_fwd_DH.translation().transpose() << endl;
    }

    /*
    double bx_wrt1, by_wrt1, bz_wrt1;
    double s234, c234, s5, c5;
    s5 = sin(g_q_vec[4]);
    c5 = cos(g_q_vec[4]);
    double q234 = g_q_vec[1]+g_q_vec[2]+g_q_vec[3];
    ROS_INFO("true q234 = %f",q234);

    s234 = sin(q234);
    c234 = cos(q234);
    bx_wrt1 = c234*s5;
    by_wrt1 = s234*s5;
    ROS_INFO("true bx_wrt1, by_wrt1 = %f, %f",bx_wrt1,by_wrt1);
    double q234_atan;
    q234_atan = atan2(sgn(s5)*by_wrt1,sgn(s5)*bx_wrt1);
    ROS_INFO("s5 = %f",s5);
    ROS_INFO("q234 from atan2: %f",q234_atan);
    */
    ros::Duration(1.0).sleep();
    ros::spinOnce();
  }
  return 0;
}
