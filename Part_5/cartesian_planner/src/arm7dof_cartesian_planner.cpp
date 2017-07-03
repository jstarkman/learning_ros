

#include <cartesian_planner/arm7dof_cartesian_planner.h>

CartTrajPlanner::CartTrajPlanner()

{
  ROS_INFO("in constructor of CartTrajPlanner...");

  b_des_ << 0, 0, -1;
  n_des_ << 1, 0, 0;
  t_des_ = b_des_.cross(n_des_);

  R_gripper_down_.col(0) = n_des_;
  R_gripper_down_.col(1) = t_des_;
  R_gripper_down_.col(2) = b_des_;

  b_des_up_ << 0, 0, 1;
  n_des_up_ << 1, 0, 0;
  t_des_up_ = b_des_up_.cross(n_des_up_);

  R_gripper_up_.col(0) = n_des_up_;
  R_gripper_up_.col(1) = t_des_up_;
  R_gripper_up_.col(2) = b_des_up_;

  tool_n_des_horiz_ << 1, 0, 0;
  tool_b_des_horiz_ << 0, 1, 0;
  tool_t_des_horiz_ = tool_b_des_horiz_.cross(tool_n_des_horiz_);
  R_gripper_horiz_.col(0) = tool_n_des_horiz_;
  R_gripper_horiz_.col(1) = tool_t_des_horiz_;
  R_gripper_horiz_.col(2) = tool_b_des_horiz_;
}

bool CartTrajPlanner::cartesian_path_planner(Eigen::Affine3d a_flange_start, Eigen::Affine3d a_flange_end,
                                             std::vector<Eigen::VectorXd> &optimal_path)
{
  std::vector<std::vector<Eigen::VectorXd> > path_options;
  path_options.clear();
  std::vector<Eigen::VectorXd> single_layer_nodes;
  Eigen::VectorXd node;
  Eigen::Affine3d a_flange_des;
  Eigen::Matrix3d R_des = a_flange_end.linear();
  a_flange_des.linear() = R_des;

  int nsolns;
  bool reachable_proposition;
  int nsteps = 0;
  Eigen::Vector3d p_des, dp_vec, del_p, p_start, p_end;
  p_start = a_flange_start.translation();
  p_end = a_flange_end.translation();
  del_p = p_end - p_start;
  double dp_scalar = CARTESIAN_PATH_SAMPLE_SPACING;
  nsteps = round(del_p.norm() / dp_scalar);
  if (nsteps < 1)
    nsteps = 1;
  dp_vec = del_p / nsteps;
  nsteps++;

  std::vector<Vectorq7x1> q_solns;
  p_des = p_start;

  for (int istep = 0; istep < nsteps; istep++)
  {
    a_flange_des.translation() = p_des;
    cout << "trying: " << p_des.transpose() << endl;

    nsolns = arm7dof_IK_solver_.ik_solns_sampled_qs0(a_flange_des, q_solns);
    std::cout << "cartesian step " << istep << " has = " << nsolns << " IK solns" << endl;
    single_layer_nodes.clear();
    if (nsolns > 0)
    {
      single_layer_nodes.resize(nsolns);
      for (int isoln = 0; isoln < nsolns; isoln++)
      {
        node = q_solns[isoln];
        single_layer_nodes[isoln] = node;
      }

      path_options.push_back(single_layer_nodes);
    }
    p_des += dp_vec;
  }

  int nlayers = path_options.size();
  if (nlayers < 1)
  {
    ROS_WARN("no viable options: quitting");
    return false;
  }

  optimal_path.resize(nlayers);
  double trip_cost;

  Eigen::VectorXd weights;
  weights.resize(VECTOR_DIM);
  for (int i = 0; i < VECTOR_DIM; i++)
  {
    weights(i) = 1.0;
  }

  cout << "instantiating a JointSpacePlanner:" << endl;
  {
    JointSpacePlanner jsp(path_options, weights);
    cout << "recovering the solution..." << endl;
    jsp.get_soln(optimal_path);
    trip_cost = jsp.get_trip_cost();
  }

  cout << "resulting solution path: " << endl;
  for (int ilayer = 0; ilayer < nlayers; ilayer++)
  {
    cout << "ilayer: " << ilayer << " node: " << optimal_path[ilayer].transpose() << endl;
  }
  cout << "soln min cost: " << trip_cost << endl;
  return true;
}

void CartTrajPlanner::test_IK_solns(std::vector<Eigen::VectorXd> &q_solns)
{
  cout << "testing IK solns: " << endl;
  int nsolns = q_solns.size();
  Eigen::Affine3d affine_fk;
  Eigen::Vector3d origin_fk;
  for (int isoln = 0; isoln < nsolns; isoln++)
  {
    affine_fk = arm7dof_fwd_solver_.fwd_kin_flange_wrt_base_solve(q_solns[isoln]);
    origin_fk = affine_fk.translation();
    cout << origin_fk.transpose() << endl;
  }
}

void CartTrajPlanner::test_IK_solns(std::vector<Vectorq7x1> &q_solns)
{
  int nsolns = q_solns.size();
  Eigen::Affine3d affine_fk;
  Eigen::Vector3d origin_fk;
  for (int isoln = 0; isoln < nsolns; isoln++)
  {
    affine_fk = arm7dof_fwd_solver_.fwd_kin_flange_wrt_base_solve(q_solns[isoln]);
    origin_fk = affine_fk.translation();
    cout << origin_fk.transpose() << endl;
  }
}

bool CartTrajPlanner::cartesian_path_planner(Vectorq7x1 q_start, Eigen::Affine3d a_tool_end,
                                             std::vector<Eigen::VectorXd> &optimal_path)
{
  std::vector<std::vector<Eigen::VectorXd> > path_options;
  path_options.clear();
  std::vector<Eigen::VectorXd> single_layer_nodes;
  Eigen::VectorXd node;
  Eigen::Affine3d a_tool_des, a_tool_start;
  Eigen::Matrix3d R_des = a_tool_end.linear();

  a_tool_start = arm7dof_fwd_solver_.fwd_kin_flange_wrt_base_solve(q_start);
  cout << "fwd kin from q_start: " << a_tool_start.translation().transpose() << endl;
  cout << "fwd kin from q_start R: " << endl;
  cout << a_tool_start.linear() << endl;

  a_tool_start.linear() = R_des;
  a_tool_des.linear() = R_des;

  int nsolns;
  bool reachable_proposition;
  int nsteps = 0;
  Eigen::Vector3d p_des, dp_vec, del_p, p_start, p_end;
  p_start = a_tool_start.translation();
  p_end = a_tool_end.translation();
  del_p = p_end - p_start;
  cout << "p_start: " << p_start.transpose() << endl;
  cout << "p_end: " << p_end.transpose() << endl;
  cout << "del_p: " << del_p.transpose() << endl;
  double dp_scalar = CARTESIAN_PATH_SAMPLE_SPACING;
  nsteps = round(del_p.norm() / dp_scalar);
  if (nsteps < 1)
    nsteps = 1;
  dp_vec = del_p / nsteps;
  cout << "dp_vec for nsteps = " << nsteps << " is: " << dp_vec.transpose() << endl;
  nsteps++;

  single_layer_nodes.clear();
  node = q_start;
  single_layer_nodes.push_back(node);
  path_options.push_back(single_layer_nodes);

  std::vector<Vectorq7x1> q_solns;
  p_des = p_start;
  int ans;
  for (int istep = 1; istep < nsteps; istep++)
  {
    p_des += dp_vec;
    a_tool_des.translation() = p_des;
    cout << "trying: " << p_des.transpose() << endl;
    nsolns = arm7dof_IK_solver_.ik_solns_sampled_qs0(a_tool_des, q_solns);
    std::cout << "nsolns = " << nsolns << endl;

    test_IK_solns(q_solns);
    cout << "enter 1: ";
    cin >> ans;

    single_layer_nodes.clear();
    if (nsolns > 0)
    {
      single_layer_nodes.resize(nsolns);
      for (int isoln = 0; isoln < nsolns; isoln++)
      {
        node = q_solns[isoln];
        single_layer_nodes[isoln] = node;
      }

      path_options.push_back(single_layer_nodes);
    }
    else
    {
      return false;
    }
  }

  int nlayers = path_options.size();
  if (nlayers < 1)
  {
    ROS_WARN("no viable options: quitting");
    return false;
  }

  optimal_path.resize(nlayers);
  double trip_cost;

  Eigen::VectorXd weights;
  weights.resize(VECTOR_DIM);
  for (int i = 0; i < VECTOR_DIM; i++)
  {
    weights(i) = 1.0;
  }

  cout << "instantiating a JointSpacePlanner:" << endl;
  {
    JointSpacePlanner jsp(path_options, weights);
    cout << "recovering the solution..." << endl;
    jsp.get_soln(optimal_path);
    trip_cost = jsp.get_trip_cost();
  }

  cout << "resulting solution path: " << endl;
  for (int ilayer = 0; ilayer < nlayers; ilayer++)
  {
    cout << "ilayer: " << ilayer << " node: " << optimal_path[ilayer].transpose() << endl;
  }
  test_IK_solns(optimal_path);
  cout << "soln min cost: " << trip_cost << endl;
  return true;
}

bool CartTrajPlanner::cartesian_path_planner_delta_p(Vectorq7x1 q_start, Eigen::Vector3d delta_p,
                                                     std::vector<Eigen::VectorXd> &optimal_path)
{
  std::vector<std::vector<Eigen::VectorXd> > path_options;
  path_options.clear();
  std::vector<Eigen::VectorXd> single_layer_nodes;
  Eigen::VectorXd node;
  Eigen::Affine3d a_tool_des, a_tool_start, a_tool_end;
  Eigen::Vector3d p_des, dp_vec, del_p, p_start, p_end;

  a_tool_start = arm7dof_fwd_solver_.fwd_kin_flange_wrt_base_solve(q_start);
  Eigen::Matrix3d R_des = a_tool_start.linear();
  p_des = a_tool_start.translation();
  cout << "fwd kin from q_start p: " << p_des.transpose() << endl;
  cout << "fwd kin from q_start R: " << endl;
  cout << a_tool_start.linear() << endl;

  a_tool_end.linear() = R_des;
  a_tool_des.linear() = R_des;
  a_tool_end.translation() = p_des + delta_p;

  int nsolns;
  bool reachable_proposition;
  int nsteps = 0;

  p_start = a_tool_start.translation();
  p_end = a_tool_end.translation();
  del_p = p_end - p_start;
  cout << "p_start: " << p_start.transpose() << endl;
  cout << "p_end: " << p_end.transpose() << endl;
  cout << "del_p: " << del_p.transpose() << endl;
  double dp_scalar = 0.05;
  nsteps = round(del_p.norm() / dp_scalar);
  if (nsteps < 1)
    nsteps = 1;
  dp_vec = del_p / nsteps;
  cout << "dp_vec for nsteps = " << nsteps << " is: " << dp_vec.transpose() << endl;
  nsteps++;

  single_layer_nodes.clear();
  node = q_start;
  single_layer_nodes.push_back(node);
  path_options.push_back(single_layer_nodes);

  std::vector<Vectorq7x1> q_solns;
  p_des = p_start;

  for (int istep = 1; istep < nsteps; istep++)
  {
    p_des += dp_vec;
    a_tool_des.translation() = p_des;
    cout << "trying: " << p_des.transpose() << endl;
    nsolns = arm7dof_IK_solver_.ik_solns_sampled_qs0(a_tool_des, q_solns);
    std::cout << "nsolns = " << nsolns << endl;
    single_layer_nodes.clear();
    if (nsolns > 0)
    {
      single_layer_nodes.resize(nsolns);
      for (int isoln = 0; isoln < nsolns; isoln++)
      {
        node = q_solns[isoln];
        single_layer_nodes[isoln] = node;
      }

      path_options.push_back(single_layer_nodes);
    }
    else
    {
      return false;
    }
  }

  int nlayers = path_options.size();
  if (nlayers < 1)
  {
    ROS_WARN("no viable options: quitting");
    return false;
  }

  optimal_path.resize(nlayers);
  double trip_cost;

  Eigen::VectorXd weights;
  weights.resize(VECTOR_DIM);
  for (int i = 0; i < VECTOR_DIM; i++)
  {
    weights(i) = 1.0;
  }

  cout << "instantiating a JointSpacePlanner:" << endl;
  {
    JointSpacePlanner jsp(path_options, weights);
    cout << "recovering the solution..." << endl;
    jsp.get_soln(optimal_path);
    trip_cost = jsp.get_trip_cost();
  }

  cout << "resulting solution path: " << endl;
  for (int ilayer = 0; ilayer < nlayers; ilayer++)
  {
    cout << "ilayer: " << ilayer << " node: " << optimal_path[ilayer].transpose() << endl;
  }
  cout << "soln min cost: " << trip_cost << endl;
  return true;
}

/*

bool CartTrajPlanner::cartesian_path_planner_wrist(Vectorq7x1 q_start,Eigen::Affine3d a_tool_end,
std::vector<Eigen::VectorXd> &optimal_path) {
    std::vector<std::vector<Eigen::VectorXd> > path_options;
    path_options.clear();
    std::vector<Eigen::VectorXd> single_layer_nodes;
    Eigen::VectorXd node;
    Eigen::Affine3d a_tool_des,a_tool_start;
    Eigen::Matrix3d R_des = a_tool_end.linear();
    a_tool_start = arm7dof_fwd_solver_.fwd_kin_flange_wrt_base_solve(q_start);
    cout<<"fwd kin from q_start: "<<a_tool_start.translation().transpose()<<endl;
    cout<<"fwd kin from q_start R: "<<endl;
    cout<<a_tool_start.linear()<<endl;

    a_tool_start.linear() = R_des;
    a_tool_des.linear() = R_des;

    int nsolns;
    bool reachable_proposition;
    int nsteps = 0;
    Eigen::Vector3d p_des,dp_vec,del_p,p_start,p_end;
    p_start = a_tool_start.translation();
    p_end = a_tool_end.translation();
    del_p = p_end-p_start;
    cout<<"p_start: "<<p_start.transpose()<<endl;
    cout<<"p_end: "<<p_end.transpose()<<endl;
    cout<<"del_p: "<<del_p.transpose()<<endl;
    double dp_scalar = CARTESIAN_PATH_SAMPLE_SPACING;
    nsteps = round(del_p.norm()/dp_scalar);
    if (nsteps<1) nsteps=1;
    dp_vec = del_p/nsteps;
    cout<<"dp_vec for nsteps = "<<nsteps<<" is: "<<dp_vec.transpose()<<endl;
    nsteps++;


    single_layer_nodes.clear();
    node = q_start;
    single_layer_nodes.push_back(node);
    path_options.push_back(single_layer_nodes);

    std::vector<Vectorq7x1> q_solns;
    p_des = p_start;

    for (int istep=1;istep<nsteps;istep++)
    {
            p_des += dp_vec;
            a_tool_des.translation() = p_des;
            cout<<"trying: "<<p_des.transpose()<<endl;

            nsolns = arm7dof_IK_solver_.ik_wristpt_solve_approx_wrt_torso(a_tool_des, q_solns);
            std::cout<<"nsolns = "<<nsolns<<endl;
            single_layer_nodes.clear();
            if (nsolns>0) {
                single_layer_nodes.resize(nsolns);
                for (int isoln = 0; isoln < nsolns; isoln++) {

                    node = q_solns[isoln];
                    single_layer_nodes[isoln] = node;
                }

                path_options.push_back(single_layer_nodes);
            }
            else {
                return false;
            }

    }


    int nlayers = path_options.size();
    if (nlayers < 1) {
        ROS_WARN("no viable options: quitting");
        return false;
    }

    optimal_path.resize(nlayers);
    double trip_cost;

    Eigen::VectorXd weights;
    weights.resize(VECTOR_DIM);
    for (int i = 0; i < VECTOR_DIM; i++) {
        weights(i) = 1.0;
    }


    cout << "instantiating a JointSpacePlanner:" << endl;
    {
        JointSpacePlanner jsp(path_options, weights);
        cout << "recovering the solution..." << endl;
        jsp.get_soln(optimal_path);
        trip_cost = jsp.get_trip_cost();
    }


    cout << "resulting solution path: " << endl;
    for (int ilayer = 0; ilayer < nlayers; ilayer++) {
        cout << "ilayer: " << ilayer << " node: " << optimal_path[ilayer].transpose() << endl;
    }
    cout << "soln min cost: " << trip_cost << endl;
    return true;
}
*/

bool CartTrajPlanner::jspace_trivial_path_planner(Vectorq7x1 q_start, Vectorq7x1 q_end,
                                                  std::vector<Eigen::VectorXd> &optimal_path)
{
  Eigen::VectorXd qx_start(7), qx_end(7);

  for (int i = 0; i < 7; i++)
  {
    qx_start[i] = q_start[i];
    qx_end[i] = q_end[i];
  }
  cout << "jspace_trivial_path_planner: " << endl;
  cout << "q_start: " << qx_start.transpose() << endl;
  cout << "q_end: " << qx_end.transpose() << endl;
  optimal_path.clear();
  optimal_path.push_back(qx_start);
  optimal_path.push_back(qx_end);
  return true;
}

Eigen::Affine3d CartTrajPlanner::get_fk_Affine_from_qvec(Vectorq7x1 q_vec)
{
  Eigen::Affine3d Affine_pose;
  Affine_pose = arm7dof_fwd_solver_.fwd_kin_flange_wrt_base_solve(q_vec);
}