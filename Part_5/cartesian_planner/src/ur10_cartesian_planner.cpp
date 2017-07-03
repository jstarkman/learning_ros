

#include <cartesian_planner/ur10_cartesian_planner.h>

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

  jspace_planner_weights_.resize(NJNTS);
  jspace_planner_weights_[0] = 5;

  jspace_planner_weights_[1] = 5;
  jspace_planner_weights_[2] = 3;

  jspace_planner_weights_[3] = 0.5;
  jspace_planner_weights_[4] = 0.2;
  jspace_planner_weights_[5] = 0.2;
}

bool CartTrajPlanner::cartesian_path_planner(Eigen::VectorXd q_start, Eigen::Affine3d a_flange_end,
                                             std::vector<Eigen::VectorXd> &optimal_path)
{
  return cartesian_path_planner(q_start, a_flange_end, optimal_path, CARTESIAN_PATH_SAMPLE_SPACING);
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

  cartesian_affine_samples_.clear();
  a_flange_des = a_flange_end;
  a_flange_start.linear() = R_des;
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

  std::vector<Eigen::VectorXd> q_solns;
  p_des = p_start;
  cartesian_affine_samples_.push_back(a_flange_start);
  for (int istep = 0; istep < nsteps; istep++)
  {
    a_flange_des.translation() = p_des;
    cartesian_affine_samples_.push_back(a_flange_des);
    cout << "trying: " << p_des.transpose() << endl;

    nsolns = ur10IkSolver_.ik_solve(a_flange_des, q_solns);
    std::cout << "cartesian step " << istep << " has = " << nsolns << " IK solns" << endl;
    single_layer_nodes.clear();
    if (nsolns > 0)
    {
      single_layer_nodes.resize(nsolns);
      for (int isoln = 0; isoln < nsolns; isoln++)
      {
        single_layer_nodes[isoln] = q_solns[isoln];
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
  weights.resize(NJNTS);
  for (int i = 0; i < NJNTS; i++)
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
    affine_fk = ur10FwdSolver_.fwd_kin_solve(q_solns[isoln]);
    origin_fk = affine_fk.translation();
    cout << origin_fk.transpose() << endl;
  }
}

bool CartTrajPlanner::cartesian_path_planner(Eigen::VectorXd q_start, Eigen::Affine3d a_tool_end,
                                             std::vector<Eigen::VectorXd> &optimal_path, double dp_scalar)
{
  std::vector<std::vector<Eigen::VectorXd> > path_options;
  path_options.clear();
  std::vector<Eigen::VectorXd> single_layer_nodes;
  Eigen::VectorXd node;
  Eigen::Affine3d a_tool_des, a_tool_start;
  Eigen::Matrix3d R_des = a_tool_end.linear();

  a_tool_start = ur10FwdSolver_.fwd_kin_solve(q_start);
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

  std::vector<Eigen::VectorXd> q_solns;
  p_des = p_start;
  int ans;
  for (int istep = 1; istep < nsteps; istep++)
  {
    p_des += dp_vec;
    a_tool_des.translation() = p_des;
    cout << "trying: " << p_des.transpose() << endl;
    nsolns = ur10IkSolver_.ik_solve(a_tool_des, q_solns);
    std::cout << "nsolns = " << nsolns << endl;

    single_layer_nodes.clear();
    if (nsolns > 0)
    {
      single_layer_nodes.resize(nsolns);
      for (int isoln = 0; isoln < nsolns; isoln++)
      {
        single_layer_nodes[isoln] = q_solns[isoln];
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
  weights.resize(NJNTS);
  for (int i = 0; i < NJNTS; i++)
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

bool CartTrajPlanner::cartesian_path_planner_delta_p(Eigen::VectorXd q_start, Eigen::Vector3d delta_p,
                                                     std::vector<Eigen::VectorXd> &optimal_path)
{
  std::vector<std::vector<Eigen::VectorXd> > path_options;
  path_options.clear();
  std::vector<Eigen::VectorXd> single_layer_nodes;
  Eigen::VectorXd node;
  Eigen::Affine3d a_tool_des, a_tool_start, a_tool_end;
  Eigen::Vector3d p_des, dp_vec, del_p, p_start, p_end;

  a_tool_start = ur10FwdSolver_.fwd_kin_solve(q_start);
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

  single_layer_nodes.push_back(q_start);
  path_options.push_back(single_layer_nodes);

  std::vector<Eigen::VectorXd> q_solns;
  p_des = p_start;

  for (int istep = 1; istep < nsteps; istep++)
  {
    p_des += dp_vec;
    a_tool_des.translation() = p_des;
    cout << "trying: " << p_des.transpose() << endl;
    nsolns = ur10IkSolver_.ik_solve(a_tool_des, q_solns);
    std::cout << "nsolns = " << nsolns << endl;
    single_layer_nodes.clear();
    if (nsolns > 0)
    {
      single_layer_nodes.resize(nsolns);
      for (int isoln = 0; isoln < nsolns; isoln++)
      {
        single_layer_nodes[isoln] = q_solns[isoln];
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
  weights.resize(NJNTS);
  for (int i = 0; i < NJNTS; i++)
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

bool CartTrajPlanner::jspace_trivial_path_planner(Eigen::VectorXd q_start, Eigen::VectorXd q_end,
                                                  std::vector<Eigen::VectorXd> &optimal_path)
{
  Eigen::VectorXd qx_start(NJNTS), qx_end(NJNTS);

  cout << "jspace_trivial_path_planner: " << endl;
  cout << "q_start: " << q_start.transpose() << endl;
  cout << "q_end: " << q_end.transpose() << endl;
  optimal_path.clear();
  optimal_path.push_back(q_start);
  optimal_path.push_back(q_end);
  return true;
}

bool CartTrajPlanner::fine_cartesian_path_planner(Eigen::VectorXd q_start, Eigen::Affine3d a_flange_end,
                                                  std::vector<Eigen::VectorXd> &optimal_path)
{
  bool valid = cartesian_path_planner(q_start, a_flange_end, optimal_path, CARTESIAN_PATH_FINE_SAMPLE_SPACING);
  if (!valid)
  {
    return false;
  }
  return valid;
}

bool CartTrajPlanner::jspace_path_planner_to_affine_goal(Eigen::VectorXd q_start, Eigen::Affine3d a_flange_end,
                                                         std::vector<Eigen::VectorXd> &optimal_path)
{
  Eigen::VectorXd qx_end(NJNTS);
  std::vector<Eigen::VectorXd> q_solns;
  std::vector<std::vector<Eigen::VectorXd> > path_options;
  path_options.clear();
  std::vector<Eigen::VectorXd> single_layer_nodes;

  single_layer_nodes.clear();
  single_layer_nodes.push_back(q_start);

  cout << "jspace planner to Cartesian goal: " << endl;

  int nsolns = ur10IkSolver_.ik_solve(a_flange_end, q_solns);
  std::cout << "nsolns at goal pose = " << nsolns << endl;
  single_layer_nodes.clear();
  if (nsolns < 1)
    return false;

  for (int isoln = 0; isoln < nsolns; isoln++)
  {
    single_layer_nodes.push_back(q_solns[isoln]);
  }
  nsolns = single_layer_nodes.size();
  ROS_INFO("found %d goal IK solns", nsolns);
  if (nsolns < 1)
  {
    return false;
  }

  optimal_path.clear();
  optimal_path.push_back(q_start);

  Eigen::VectorXd dq_move(NJNTS), q_modified_start(NJNTS);
  q_modified_start = q_start;

  double penalty_best = 1000000;
  double penalty;

  cout << "jspace_planner_weights_: " << jspace_planner_weights_.transpose() << endl;
  qx_end = single_layer_nodes[0];
  cout << "qx_end: " << qx_end.transpose() << endl;
  for (int i = 0; i < nsolns; i++)
  {
    dq_move = q_modified_start - single_layer_nodes[i];
    cout << "dq_move: " << dq_move.transpose() << endl;
    penalty = 0.0;
    for (int j = 0; j < NJNTS; j++)
    {
      penalty += jspace_planner_weights_[j] * fabs(dq_move[j]);
    }
    ROS_INFO("soln %d has penalty = %f", i, penalty);
    if (penalty < penalty_best)
    {
      penalty_best = penalty;
      qx_end = single_layer_nodes[i];
    }
  }

  optimal_path.push_back(qx_end);
  return true;
}
