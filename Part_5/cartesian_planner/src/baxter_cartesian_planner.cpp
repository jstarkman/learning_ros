

#include <cartesian_planner/baxter_cartesian_planner.h>

CartTrajPlanner::CartTrajPlanner()

{
  ROS_INFO("in constructor of CartTrajPlanner...");

  b_des_ << 0, 0, -1;
  n_des_ << 1, 0, 0;
  t_des_ = b_des_.cross(n_des_);

  R_gripper_down_.col(0) = n_des_;
  R_gripper_down_.col(1) = t_des_;
  R_gripper_down_.col(2) = b_des_;

  tool_n_des_horiz_ << 1, 0, 0;
  tool_b_des_horiz_ << 0, 1, 0;
  tool_t_des_horiz_ = tool_b_des_horiz_.cross(tool_n_des_horiz_);
  R_gripper_horiz_.col(0) = tool_n_des_horiz_;
  R_gripper_horiz_.col(1) = tool_t_des_horiz_;
  R_gripper_horiz_.col(2) = tool_b_des_horiz_;

  jspace_planner_weights_.resize(7);
  jspace_planner_weights_[0] = 2;

  jspace_planner_weights_[1] = 10;

  jspace_planner_weights_[2] = 3;

  jspace_planner_weights_[3] = 0.5;
  jspace_planner_weights_[4] = 0.2;
  jspace_planner_weights_[5] = 0.2;
  jspace_planner_weights_[6] = 0.2;
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
    cartesian_affine_samples_.push_back(a_flange_des);

    cout << "trying: " << p_des.transpose() << endl;
    nsolns = baxter_IK_solver_.ik_solve_approx_wrt_torso(a_flange_des, q_solns);
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

  cout << "instantiating a JointSpacePlanner:" << endl;
  {
    JointSpacePlanner jsp(path_options, jspace_planner_weights_);
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

bool CartTrajPlanner::cartesian_path_planner(Vectorq7x1 q_start, Eigen::Affine3d a_flange_end,
                                             std::vector<Eigen::VectorXd> &optimal_path,
                                             double dp_scalar /* = CARTESIAN_PATH_SAMPLE_SPACING*/)
{
  std::vector<std::vector<Eigen::VectorXd> > path_options;
  path_options.clear();
  std::vector<Eigen::VectorXd> single_layer_nodes;
  Eigen::VectorXd node;
  Eigen::Affine3d a_flange_des, a_flange_start;
  Eigen::Matrix3d R_des = a_flange_end.linear();
  a_flange_start = baxter_fwd_solver_.fwd_kin_flange_wrt_torso_solve(q_start);
  cout << "fwd kin from q_start: " << a_flange_start.translation().transpose() << endl;
  cout << "fwd kin from q_start R: " << endl;
  cout << a_flange_start.linear() << endl;

  cartesian_affine_samples_.clear();
  a_flange_des = a_flange_end;
  a_flange_start.linear() = R_des;
  a_flange_des.linear() = R_des;

  int nsolns;
  bool reachable_proposition;
  int nsteps = 0;
  Eigen::Vector3d p_des, dp_vec, del_p, p_start, p_end;
  p_start = a_flange_start.translation();
  p_end = a_flange_des.translation();
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

  std::vector<Vectorq7x1> q_solns;

  p_des = p_start;
  cartesian_affine_samples_.push_back(a_flange_start);

  for (int istep = 1; istep < nsteps; istep++)
  {
    p_des += dp_vec;
    a_flange_des.translation() = p_des;
    cartesian_affine_samples_.push_back(a_flange_des);
    cout << "trying: " << p_des.transpose() << endl;
    nsolns = baxter_IK_solver_.ik_solve_approx_wrt_torso(a_flange_des, q_solns);
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

  cout << "instantiating a JointSpacePlanner:" << endl;
  {
    JointSpacePlanner jsp(path_options, jspace_planner_weights_);
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

bool CartTrajPlanner::fine_cartesian_path_planner(Vectorq7x1 q_start, Eigen::Affine3d a_flange_end,
                                                  std::vector<Eigen::VectorXd> &optimal_path)
{
  bool valid = cartesian_path_planner(q_start, a_flange_end, optimal_path, CARTESIAN_PATH_FINE_SAMPLE_SPACING);
  if (!valid)
  {
    return false;
  }

  refine_cartesian_path_plan(optimal_path);
  return valid;
}

bool CartTrajPlanner::cartesian_path_planner_delta_p(Vectorq7x1 q_start, Eigen::Vector3d delta_p,
                                                     std::vector<Eigen::VectorXd> &optimal_path)
{
  std::vector<std::vector<Eigen::VectorXd> > path_options;

  cartesian_affine_samples_.clear();
  path_options.clear();
  std::vector<Eigen::VectorXd> single_layer_nodes;
  Eigen::VectorXd node;
  Eigen::Affine3d a_flange_des, a_flange_start, a_flange_end;
  Eigen::Vector3d p_des, dp_vec, del_p, p_start, p_end;

  a_flange_start = baxter_fwd_solver_.fwd_kin_flange_wrt_torso_solve(q_start);
  Eigen::Matrix3d R_des = a_flange_start.linear();
  p_des = a_flange_start.translation();
  cartesian_affine_samples_.push_back(a_flange_start);
  cout << "fwd kin from q_start p: " << p_des.transpose() << endl;
  cout << "fwd kin from q_start R: " << endl;
  cout << a_flange_start.linear() << endl;

  a_flange_end.linear() = R_des;
  a_flange_des.linear() = R_des;
  a_flange_end.translation() = p_des + delta_p;

  int nsolns;
  bool reachable_proposition;
  int nsteps = 0;

  p_start = a_flange_start.translation();
  p_end = a_flange_end.translation();
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
    a_flange_des.translation() = p_des;
    cartesian_affine_samples_.push_back(a_flange_des);
    cout << "trying: " << p_des.transpose() << endl;
    nsolns = baxter_IK_solver_.ik_solve_approx_wrt_torso(a_flange_des, q_solns);
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

  cout << "instantiating a JointSpacePlanner:" << endl;
  {
    JointSpacePlanner jsp(path_options, jspace_planner_weights_);
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

bool CartTrajPlanner::refine_cartesian_path_plan(std::vector<Eigen::VectorXd> &optimal_path)
{
  int nsamps_path = optimal_path.size();

  int nsamps_cart = cartesian_affine_samples_.size();

  if (nsamps_path != nsamps_cart)
  {
    ROS_WARN("found %d samples in provided optimal path", nsamps_path);
    ROS_WARN("found %d samples in internal vector of Cartesian samples", nsamps_cart);
    ROS_WARN("number of jspace samples does not match number of cartesian samples; cannot refine path:");
    return false;
  }
  if (nsamps_path < 2)
  {
    ROS_WARN("refine_cartesian path: not enough points in provided path");
    return false;
  }

  ROS_INFO("refining cartesian solutions...");
  Eigen::Affine3d des_flange_affine;
  Eigen::VectorXd approx_jspace_soln;
  Eigen::VectorXd refined_jspace_soln;
  Vectorq7x1 q_in, q_7dof_precise;
  bool valid;

  for (int i = 1; i < nsamps_cart; i++)
  {
    des_flange_affine = cartesian_affine_samples_[i];
    approx_jspace_soln = optimal_path[i];
    q_in = approx_jspace_soln;
    valid = baxter_IK_solver_.improve_7dof_soln_wrt_torso(des_flange_affine, q_in, q_7dof_precise);
    if (valid)
    {
      refined_jspace_soln = q_7dof_precise;
      optimal_path[i] = refined_jspace_soln;
    }
  }
  return true;
}

bool CartTrajPlanner::cartesian_path_planner_wrist(Vectorq7x1 q_start, Eigen::Affine3d a_flange_end,
                                                   std::vector<Eigen::VectorXd> &optimal_path)
{
  std::vector<std::vector<Eigen::VectorXd> > path_options;
  path_options.clear();
  std::vector<Eigen::VectorXd> single_layer_nodes;
  Eigen::VectorXd node;
  Eigen::Affine3d a_flange_des, a_flange_start;
  Eigen::Matrix3d R_des = a_flange_end.linear();
  a_flange_start = baxter_fwd_solver_.fwd_kin_flange_wrt_torso_solve(q_start);
  cout << "fwd kin from q_start: " << a_flange_start.translation().transpose() << endl;
  cout << "fwd kin from q_start R: " << endl;
  cout << a_flange_start.linear() << endl;

  a_flange_start.linear() = R_des;
  a_flange_des.linear() = R_des;

  int nsolns;
  bool reachable_proposition;
  int nsteps = 0;
  Eigen::Vector3d p_des, dp_vec, del_p, p_start, p_end;
  p_start = a_flange_start.translation();
  p_end = a_flange_end.translation();
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

  for (int istep = 1; istep < nsteps; istep++)
  {
    p_des += dp_vec;
    a_flange_des.translation() = p_des;
    cout << "trying: " << p_des.transpose() << endl;

    nsolns = baxter_IK_solver_.ik_wristpt_solve_approx_wrt_torso(a_flange_des, q_solns);
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

  cout << "instantiating a JointSpacePlanner:" << endl;
  {
    JointSpacePlanner jsp(path_options, jspace_planner_weights_);
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

bool CartTrajPlanner::jspace_path_planner_to_affine_goal(Vectorq7x1 q_start, Eigen::Affine3d a_flange_end,
                                                         std::vector<Eigen::VectorXd> &optimal_path)
{
  Eigen::VectorXd qx_start(7), qx_end(7);
  std::vector<Vectorq7x1> q_solns;
  std::vector<std::vector<Eigen::VectorXd> > path_options;
  path_options.clear();
  std::vector<Eigen::VectorXd> single_layer_nodes;
  Eigen::VectorXd node, precise_node;
  Vectorq7x1 q_approx, q_refined;
  single_layer_nodes.clear();
  node = q_start;
  single_layer_nodes.push_back(node);

  for (int i = 0; i < 7; i++)
  {
    qx_start[i] = q_start[i];
  }
  cout << "jspace planner to Cartesian goal: " << endl;

  int nsolns = baxter_IK_solver_.ik_solve_approx_wrt_torso(a_flange_end, q_solns);
  std::cout << "nsolns at goal pose = " << nsolns << endl;
  single_layer_nodes.clear();
  if (nsolns < 1)
    return false;

  for (int isoln = 0; isoln < nsolns; isoln++)
  {
    q_approx = q_solns[isoln];
    if (baxter_IK_solver_.improve_7dof_soln_wrt_torso(a_flange_end, q_approx, q_refined))
    {
      precise_node = q_refined;
      cout << "precise_node: " << precise_node.transpose() << endl;
      single_layer_nodes.push_back(precise_node);
    }
  }
  nsolns = single_layer_nodes.size();
  ROS_INFO("found %d refined goal IK solns", nsolns);
  if (nsolns < 1)
  {
    return false;
  }

  optimal_path.clear();
  optimal_path.push_back(qx_start);

  Eigen::VectorXd dq_move(7), q_modified_start(7);
  q_modified_start = qx_start;
  q_modified_start[1] = 0;

  cout << "q_modified_start: " << q_modified_start.transpose() << endl;

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
    for (int j = 0; j < 7; j++)
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

Eigen::Affine3d CartTrajPlanner::get_fk_Affine_from_qvec(Vectorq7x1 q_vec)
{
  Eigen::Affine3d Affine_pose;
  Affine_pose = baxter_fwd_solver_.fwd_kin_flange_wrt_torso_solve(q_vec);
}
