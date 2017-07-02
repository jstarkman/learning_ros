#include "joint_space_planner/joint_space_planner.hpp"

JointSpacePlanner::JointSpacePlanner(int i, int j)
{
}

JointSpacePlanner::JointSpacePlanner(std::vector<std::vector<Eigen::VectorXd> > &path_options, Eigen::VectorXd weights)
  : penalty_weights_(weights)
{
  path_options_ptr_ = &path_options;

  problem_dimension_ = weights.size();

  std::cout << "vector size: " << problem_dimension_ << std::endl;
  nlayers_ = path_options.size();
  std::cout << "num layers = " << nlayers_ << std::endl;

  all_costs_.resize(nlayers_);
  next_indices_.resize(nlayers_);
  for (int i = 0; i < nlayers_; i++)
  {
    int size_ilayer = path_options[i].size();
    std::cout << "ilayer: " << i << "; # options = " << size_ilayer << std::endl;
    all_costs_[i].resize(size_ilayer);
    next_indices_[i].resize(size_ilayer);
  }
  std::cout << "calling constructor helper: " << std::endl;
  constructor_helper_();

  std::cout << "computing min costs from constructor" << std::endl;
  compute_all_min_costs(path_options);

  std::cout << "computing optimal path in constructor" << std::endl;
  compute_optimal_path(path_options);
  std::cout << "done with constructor" << std::endl;
}

JointSpacePlanner::JointSpacePlanner(std::vector<std::vector<Eigen::VectorXd> > &path_options, Eigen::VectorXd weights,
                                     std::vector<std::vector<double> > &humerus_sensitivities)
  : penalty_weights_(weights)
{
  path_options_ptr_ = &path_options;

  problem_dimension_ = weights.size();

  std::cout << "vector size: " << problem_dimension_ << std::endl;
  nlayers_ = path_options.size();
  std::cout << "num layers = " << nlayers_ << std::endl;

  for (int i = 0; i < nlayers_; i++)
  {
    int size_ilayer = path_options[i].size();
    all_costs_[i].resize(size_ilayer);
    next_indices_[i].resize(size_ilayer);
  }

  constructor_helper_();

  std::cout << "computing min costs from constructor" << std::endl;
  compute_all_min_costs(path_options);

  std::cout << "computing optimal path in constructor" << std::endl;
  compute_optimal_path(path_options);
  std::cout << "done with constructor" << std::endl;
}

void JointSpacePlanner::augment_all_min_costs_(std::vector<std::vector<double> > &humerus_sensitivities)
{
}

void JointSpacePlanner::constructor_helper_()
{
  all_costs_.resize(nlayers_);
  next_indices_.resize(nlayers_);

  int size_last_layer = all_costs_[nlayers_ - 1].size();
  std::cout << "last layer has num options = " << size_last_layer << std::endl;
  for (int i = 0; i < size_last_layer; i++)
  {
    all_costs_[nlayers_ - 1][i] = 0.0;
    next_indices_[nlayers_ - 1][i] = -1;
  }

  optimal_path_.resize(nlayers_);
  vec_of_zeros_.resize(problem_dimension_);
  diff_.resize(problem_dimension_);
  diff_sqd_.resize(problem_dimension_);

  for (int i = 0; i < nlayers_; i++)
  {
    optimal_path_[i] = vec_of_zeros_;
  }
}

void JointSpacePlanner::get_soln(std::vector<Eigen::VectorXd> &optimal_path)
{
  for (int ilayer = 0; ilayer < nlayers_; ilayer++)
  {
    optimal_path[ilayer] = optimal_path_[ilayer];
  }
}

double JointSpacePlanner::score_move(Eigen::VectorXd pose1, Eigen::VectorXd pose2)
{
  diff_ = pose1 - pose2;
  for (int i = 0; i < problem_dimension_; i++)
  {
    diff_sqd_(i) = diff_(i) * diff_(i);
  }
  double penalty = penalty_weights_.dot(diff_sqd_);

  return penalty;
}

bool JointSpacePlanner::compute_all_min_costs()
{
  for (int i_layer = nlayers_ - 1; i_layer > 0; i_layer--)
  {
    find_best_moves_single_layer(i_layer);
  }
  std::cout << "done computing all costs" << std::endl;

  return true;
}

bool JointSpacePlanner::compute_all_min_costs(std::vector<std::vector<Eigen::VectorXd> > &path_options)
{
  for (int i_layer = nlayers_ - 1; i_layer > 0; i_layer--)
  {
    int target_options = path_options[i_layer].size();
    int prior_options = path_options[i_layer - 1].size();
    std::cout << "compute_all_min_costs, layer: " << i_layer << " has " << target_options << " target options and "
              << prior_options << " prior options" << std::endl;
    find_best_moves_single_layer(path_options, i_layer);
  }

  return true;
}

bool JointSpacePlanner::find_best_moves_single_layer(int target_layer_index)
{
  double cost_to_go;
  double inc_cost_to_go;

  std::cout << "find_best_moves_single_layer: target layer: " << target_layer_index << std::endl;
  prior_pose_options_ = path_options_[target_layer_index - 1];
  int n_prior_poses = prior_pose_options_.size();
  std::cout << "n_prior_poses= " << n_prior_poses << std::endl;

  next_pose_options_ = path_options_[target_layer_index];
  int n_target_poses = next_pose_options_.size();
  std::cout << "n_target_poses= " << n_target_poses << std::endl;
  target_pose_costs_to_go_ = all_costs_[target_layer_index];

  int j_target_0;
  for (int i_prior_pose = 0; i_prior_pose < n_prior_poses; i_prior_pose++)
  {
    prior_pose_ = prior_pose_options_[i_prior_pose];

    move_index_min_cost_to_go_ = 0;
    j_target_0 = move_index_min_cost_to_go_;
    target_pose_ = next_pose_options_[j_target_0];
    inc_cost_to_go = score_move(prior_pose_, target_pose_);
    cost_to_go = target_pose_costs_to_go_[j_target_0] + inc_cost_to_go;
    min_cost_to_go_ = cost_to_go;

    std::cout << "target layer " << target_layer_index << "; i_prior_pose: " << i_prior_pose
              << "; j_target: " << j_target_0 << std::endl;
    std::cout << "prior pose = " << prior_pose_.transpose() << std::endl;
    std::cout << "target pose: " << target_pose_.transpose() << std::endl;
    std::cout << "target cost-to-go = " << target_pose_costs_to_go_[j_target_0]
              << "; inc_cost_to_go = " << inc_cost_to_go << std::endl;

    for (int j_target = 1; j_target < n_target_poses; j_target++)
    {
      target_pose_ = next_pose_options_[j_target];
      inc_cost_to_go = score_move(prior_pose_, target_pose_);
      cost_to_go = target_pose_costs_to_go_[j_target] + inc_cost_to_go;
      std::cout << "target layer " << target_layer_index << "; i_prior_pose: " << i_prior_pose
                << "; j_target: " << j_target << std::endl;
      std::cout << "target pose: " << target_pose_.transpose() << std::endl;
      std::cout << "target cost-to-go = " << target_pose_costs_to_go_[j_target]
                << "; inc_cost_to_go = " << inc_cost_to_go << std::endl;
      if (cost_to_go < min_cost_to_go_)
      {
        min_cost_to_go_ = cost_to_go;
        move_index_min_cost_to_go_ = j_target;
      }
    }

    all_costs_[target_layer_index - 1][i_prior_pose] = min_cost_to_go_;
    next_indices_[target_layer_index - 1][i_prior_pose] = move_index_min_cost_to_go_;
  }

  std::cout << "done computing costs for target layer " << target_layer_index << std::endl;
  return true;
}

bool JointSpacePlanner::find_best_moves_single_layer(std::vector<std::vector<Eigen::VectorXd> > &path_options,
                                                     int target_layer_index)
{
  double cost_to_go;
  double inc_cost_to_go;

  prior_pose_options_ = path_options[target_layer_index - 1];
  int n_prior_poses = prior_pose_options_.size();

  next_pose_options_ = path_options[target_layer_index];
  int n_target_poses = next_pose_options_.size();

  target_pose_costs_to_go_ = all_costs_[target_layer_index];

  int j_target_0;
  for (int i_prior_pose = 0; i_prior_pose < n_prior_poses; i_prior_pose++)
  {
    prior_pose_ = prior_pose_options_[i_prior_pose];

    move_index_min_cost_to_go_ = 0;
    j_target_0 = move_index_min_cost_to_go_;
    target_pose_ = next_pose_options_[j_target_0];
    inc_cost_to_go = score_move(prior_pose_, target_pose_);
    cost_to_go = target_pose_costs_to_go_[j_target_0] + inc_cost_to_go;
    min_cost_to_go_ = cost_to_go;

    for (int j_target = 1; j_target < n_target_poses; j_target++)
    {
      target_pose_ = next_pose_options_[j_target];
      inc_cost_to_go = score_move(prior_pose_, target_pose_);
      cost_to_go = target_pose_costs_to_go_[j_target] + inc_cost_to_go;

      if (cost_to_go < min_cost_to_go_)
      {
        min_cost_to_go_ = cost_to_go;
        move_index_min_cost_to_go_ = j_target;
      }
    }

    all_costs_[target_layer_index - 1][i_prior_pose] = min_cost_to_go_;
    next_indices_[target_layer_index - 1][i_prior_pose] = move_index_min_cost_to_go_;
  }

  return true;
}

bool JointSpacePlanner::compute_optimal_path(std::vector<Eigen::VectorXd> optimal_path)
{
  std::cout << "computing all min costs" << std::endl;

  std::cout << "finding optimal path" << std::endl;
  double cost_to_go, min_total_trip_cost;
  double trip_cost;

  int nstart_solns = all_costs_[0].size();
  // int start_index = 0;
  min_cost_to_go_ = all_costs_[0][0];
  move_index_min_cost_to_go_ = 0;
  for (int istart = 1; istart < nstart_solns; istart++)
  {
    cost_to_go = all_costs_[0][istart];
    if (cost_to_go < min_cost_to_go_)
    {
      min_cost_to_go_ = cost_to_go;
      move_index_min_cost_to_go_ = istart;
    }
  }
  trip_cost = 0;
  std::cout << "entire trip min cost is " << min_cost_to_go_ << " starting from initial node "
            << move_index_min_cost_to_go_ << std::endl;
  min_total_trip_cost_ = min_cost_to_go_;
  min_total_trip_cost = min_cost_to_go_;

  optimal_path_[0] = path_options_[0][move_index_min_cost_to_go_];

  std::cout << "layer 0 pose = " << optimal_path_[0].transpose() << std::endl;

  for (int klayer = 1; klayer < nlayers_; klayer++)
  {
    move_index_min_cost_to_go_ = next_indices_[klayer - 1][move_index_min_cost_to_go_];

    optimal_path_[klayer] = path_options_[klayer][move_index_min_cost_to_go_];

    trip_cost += score_move(optimal_path_[klayer - 1], optimal_path_[klayer]);
  }
  std::cout << "recomputed trip cost = " << trip_cost << std::endl;
  std::cout << "expected trip cost = " << min_total_trip_cost << std::endl;
  optimal_path = optimal_path_;
  return true;
}

bool JointSpacePlanner::compute_optimal_path(std::vector<std::vector<Eigen::VectorXd> > &path_options)
{
  std::cout << "finding optimal path" << std::endl;
  double cost_to_go;  //, min_total_trip_cost;
  double trip_cost;

  int nstart_solns = all_costs_[0].size();
  // int start_index = 0;
  min_cost_to_go_ = all_costs_[0][0];
  move_index_min_cost_to_go_ = 0;
  for (int istart = 1; istart < nstart_solns; istart++)
  {
    cost_to_go = all_costs_[0][istart];
    if (cost_to_go < min_cost_to_go_)
    {
      min_cost_to_go_ = cost_to_go;
      move_index_min_cost_to_go_ = istart;
    }
  }
  trip_cost = 0;

  min_total_trip_cost_ = min_cost_to_go_;

  optimal_path_[0] = path_options[0][move_index_min_cost_to_go_];

  for (int klayer = 1; klayer < nlayers_; klayer++)
  {
    move_index_min_cost_to_go_ = next_indices_[klayer - 1][move_index_min_cost_to_go_];

    optimal_path_[klayer] = path_options[klayer][move_index_min_cost_to_go_];

    trip_cost += score_move(optimal_path_[klayer - 1], optimal_path_[klayer]);
  }
  std::cout << "recomputed trip cost = " << trip_cost << std::endl;
  std::cout << "expected trip cost = " << min_total_trip_cost_ << std::endl;

  return true;
}
