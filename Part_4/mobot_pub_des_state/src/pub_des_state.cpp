#include "pub_des_state.hpp"

DesStatePublisher::DesStatePublisher(rclcpp::node::Node::SharedPtr node) : node_(node)
{
  trajBuilder_.set_dt(dt);

  accel_max_ = accel_max;
  trajBuilder_.set_accel_max(accel_max_);
  alpha_max_ = alpha_max;
  trajBuilder_.set_alpha_max(alpha_max_);
  speed_max_ = speed_max;
  trajBuilder_.set_speed_max(speed_max_);
  omega_max_ = omega_max;
  trajBuilder_.set_omega_max(omega_max_);
  path_move_tol_ = path_move_tol;
  trajBuilder_.set_path_move_tol_(path_move_tol_);
  initializePublishers();
  initializeServices();

  halt_twist_.linear.x = 0.0;
  halt_twist_.linear.y = 0.0;
  halt_twist_.linear.z = 0.0;
  halt_twist_.angular.x = 0.0;
  halt_twist_.angular.y = 0.0;
  halt_twist_.angular.z = 0.0;
  motion_mode_ = DONE_W_SUBGOAL;
  e_stop_trigger_ = false;
  e_stop_reset_ = false;
  current_pose_ = trajBuilder_.xyPsi2PoseStamped(0, 0, 0);
  start_pose_ = current_pose_;
  end_pose_ = current_pose_;
  current_des_state_.twist.twist = halt_twist_;
  current_des_state_.pose.pose = current_pose_.pose;
  halt_state_ = current_des_state_;
  seg_start_state_ = current_des_state_;
  seg_end_state_ = current_des_state_;
}

void DesStatePublisher::initializeServices()
{
  std::cout << "Initializing Services" << std::endl;
  estop_service_ = node_->create_service<std_srvs::srv::Trigger>(
      "estop_service", [this](const std::shared_ptr<rmw_request_id_t> request_header,
                              const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                              std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        ignore(request_header);
        ignore(request);
        ROS_WARN("estop!!");
        e_stop_trigger_ = true;
        response->success = true;
      });
  estop_clear_service_ = node_->create_service<std_srvs::srv::Trigger>(
      "clear_estop_service", [this](const std::shared_ptr<rmw_request_id_t> request_header,
                                    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        ignore(request_header);
        ignore(request);
        ROS_INFO("estop reset");
        e_stop_reset_ = true;
        response->success = true;
      });
  flush_path_queue_ = node_->create_service<std_srvs::srv::Trigger>(
      "flush_path_queue_service", [this](const std::shared_ptr<rmw_request_id_t> request_header,
                                         const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                         std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        ignore(request_header);
        ignore(request);
        ROS_WARN("flushing path queue");
        while (!path_queue_.empty())
        {
          path_queue_.pop();
        }
        response->success = true;
      });
  append_path_ = node_->create_service<mobot_pub_des_state::srv::Path>(
      "append_path_queue_service", [this](const std::shared_ptr<rmw_request_id_t> request_header,
                                          const std::shared_ptr<mobot_pub_des_state::srv::Path::Request> request,
                                          std::shared_ptr<mobot_pub_des_state::srv::Path::Response> response) {
        ignore(request_header);
        int npts = request->path.poses.size();
        ROS_INFO("appending path queue with %d points", npts);
        for (int i = 0; i < npts; i++)
        {
          path_queue_.push(request->path.poses[i]);
        }
        response->status = true;
      });
}

void DesStatePublisher::initializePublishers()
{
  ROS_INFO("Initializing Publishers");
  desired_state_publisher_ = node_->create_publisher<nav_msgs::msg::Odometry>("/desState", rmw_qos_profile_default);
  des_psi_publisher_ = node_->create_publisher<std_msgs::msg::Float64>("/desPsi", rmw_qos_profile_default);
}

void DesStatePublisher::set_init_pose(double x, double y, double psi)
{
  current_pose_ = trajBuilder_.xyPsi2PoseStamped(x, y, psi);
}

void DesStatePublisher::pub_next_state()
{
  if (e_stop_trigger_)
  {
    e_stop_trigger_ = false;
    trajBuilder_.build_braking_traj(current_pose_, des_state_vec_);
    motion_mode_ = HALTING;
    traj_pt_i_ = 0;
    npts_traj_ = des_state_vec_.size();
  }

  if (e_stop_reset_)
  {
    e_stop_reset_ = false;
    if (motion_mode_ != E_STOPPED)
    {
      ROS_WARN("e-stop reset while not in e-stop mode");
    }

    else
    {
      motion_mode_ = DONE_W_SUBGOAL;
    }
  }

  builtin_interfaces::msg::Time t0;
  switch (motion_mode_)
  {
    case E_STOPPED:
      desired_state_publisher_->publish(halt_state_);
      break;

    case HALTING:
      current_des_state_ = des_state_vec_[traj_pt_i_];
      t0.sec = (double)std::chrono::system_clock::now().time_since_epoch().count();
      current_des_state_.header.stamp = t0;

      desired_state_publisher_->publish(current_des_state_);
      current_pose_.pose = current_des_state_.pose.pose;
      current_pose_.header = current_des_state_.header;
      des_psi_ = trajBuilder_.convertPlanarQuat2Psi(current_pose_.pose.orientation);
      float_msg_.data = des_psi_;
      des_psi_publisher_->publish(float_msg_);

      traj_pt_i_++;

      if (traj_pt_i_ >= npts_traj_)
      {
        halt_state_ = des_state_vec_.back();

        halt_state_.twist.twist = halt_twist_;
        seg_end_state_ = halt_state_;
        current_des_state_ = seg_end_state_;
        motion_mode_ = E_STOPPED;
      }
      break;

    case PURSUING_SUBGOAL:
      current_des_state_ = des_state_vec_[traj_pt_i_];
      current_pose_.pose = current_des_state_.pose.pose;
      desired_state_publisher_->publish(current_des_state_);

      t0.sec = (double)std::chrono::system_clock::now().time_since_epoch().count();
      current_des_state_.header.stamp = t0;

      des_psi_ = trajBuilder_.convertPlanarQuat2Psi(current_pose_.pose.orientation);
      float_msg_.data = des_psi_;
      des_psi_publisher_->publish(float_msg_);
      traj_pt_i_++;

      if (traj_pt_i_ >= npts_traj_)
      {
        motion_mode_ = DONE_W_SUBGOAL;
        seg_end_state_ = des_state_vec_.back();
        if (!path_queue_.empty())
        {
          path_queue_.pop();
        }
        ROS_INFO("reached a subgoal: x = %f, y= %f\n", current_pose_.pose.position.x, current_pose_.pose.position.y);
      }
      break;

    case DONE_W_SUBGOAL:
      if (!path_queue_.empty())
      {
        int n_path_pts = path_queue_.size();
        ROS_INFO("%d points in path queue\n", n_path_pts);
        start_pose_ = current_pose_;
        end_pose_ = path_queue_.front();
        trajBuilder_.build_point_and_go_traj(start_pose_, end_pose_, des_state_vec_);
        traj_pt_i_ = 0;
        npts_traj_ = des_state_vec_.size();
        motion_mode_ = PURSUING_SUBGOAL;
        ROS_INFO("computed new trajectory to pursue");
      }
      else
      {
        desired_state_publisher_->publish(seg_end_state_);
      }
      break;

    default:
      ROS_WARN("motion mode not recognized!");
      desired_state_publisher_->publish(current_des_state_);
      break;
  }
}
