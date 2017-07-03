

#include <cartesian_planner/baxter_rt_arm_cartesian_planner.h>
#include <cartesian_planner/cart_moveAction.h>

#include <baxter_trajectory_streamer/baxter_trajectory_streamer.h>
#include <baxter_trajectory_streamer/trajAction.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib/server/simple_action_server.h>

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt8.h>

#include <tf/transform_listener.h>
#include <xform_utils/xform_utils.h>

const double ARM_ERR_TOL = 0.1;

int g_js_doneCb_flag = 0;

class ArmMotionInterface
{
private:
  ros::NodeHandle nh_;
  XformUtils xformUtils;

  actionlib::SimpleActionServer<cartesian_planner::cart_moveAction> cart_move_as_;

  actionlib::SimpleActionClient<baxter_trajectory_streamer::trajAction> traj_streamer_action_client_;

  cartesian_planner::cart_moveGoal cart_goal_;
  cartesian_planner::cart_moveResult cart_result_;

  baxter_trajectory_streamer::trajGoal js_goal_;
  baxter_trajectory_streamer::trajResult js_result_;

  void js_doneCb_(const actionlib::SimpleClientGoalState& state,
                  const baxter_trajectory_streamer::trajResultConstPtr& result);

  void executeCB(const actionlib::SimpleActionServer<cartesian_planner::cart_moveAction>::GoalConstPtr& goal);

  double computed_arrival_time_;

  geometry_msgs::PoseStamped goal_gripper_pose_;

  geometry_msgs::Pose current_gripper_pose_, current_flange_pose_;
  geometry_msgs::PoseStamped current_gripper_stamped_pose_, current_flange_stamped_pose_;

  tf::StampedTransform generic_toolflange_frame_wrt_gripper_frame_stf_;
  tf::StampedTransform generic_gripper_frame_wrt_tool_flange_stf_;
  tf::StampedTransform torso_wrt_system_ref_frame_stf_;

  Eigen::Affine3d goal_gripper_affine_, goal_flange_affine_;

  unsigned short int command_mode_;

  Vectorq7x1 q_vec_;
  Eigen::VectorXd q_vec_Xd_;

  Eigen::VectorXd q_start_Xd_;
  Eigen::VectorXd last_arm_jnt_cmd_;

  Eigen::Affine3d affine_tool_wrt_torso_, affine_flange_wrt_torso_;
  Eigen::Affine3d A_tool_wrt_flange_;

  double arrival_time_;
  bool path_is_valid_;

  std::vector<Eigen::VectorXd> optimal_path_;
  trajectory_msgs::JointTrajectory des_trajectory_;

  Eigen::VectorXd last_arm_jnt_;

  Eigen::Matrix3d R_gripper_down_;
  Vectorq7x1 q_pre_pose_;
  Eigen::VectorXd q_pre_pose_Xd_;
  Eigen::VectorXd q_goal_pose_Xd_;

  Baxter_fwd_solver baxter_fwd_solver_;
  CartTrajPlanner cartTrajPlanner_;

  Baxter_traj_streamer baxter_traj_streamer_;

  void execute_planned_move(void);

  ros::ServiceServer arm_motion_interface_service_;

  int path_id_;
  Eigen::Affine3d a_tool_start_, a_tool_end_;

  Eigen::Vector3d delta_p_;
  Eigen::VectorXd q_vec_start_rqst_;
  Eigen::VectorXd q_vec_end_rqst_;
  Eigen::VectorXd q_vec_start_resp_;
  Eigen::VectorXd q_vec_end_resp_;
  Eigen::Affine3d a_flange_end_;

  double time_scale_stretch_factor_;
  bool received_new_request_;
  bool busy_working_on_a_request_;
  bool finished_before_timeout_;

public:
  ArmMotionInterface(ros::NodeHandle* nodehandle);

  ~ArmMotionInterface(void)
  {
  }

  tf::TransformListener* tfListener_;

  void display_affine(Eigen::Affine3d affine);

  Eigen::VectorXd get_jspace_start_(void);

  Eigen::VectorXd get_joint_angles(void);

  void compute_tool_stamped_pose(void);
  void compute_flange_stamped_pose(void);

  bool plan_path_current_to_goal_gripper_pose();

  bool plan_jspace_path_current_to_cart_gripper_pose();

  bool plan_fine_path_current_to_goal_gripper_pose();

  bool plan_path_current_to_goal_dp_xyz();
  bool plan_cartesian_delta_p(Vectorq7x1 q_start, Eigen::Vector3d delta_p);

  bool plan_jspace_path_qstart_to_qend(Vectorq7x1 q_start, Vectorq7x1 q_goal);
  bool plan_jspace_path_qstart_to_qend(Eigen::VectorXd q_start_Xd, Eigen::VectorXd q_goal_Xd);

  void rescale_planned_trajectory_time(double time_stretch_factor);
  bool refine_cartesian_path_soln();

  Eigen::Affine3d xform_gripper_pose_to_affine_flange_wrt_torso(geometry_msgs::PoseStamped des_pose_gripper);
};

void ArmMotionInterface::executeCB(
    const actionlib::SimpleActionServer<cartesian_planner::cart_moveAction>::GoalConstPtr& goal)
{
  ROS_INFO("in executeCB of ArmMotionInterface");
  cart_goal_ = *goal;
  command_mode_ = goal->command_code;
  ROS_INFO_STREAM("received command mode " << command_mode_);
  int njnts;

  switch (command_mode_)
  {
    case cartesian_planner::cart_moveGoal::ARM_TEST_MODE:
      ROS_INFO("responding to request TEST_MODE: ");
      cart_result_.return_code = cartesian_planner::cart_moveResult::SUCCESS;
      cart_move_as_.setSucceeded(cart_result_);
      break;

    case cartesian_planner::cart_moveGoal::GET_Q_DATA:
      ROS_INFO("responding to request GET_Q_DATA");
      get_joint_angles();
      cart_result_.q_arm.resize(7);
      for (int i = 0; i < 7; i++)
      {
        cart_result_.q_arm[i] = q_vec_Xd_[i];
      }
      cart_result_.return_code = cartesian_planner::cart_moveResult::SUCCESS;
      cart_move_as_.setSucceeded(cart_result_);
      break;

    case cartesian_planner::cart_moveGoal::GET_TOOL_POSE:
      ROS_INFO("responding to request GET_TOOL_POSE");
      compute_tool_stamped_pose();
      cart_result_.current_pose_gripper = current_gripper_stamped_pose_;
      cart_result_.return_code = cartesian_planner::cart_moveResult::SUCCESS;
      cart_move_as_.setSucceeded(cart_result_);
      break;
    /*
 case cartesian_planner::cart_moveGoal::GET_FLANGE_POSE:
    ROS_INFO("responding to request GET_FLANGE_POSE");
    compute_flange_stamped_pose();
    cart_result_.current_pose_flange = current_flange_stamped_pose_;
    cart_result_.return_code = cartesian_planner::cart_moveResult::SUCCESS;
    cart_move_as_.setSucceeded(cart_result_);
    break; */

    case cartesian_planner::cart_moveGoal::PLAN_PATH_CURRENT_TO_WAITING_POSE:
      ROS_INFO("responding to request PLAN_PATH_CURRENT_TO_WAITING_POSE");
      q_start_Xd_ = get_jspace_start_();

      plan_jspace_path_qstart_to_qend(q_start_Xd_, q_pre_pose_Xd_);
      busy_working_on_a_request_ = false;
      break;

    case cartesian_planner::cart_moveGoal::PLAN_JSPACE_PATH_CURRENT_TO_QGOAL:
      ROS_INFO("responding to request PLAN_JSPACE_PATH_CURRENT_TO_QGOAL");
      q_start_Xd_ = get_jspace_start_();
      q_goal_pose_Xd_.resize(7);
      njnts = goal->q_goal.size();
      if (njnts != 7)
      {
        ROS_WARN("joint-space goal is wrong dimension");
        cart_result_.return_code = cartesian_planner::cart_moveResult::PATH_NOT_VALID;
      }
      else
      {
        for (int i = 0; i < 7; i++)
          q_goal_pose_Xd_[i] = goal->q_goal[i];

        plan_jspace_path_qstart_to_qend(q_start_Xd_, q_goal_pose_Xd_);
        busy_working_on_a_request_ = false;
      }
      break;
    case cartesian_planner::cart_moveGoal::PLAN_PATH_CURRENT_TO_GOAL_GRIPPER_POSE:
      plan_path_current_to_goal_gripper_pose();
      break;

    case cartesian_planner::cart_moveGoal::PLAN_FINE_PATH_CURRENT_TO_GOAL_GRIPPER_POSE:
      plan_fine_path_current_to_goal_gripper_pose();

      break;

    case cartesian_planner::cart_moveGoal::PLAN_PATH_CURRENT_TO_GOAL_DP_XYZ:
      plan_path_current_to_goal_dp_xyz();
      break;

    case cartesian_planner::cart_moveGoal::REFINE_PLANNED_TRAJECTORY:
      refine_cartesian_path_soln();
      break;

    case cartesian_planner::cart_moveGoal::TIME_RESCALE_PLANNED_TRAJECTORY:
      time_scale_stretch_factor_ = goal->time_scale_stretch_factor;
      rescale_planned_trajectory_time(time_scale_stretch_factor_);
      break;

    case cartesian_planner::cart_moveGoal::EXECUTE_PLANNED_PATH:
      ROS_INFO("responding to request EXECUTE_PLANNED_PATH");
      execute_planned_move();
      break;

    case cartesian_planner::cart_moveGoal::PLAN_JSPACE_PATH_CURRENT_TO_CART_GRIPPER_POSE:
      plan_jspace_path_current_to_cart_gripper_pose();
      break;

    default:
      ROS_WARN("this command mode is not defined: %d", command_mode_);
      cart_result_.return_code = cartesian_planner::cart_moveResult::COMMAND_CODE_NOT_RECOGNIZED;
      cart_move_as_.setAborted(cart_result_);
  }
}

ArmMotionInterface::ArmMotionInterface(ros::NodeHandle* nodehandle)
  : nh_(*nodehandle)
  , cart_move_as_(*nodehandle, "cartMoveActionServer", boost::bind(&ArmMotionInterface::executeCB, this, _1), false)
  , baxter_traj_streamer_(nodehandle)
  , traj_streamer_action_client_("rightArmTrajActionServer", true)
{
  ROS_INFO("in class constructor of ArmMotionInterface");

  q_pre_pose_ << -0.907528, -0.111813, 2.06622, 1.8737, -1.295, 2.00164, 0;
  q_pre_pose_Xd_ = q_pre_pose_;
  q_vec_start_rqst_ = q_pre_pose_;
  q_vec_end_rqst_ = q_pre_pose_;
  q_vec_start_resp_ = q_pre_pose_;
  q_vec_end_resp_ = q_pre_pose_;
  R_gripper_down_ = cartTrajPlanner_.get_R_gripper_down();

  command_mode_ = cartesian_planner::cart_moveGoal::ARM_TEST_MODE;

  received_new_request_ = false;
  busy_working_on_a_request_ = false;
  path_is_valid_ = false;
  path_id_ = 0;

  tfListener_ = new tf::TransformListener;

  bool tferr = true;
  int ntries = 0;
  ROS_INFO("waiting for tf between generic gripper frame and tool flange...");

  while (tferr)
  {
    tferr = false;
    try
    {
      tfListener_->lookupTransform("generic_gripper_frame", "right_hand", ros::Time(0),
                                   generic_toolflange_frame_wrt_gripper_frame_stf_);
    }
    catch (tf::TransformException& exception)
    {
      ROS_WARN("%s; retrying...", exception.what());
      tferr = true;
      ros::Duration(0.5).sleep();
      ros::spinOnce();
      ntries++;
      if (ntries > 5)
        ROS_WARN("did you launch baxter_static_transforms.launch?");
    }
  }
  ROS_INFO("tf is good for generic gripper frame w/rt right tool flange");
  xformUtils.printStampedTf(generic_toolflange_frame_wrt_gripper_frame_stf_);

  tferr = true;
  ROS_INFO("waiting for tf between system_ref_frame and torso...");

  while (tferr)
  {
    tferr = false;
    try
    {
      tfListener_->lookupTransform("system_ref_frame", "torso", ros::Time(0), torso_wrt_system_ref_frame_stf_);
    }
    catch (tf::TransformException& exception)
    {
      ROS_WARN("%s; retrying...", exception.what());
      tferr = true;
      ros::Duration(0.5).sleep();
      ros::spinOnce();
    }
  }
  ROS_INFO("tf is good for generic gripper frame w/rt right tool flange");
  xformUtils.printStampedTf(torso_wrt_system_ref_frame_stf_);

  ROS_INFO("waiting for right-arm joint-trajectory server: ");
  bool server_exists = false;
  while ((!server_exists) && (ros::ok()))
  {
    server_exists = traj_streamer_action_client_.waitForServer(ros::Duration(0.5));
    ros::spinOnce();
    ros::Duration(0.5).sleep();
    ROS_INFO("retrying...");
  }
  ROS_INFO("connected to right-arm trajectory-streamer action server");

  ROS_INFO("getting joint states: ");

  q_vec_Xd_ = get_joint_angles();
  ROS_INFO("got valid right-arm joint state");
  last_arm_jnt_cmd_ = q_vec_Xd_;

  ROS_INFO("starting action server: cartMoveActionServer ");
  cart_move_as_.start();
}

void ArmMotionInterface::js_doneCb_(const actionlib::SimpleClientGoalState& state,
                                    const baxter_trajectory_streamer::trajResultConstPtr& result)
{
  ROS_INFO("done-callback pinged by joint-space interpolator action server done");
  g_js_doneCb_flag = 1;
}

void ArmMotionInterface::display_affine(Eigen::Affine3d affine)
{
  cout << "Affine origin: " << affine.translation().transpose() << endl;
  cout << "Affine rotation: " << endl;
  cout << affine.linear() << endl;
}

Eigen::VectorXd ArmMotionInterface::get_jspace_start_(void)
{
  q_vec_Xd_ = get_joint_angles();

  double arm_err = (last_arm_jnt_cmd_ - q_vec_Xd_).norm();
  if (arm_err < ARM_ERR_TOL)
  {
    return last_arm_jnt_cmd_;
  }
  else
  {
    return q_vec_Xd_;
  }
}

Eigen::VectorXd ArmMotionInterface::get_joint_angles(void)
{
  q_vec_[0] = 1000;
  while (fabs(q_vec_[0]) > 3)
  {
    q_vec_ = baxter_traj_streamer_.get_qvec_right_arm();
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }
  Eigen::VectorXd q_vec_xd;
  q_vec_xd = q_vec_;

  q_vec_Xd_ = q_vec_xd;
  return q_vec_xd;
}

void ArmMotionInterface::compute_tool_stamped_pose(void)
{
  get_joint_angles();

  affine_tool_wrt_torso_ = baxter_fwd_solver_.fwd_kin_tool_wrt_torso_solve(q_vec_);
  current_gripper_pose_ = xformUtils.transformEigenAffine3dToPose(affine_tool_wrt_torso_);
  current_gripper_stamped_pose_.pose = current_gripper_pose_;
  current_gripper_stamped_pose_.header.stamp = ros::Time::now();
  current_gripper_stamped_pose_.header.frame_id = "torso";
}

void ArmMotionInterface::compute_flange_stamped_pose(void)
{
  get_joint_angles();

  affine_flange_wrt_torso_ = baxter_fwd_solver_.fwd_kin_flange_wrt_torso_solve(q_vec_);
  current_flange_pose_ = xformUtils.transformEigenAffine3dToPose(affine_flange_wrt_torso_);
  current_flange_stamped_pose_.pose = current_flange_pose_;
  current_flange_stamped_pose_.header.stamp = ros::Time::now();
  current_flange_stamped_pose_.header.frame_id = "torso";
}

void ArmMotionInterface::execute_planned_move(void)
{
  if (!path_is_valid_)
  {
    cart_result_.return_code = cartesian_planner::cart_moveResult::PATH_NOT_VALID;
    ROS_WARN("attempted to execute invalid path!");
    cart_move_as_.setAborted(cart_result_);
    return;
  }

  js_goal_.trajectory = des_trajectory_;

  ROS_INFO("sending action request to traj streamer node");
  ROS_INFO("computed arrival time is %f", computed_arrival_time_);
  busy_working_on_a_request_ = true;
  g_js_doneCb_flag = 0;
  traj_streamer_action_client_.sendGoal(js_goal_, boost::bind(&ArmMotionInterface::js_doneCb_, this, _1, _2));
  ROS_INFO("waiting on trajectory streamer...");
  while (g_js_doneCb_flag == 0)
  {
    ROS_INFO("...");
    ros::Duration(1.0).sleep();
    ros::spinOnce();
  }

  /*
  if (!finished_before_timeout_) {
      ROS_WARN("EXECUTE_PLANNED_PATH: giving up waiting on result");
      cart_result_.return_code = cartesian_planner::cart_moveResult::NOT_FINISHED_BEFORE_TIMEOUT;
      cart_move_as_.setSucceeded(cart_result_);
  } else {
   * */
  ROS_INFO("finished before timeout");
  cart_result_.return_code = cartesian_planner::cart_moveResult::SUCCESS;
  cart_move_as_.setSucceeded(cart_result_);

  path_is_valid_ = false;
  busy_working_on_a_request_ = false;

  std::vector<double> last_pt;
  last_pt = des_trajectory_.points.back().positions;
  int njnts = last_pt.size();
  for (int i = 0; i < njnts; i++)
  {
    last_arm_jnt_cmd_[i] = last_pt[i];
  }
}

void ArmMotionInterface::rescale_planned_trajectory_time(double time_stretch_factor)
{
  if (!path_is_valid_)
  {
    cart_result_.return_code = cartesian_planner::cart_moveResult::PATH_NOT_VALID;
    ROS_WARN("do not have a valid path!");
    cart_move_as_.setAborted(cart_result_);
  }

  int npts = des_trajectory_.points.size();
  double arrival_time_sec, new_arrival_time_sec;
  for (int i = 0; i < npts; i++)
  {
    arrival_time_sec = des_trajectory_.points[i].time_from_start.toSec();
    new_arrival_time_sec = arrival_time_sec * time_stretch_factor;
    ros::Duration arrival_duration(new_arrival_time_sec);
    des_trajectory_.points[i].time_from_start = arrival_duration;
  }
  computed_arrival_time_ = des_trajectory_.points.back().time_from_start.toSec();
  cart_result_.computed_arrival_time = computed_arrival_time_;

  cart_result_.return_code = cartesian_planner::cart_moveResult::SUCCESS;
  cart_move_as_.setSucceeded(cart_result_);
}

bool ArmMotionInterface::plan_cartesian_delta_p(Vectorq7x1 q_start, Eigen::Vector3d delta_p)
{
  cout << delta_p.transpose() << endl;

  path_is_valid_ = cartTrajPlanner_.cartesian_path_planner_delta_p(q_start, delta_p, optimal_path_);
  if (path_is_valid_)
  {
    ROS_INFO("plan_cartesian_delta_p: computed valid delta-p path");
    q_vec_start_resp_ = optimal_path_.front();
    q_vec_end_resp_ = optimal_path_.back();
  }
  else
  {
    ROS_WARN("plan_cartesian_delta_p: path plan attempt not successful");
  }

  return path_is_valid_;
}

bool ArmMotionInterface::plan_jspace_path_qstart_to_qend(Vectorq7x1 q_start, Vectorq7x1 q_goal)
{
  ROS_INFO("setting up a joint-space path");
  path_is_valid_ = cartTrajPlanner_.jspace_trivial_path_planner(q_start, q_goal, optimal_path_);
  if (path_is_valid_)
  {
    baxter_traj_streamer_.stuff_trajectory_right_arm(optimal_path_, des_trajectory_);
    computed_arrival_time_ = des_trajectory_.points.back().time_from_start.toSec();
    cart_result_.return_code = cartesian_planner::cart_moveResult::SUCCESS;
    cart_result_.computed_arrival_time = computed_arrival_time_;
    cart_move_as_.setSucceeded(cart_result_);
  }
  else
  {
    cart_result_.return_code = cartesian_planner::cart_moveResult::PATH_NOT_VALID;
    cart_result_.computed_arrival_time = -1.0;
    cart_move_as_.setSucceeded(cart_result_);
  }
  return path_is_valid_;
}

bool ArmMotionInterface::plan_jspace_path_qstart_to_qend(Eigen::VectorXd q_start_Xd, Eigen::VectorXd q_goal_Xd)
{
  ROS_INFO("setting up a joint-space path");
  Vectorq7x1 q_start, q_goal;
  q_start = q_start_Xd;
  q_goal = q_goal_Xd;

  path_is_valid_ = cartTrajPlanner_.jspace_trivial_path_planner(q_start, q_goal, optimal_path_);
  if (path_is_valid_)
  {
    baxter_traj_streamer_.stuff_trajectory_right_arm(optimal_path_, des_trajectory_);
    computed_arrival_time_ = des_trajectory_.points.back().time_from_start.toSec();
    cart_result_.return_code = cartesian_planner::cart_moveResult::SUCCESS;
    cart_result_.computed_arrival_time = computed_arrival_time_;
    cart_move_as_.setSucceeded(cart_result_);
  }
  else
  {
    cart_result_.return_code = cartesian_planner::cart_moveResult::PATH_NOT_VALID;
    cart_result_.computed_arrival_time = -1.0;
    cart_move_as_.setSucceeded(cart_result_);
  }

  return path_is_valid_;
}

Eigen::Affine3d
ArmMotionInterface::xform_gripper_pose_to_affine_flange_wrt_torso(geometry_msgs::PoseStamped des_pose_gripper)
{
  Eigen::Affine3d affine_flange_wrt_torso;
  tf::StampedTransform flange_stf, flange_wrt_torso_stf;
  geometry_msgs::PoseStamped flange_gmps, flange_wrt_torso_gmps;

  ROS_WARN("xform_gripper_pose_to_affine_flange_wrt_torso: input pose-stamped: ");
  xformUtils.printStampedPose(des_pose_gripper);
  tf::StampedTransform gripper_stf =
      xformUtils.convert_poseStamped_to_stampedTransform(des_pose_gripper, "generic_gripper_frame");

  ROS_INFO("gripper_stf: ");
  xformUtils.printStampedTf(gripper_stf);
  ROS_INFO("flange_stf");
  xformUtils.printStampedTf(flange_stf);
  bool mult_ok =
      xformUtils.multiply_stamped_tfs(gripper_stf, generic_toolflange_frame_wrt_gripper_frame_stf_, flange_stf);
  if (!mult_ok)
  {
    ROS_WARN("stf multiply not legal! ");
  }

  flange_gmps = xformUtils.get_pose_from_stamped_tf(flange_stf);

  tfListener_->transformPose("torso", flange_gmps, flange_wrt_torso_gmps);
  ROS_INFO("corresponding flange frame w/rt torso frame: ");
  xformUtils.printStampedPose(flange_wrt_torso_gmps);

  affine_flange_wrt_torso = xformUtils.transformPoseToEigenAffine3d(flange_wrt_torso_gmps);
  return affine_flange_wrt_torso;
}

bool ArmMotionInterface::plan_path_current_to_goal_gripper_pose()
{
  ROS_INFO("computing a cartesian trajectory to gripper goal pose");

  goal_gripper_pose_ = cart_goal_.des_pose_gripper;
  xformUtils.printStampedPose(goal_gripper_pose_);
  goal_flange_affine_ = xform_gripper_pose_to_affine_flange_wrt_torso(goal_gripper_pose_);

  ROS_INFO("flange goal");
  display_affine(goal_flange_affine_);
  Vectorq7x1 q_start;
  q_start = get_jspace_start_();
  path_is_valid_ = cartTrajPlanner_.cartesian_path_planner(q_start, goal_flange_affine_, optimal_path_);

  if (path_is_valid_)
  {
    baxter_traj_streamer_.stuff_trajectory_right_arm(optimal_path_, des_trajectory_);
    computed_arrival_time_ = des_trajectory_.points.back().time_from_start.toSec();
    cart_result_.return_code = cartesian_planner::cart_moveResult::SUCCESS;
    cart_result_.computed_arrival_time = computed_arrival_time_;
    cart_move_as_.setSucceeded(cart_result_);
  }
  else
  {
    cart_result_.return_code = cartesian_planner::cart_moveResult::PATH_NOT_VALID;
    cart_result_.computed_arrival_time = -1.0;
    cart_move_as_.setSucceeded(cart_result_);
  }

  return path_is_valid_;
}

/*
bool ArmMotionInterface::plan_path_current_to_goal_flange_pose() {
    ROS_INFO("computing a joint-space trajectory to right-arm flange goal pose");

    goal_flange_affine_ = xformUtils.transformPoseToEigenAffine3d(cart_goal_.des_pose_flange.pose);

    ROS_INFO("flange goal");
    display_affine(goal_flange_affine_);
    Vectorq7x1 q_start;
    q_start = get_jspace_start_();
    path_is_valid_ = cartTrajPlanner_.cartesian_path_planner(q_start, goal_flange_affine_, optimal_path_);

    if (path_is_valid_) {
        baxter_traj_streamer_.stuff_trajectory_right_arm(optimal_path_, des_trajectory_);
        computed_arrival_time_ = des_trajectory_.points.back().time_from_start.toSec();
        cart_result_.return_code = cartesian_planner::cart_moveResult::SUCCESS;
        cart_result_.computed_arrival_time = computed_arrival_time_;
        cart_move_as_.setSucceeded(cart_result_);
    } else {
        cart_result_.return_code = cartesian_planner::cart_moveResult::PATH_NOT_VALID;
        cart_result_.computed_arrival_time = -1.0;
        cart_move_as_.setSucceeded(cart_result_);
    }

    return path_is_valid_;
}
*/
bool ArmMotionInterface::plan_jspace_path_current_to_cart_gripper_pose()
{
  ROS_INFO("computing a jspace trajectory to right-arm gripper goal pose");

  goal_gripper_pose_ = cart_goal_.des_pose_gripper;
  xformUtils.printStampedPose(goal_gripper_pose_);
  goal_flange_affine_ = xform_gripper_pose_to_affine_flange_wrt_torso(goal_gripper_pose_);

  ROS_INFO("flange goal");
  display_affine(goal_flange_affine_);
  Vectorq7x1 q_start;
  q_start = get_jspace_start_();

  path_is_valid_ = cartTrajPlanner_.jspace_path_planner_to_affine_goal(q_start, goal_flange_affine_, optimal_path_);

  if (path_is_valid_)
  {
    baxter_traj_streamer_.stuff_trajectory_right_arm(optimal_path_, des_trajectory_);
    computed_arrival_time_ = des_trajectory_.points.back().time_from_start.toSec();
    cart_result_.return_code = cartesian_planner::cart_moveResult::SUCCESS;
    cart_result_.computed_arrival_time = computed_arrival_time_;
    cart_move_as_.setSucceeded(cart_result_);
  }
  else
  {
    cart_result_.return_code = cartesian_planner::cart_moveResult::PATH_NOT_VALID;
    cart_result_.computed_arrival_time = -1.0;
    cart_move_as_.setSucceeded(cart_result_);
  }

  return path_is_valid_;
}

bool ArmMotionInterface::plan_fine_path_current_to_goal_gripper_pose()
{
  return false;
}
/*
bool ArmMotionInterface::plan_fine_path_current_to_goal_flange_pose() {
    ROS_INFO("computing a hi-res cartesian trajectory to right-arm flange goal pose");

    goal_flange_affine_ = xformUtils.transformPoseToEigenAffine3d(cart_goal_.des_pose_flange.pose);

    ROS_INFO("flange goal");
    display_affine(goal_flange_affine_);
    Vectorq7x1 q_start;
    q_start = get_jspace_start_();
    path_is_valid_ = cartTrajPlanner_.fine_cartesian_path_planner(q_start, goal_flange_affine_, optimal_path_);

    if (path_is_valid_) {
        baxter_traj_streamer_.stuff_trajectory_right_arm(optimal_path_, des_trajectory_);
        computed_arrival_time_ = des_trajectory_.points.back().time_from_start.toSec();
        cart_result_.return_code = cartesian_planner::cart_moveResult::SUCCESS;
        cart_result_.computed_arrival_time = computed_arrival_time_;
        cart_move_as_.setSucceeded(cart_result_);
    } else {
        cart_result_.return_code = cartesian_planner::cart_moveResult::PATH_NOT_VALID;
        cart_result_.computed_arrival_time = -1.0;
        cart_move_as_.setSucceeded(cart_result_);
    }

    return path_is_valid_;
}
*/
bool ArmMotionInterface::plan_path_current_to_goal_dp_xyz()
{
  Eigen::Vector3d dp_vec;

  ROS_INFO("called plan_path_current_to_goal_dp_xyz");

  int ndim = cart_goal_.arm_dp.size();
  if (ndim != 3)
  {
    ROS_WARN("requested displacement, arm_dp, is wrong dimension");
    cart_result_.return_code = cartesian_planner::cart_moveResult::PATH_NOT_VALID;
    path_is_valid_ = false;
    return path_is_valid_;
  }
  for (int i = 0; i < 3; i++)
    dp_vec[i] = cart_goal_.arm_dp[i];
  ROS_INFO("requested dp = %f, %f, %f", dp_vec[0], dp_vec[1], dp_vec[2]);
  Vectorq7x1 q_start;
  q_start = get_jspace_start_();
  path_is_valid_ = plan_cartesian_delta_p(q_start, dp_vec);

  if (path_is_valid_)
  {
    baxter_traj_streamer_.stuff_trajectory_right_arm(optimal_path_, des_trajectory_);
    computed_arrival_time_ = des_trajectory_.points.back().time_from_start.toSec();
    cart_result_.return_code = cartesian_planner::cart_moveResult::SUCCESS;
    cart_result_.computed_arrival_time = computed_arrival_time_;
    cart_move_as_.setSucceeded(cart_result_);
  }
  else
  {
    cart_result_.return_code = cartesian_planner::cart_moveResult::PATH_NOT_VALID;
    cart_result_.computed_arrival_time = -1.0;
    cart_move_as_.setSucceeded(cart_result_);
  }

  return path_is_valid_;
}

bool ArmMotionInterface::refine_cartesian_path_soln(void)
{
  ROS_INFO("ArmMotionInterface: refining trajectory");
  if (path_is_valid_)
  {
    bool valid = cartTrajPlanner_.refine_cartesian_path_plan(optimal_path_);
    if (!valid)
      return false;

    baxter_traj_streamer_.stuff_trajectory_right_arm(optimal_path_, des_trajectory_);
    computed_arrival_time_ = des_trajectory_.points.back().time_from_start.toSec();
    cart_result_.return_code = cartesian_planner::cart_moveResult::SUCCESS;
    cart_result_.computed_arrival_time = computed_arrival_time_;
    cart_move_as_.setSucceeded(cart_result_);
  }
  else
  {
    cart_result_.return_code = cartesian_planner::cart_moveResult::PATH_NOT_VALID;
    cart_result_.computed_arrival_time = -1.0;
    cart_move_as_.setSucceeded(cart_result_);
    return false;
  }
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "baxter_rt_arm_cart_move_as");
  ros::NodeHandle nh;

  ROS_INFO("instantiating an ArmMotionInterface: ");
  ArmMotionInterface armMotionInterface(&nh);

  ROS_INFO("ready to start servicing cartesian-space goals");
  while (ros::ok())
  {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }

  return 0;
}
