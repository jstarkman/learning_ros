

#include <object_grabber/object_grabber2.h>
using namespace std;

ObjectGrabber::ObjectGrabber(ros::NodeHandle* nodehandle)
  : nh_(*nodehandle)
  , object_grabber_as_(nh_, "object_grabber_action_service", boost::bind(&ObjectGrabber::executeCB, this, _1), false)
  , cart_move_action_client_("cartMoveActionServer", true)
{
  ROS_INFO("in constructor of ObjectGrabber");

  if (!get_gripper_id())
  {
    ROS_WARN("no gripper ID; quitting");
    exit(0);
  }

  manip_properties_client_ =
      nh_.serviceClient<object_manipulation_properties::objectManipulationQuery>("object_manip_query_svc");

  ROS_INFO("waiting on manipulation-properties service: ");
  manip_properties_srv_.request.query_code = object_manipulation_properties::objectManipulationQueryRequest::TEST_PING;
  double t_test = 0;
  while (!manip_properties_client_.call(manip_properties_srv_))
  {
    ros::spinOnce();
    ros::Duration(0.5).sleep();
    t_test += 0.5;
    if (t_test > 2.0)
      ROS_WARN("can't connect to service object_manip_query_svc; is it running?");
  }

  gripper_client_ = nh_.serviceClient<generic_gripper_services::genericGripperInterface>("generic_gripper_svc");

  ROS_INFO("waiting on gripper interface service: ");
  gripper_srv_.request.cmd_code = generic_gripper_services::genericGripperInterfaceRequest::TEST_PING;
  t_test = 0;
  while (!gripper_client_.call(gripper_srv_))
  {
    ros::spinOnce();
    ros::Duration(0.5).sleep();
    t_test += 0.5;
    if (t_test > 2.0)
      ROS_WARN("can't connect to service generic_gripper_svc; is it running?");
  }

  /*
  ROS_INFO("waiting for cartesian-move action server: ");
  bool server_exists = false;
  while ((!server_exists)&&(ros::ok())) {
      server_exists = cart_move_action_client_.waitForServer(ros::Duration(0.5));
      ros::spinOnce();
      ros::Duration(0.5).sleep();
      ROS_INFO("retrying...");
  }
  ROS_INFO("connected to action server");
   */
  object_grabber_as_.start();
}

bool ObjectGrabber::get_gripper_id()
{
  if (nh_.getParam("gripper_ID", gripper_id_))
  {
    ROS_INFO("found gripper ID %d on parameter server", gripper_id_);
    if (gripper_id_ == GripperIdCodes::RETHINK_ELECTRIC_GRIPPER_RT)
    {
      ROS_INFO("Baxter electric gripper");
    }
    return true;
  }
  else
  {
    ROS_WARN("could not find gripper ID on parameter server");
    return false;
  }
}

void ObjectGrabber::cartMoveDoneCb_(const actionlib::SimpleClientGoalState& state,
                                    const cartesian_planner::cart_moveResultConstPtr& result)
{
  ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
  ROS_INFO("got return value= %d", result->return_code);
  cart_result_ = *result;
}

bool ObjectGrabber::get_default_grab_poses(int object_id, geometry_msgs::PoseStamped object_pose_stamped)
{
  manip_properties_srv_.request.gripper_ID = gripper_id_;
  manip_properties_srv_.request.object_ID = object_id;
  manip_properties_srv_.request.query_code =
      object_manipulation_properties::objectManipulationQueryRequest::GRASP_STRATEGY_OPTIONS_QUERY;
  manip_properties_client_.call(manip_properties_srv_);
  int n_grasp_strategy_options = manip_properties_srv_.response.grasp_strategy_options.size();
  ROS_INFO("there are %d grasp options for this gripper/object combo; choosing 1st option (default)",
           n_grasp_strategy_options);
  if (n_grasp_strategy_options < 1)
    return false;
  int grasp_option = manip_properties_srv_.response.grasp_strategy_options[0];
  ROS_INFO("chosen grasp strategy is code %d", grasp_option);

  manip_properties_srv_.request.grasp_option = grasp_option;
  manip_properties_srv_.request.query_code =
      object_manipulation_properties::objectManipulationQueryRequest::GET_GRASP_POSE_TRANSFORMS;
  manip_properties_client_.call(manip_properties_srv_);
  int n_grasp_pose_options = manip_properties_srv_.response.gripper_pose_options.size();
  if (n_grasp_pose_options < 1)
  {
    ROS_WARN("no pose options returned for gripper_ID %d and object_ID %d", gripper_id_, object_id);
    ;
    return false;
  }

  grasp_object_pose_wrt_gripper_ = manip_properties_srv_.response.gripper_pose_options[0];

  manip_properties_srv_.request.query_code =
      object_manipulation_properties::objectManipulationQueryRequest::APPROACH_STRATEGY_OPTIONS_QUERY;
  manip_properties_client_.call(manip_properties_srv_);
  int n_approach_strategy_options = manip_properties_srv_.response.grasp_strategy_options.size();
  ROS_INFO("there are %d approach options for this gripper/object combo; choosing 1st option (default)",
           n_approach_strategy_options);
  if (n_approach_strategy_options < 1)
    return false;
  int approach_option = manip_properties_srv_.response.grasp_strategy_options[0];
  ROS_INFO("chosen approach strategy is code %d", approach_option);

  manip_properties_srv_.request.grasp_option = approach_option;
  manip_properties_srv_.request.query_code =
      object_manipulation_properties::objectManipulationQueryRequest::GET_APPROACH_POSE_TRANSFORMS;
  manip_properties_client_.call(manip_properties_srv_);
  int n_approach_pose_options = manip_properties_srv_.response.gripper_pose_options.size();
  if (n_approach_pose_options < 1)
  {
    ROS_WARN("no approach pose options returned for gripper_ID %d and object_ID %d", gripper_id_, object_id);
    ROS_WARN("should not happen--apparent bug in manipulation properties service");
    return false;
  }
  approach_object_pose_wrt_gripper_ = manip_properties_srv_.response.gripper_pose_options[0];

  manip_properties_srv_.request.query_code =
      object_manipulation_properties::objectManipulationQueryRequest::DEPART_STRATEGY_OPTIONS_QUERY;
  manip_properties_client_.call(manip_properties_srv_);
  int n_depart_strategy_options = manip_properties_srv_.response.grasp_strategy_options.size();
  ROS_INFO("there are %d depart options for this gripper/object combo; choosing 1st option (default)",
           n_depart_strategy_options);
  if (n_depart_strategy_options < 1)
    return false;
  int depart_option = manip_properties_srv_.response.grasp_strategy_options[0];
  ROS_INFO("chosen depart strategy is code %d", depart_option);

  manip_properties_srv_.request.grasp_option = depart_option;

  manip_properties_srv_.request.query_code =
      object_manipulation_properties::objectManipulationQueryRequest::GET_DEPART_POSE_TRANSFORMS;
  manip_properties_client_.call(manip_properties_srv_);
  int n_depart_pose_options = manip_properties_srv_.response.gripper_pose_options.size();
  if (n_depart_pose_options < 1)
  {
    ROS_WARN("no depart pose options returned for gripper_ID %d and object_ID %d", gripper_id_, object_id);
    ROS_WARN("should not happen--apparent bug in manipulation properties service");
    return false;
  }
  depart_object_pose_wrt_gripper_ = manip_properties_srv_.response.gripper_pose_options[0];

  ROS_INFO("computing grasp stf: ");
  tf::StampedTransform object_stf =
      xformUtils.convert_poseStamped_to_stampedTransform(object_pose_stamped, "object_frame");
  geometry_msgs::PoseStamped object_wrt_gripper_ps;
  object_wrt_gripper_ps.pose = grasp_object_pose_wrt_gripper_;
  object_wrt_gripper_ps.header.frame_id = "generic_gripper_frame";
  tf::StampedTransform object_wrt_gripper_stf =
      xformUtils.convert_poseStamped_to_stampedTransform(object_wrt_gripper_ps, "object_frame");

  tf::StampedTransform gripper_wrt_object_stf = xformUtils.stamped_transform_inverse(object_wrt_gripper_stf);

  tf::StampedTransform gripper_stf;
  if (!xformUtils.multiply_stamped_tfs(object_stf, gripper_wrt_object_stf, gripper_stf))
  {
    ROS_WARN("illegal stamped-transform multiply");
    return false;
  }

  grasp_pose_ = xformUtils.get_pose_from_stamped_tf(gripper_stf);

  ROS_INFO("computing approach stf: ");
  object_wrt_gripper_ps.pose = approach_object_pose_wrt_gripper_;
  object_wrt_gripper_ps.header.frame_id = "generic_gripper_frame";
  object_wrt_gripper_stf = xformUtils.convert_poseStamped_to_stampedTransform(object_wrt_gripper_ps, "object_frame");

  gripper_wrt_object_stf = xformUtils.stamped_transform_inverse(object_wrt_gripper_stf);

  if (!xformUtils.multiply_stamped_tfs(object_stf, gripper_wrt_object_stf, gripper_stf))
  {
    ROS_WARN("illegal stamped-transform multiply");
    return false;
  }

  approach_pose_ = xformUtils.get_pose_from_stamped_tf(gripper_stf);

  object_wrt_gripper_ps.pose = depart_object_pose_wrt_gripper_;
  object_wrt_gripper_ps.header.frame_id = "generic_gripper_frame";
  object_wrt_gripper_stf = xformUtils.convert_poseStamped_to_stampedTransform(object_wrt_gripper_ps, "object_frame");

  gripper_wrt_object_stf = xformUtils.stamped_transform_inverse(object_wrt_gripper_stf);

  if (!xformUtils.multiply_stamped_tfs(object_stf, gripper_wrt_object_stf, gripper_stf))
  {
    ROS_WARN("illegal stamped-transform multiply");
    return false;
  }

  depart_pose_ = xformUtils.get_pose_from_stamped_tf(gripper_stf);

  return true;
}

bool ObjectGrabber::get_default_dropoff_poses(int object_id, geometry_msgs::PoseStamped object_dropoff_pose_stamped)
{
  manip_properties_srv_.request.gripper_ID = gripper_id_;
  manip_properties_srv_.request.object_ID = object_id;
  manip_properties_srv_.request.query_code =
      object_manipulation_properties::objectManipulationQueryRequest::GRASP_STRATEGY_OPTIONS_QUERY;
  manip_properties_client_.call(manip_properties_srv_);
  int n_grasp_strategy_options = manip_properties_srv_.response.grasp_strategy_options.size();
  ROS_INFO("there are %d grasp options for this gripper/object combo; choosing 1st option (default)",
           n_grasp_strategy_options);
  if (n_grasp_strategy_options < 1)
    return false;
  int grasp_option = manip_properties_srv_.response.grasp_strategy_options[0];
  ROS_INFO("chosen grasp strategy is code %d", grasp_option);

  manip_properties_srv_.request.grasp_option = grasp_option;
  manip_properties_srv_.request.query_code =
      object_manipulation_properties::objectManipulationQueryRequest::GET_GRASP_POSE_TRANSFORMS;
  manip_properties_client_.call(manip_properties_srv_);
  int n_grasp_pose_options = manip_properties_srv_.response.gripper_pose_options.size();
  if (n_grasp_pose_options < 1)
  {
    ROS_WARN("no pose options returned for gripper_ID %d and object_ID %d", gripper_id_, object_id);
    ;
    return false;
  }

  grasp_object_pose_wrt_gripper_ = manip_properties_srv_.response.gripper_pose_options[0];
  ROS_INFO("default grasped pose of object w/rt gripper: ");
  xformUtils.printPose(grasp_object_pose_wrt_gripper_);

  manip_properties_srv_.request.query_code =
      object_manipulation_properties::objectManipulationQueryRequest::APPROACH_STRATEGY_OPTIONS_QUERY;
  manip_properties_client_.call(manip_properties_srv_);
  int n_approach_strategy_options = manip_properties_srv_.response.grasp_strategy_options.size();
  ROS_INFO("there are %d approach options for this gripper/object combo; choosing 1st option (default)",
           n_approach_strategy_options);
  if (n_approach_strategy_options < 1)
    return false;
  int approach_option = manip_properties_srv_.response.grasp_strategy_options[0];
  ROS_INFO("chosen approach strategy is code %d", approach_option);

  manip_properties_srv_.request.grasp_option = approach_option;
  manip_properties_srv_.request.query_code =
      object_manipulation_properties::objectManipulationQueryRequest::GET_APPROACH_POSE_TRANSFORMS;
  manip_properties_client_.call(manip_properties_srv_);
  int n_approach_pose_options = manip_properties_srv_.response.gripper_pose_options.size();
  if (n_approach_pose_options < 1)
  {
    ROS_WARN("no approach pose options returned for gripper_ID %d and object_ID %d", gripper_id_, object_id);
    ROS_WARN("should not happen--apparent bug in manipulation properties service");
    return false;
  }
  approach_object_pose_wrt_gripper_ = manip_properties_srv_.response.gripper_pose_options[0];
  ROS_INFO("default approach pose, expressed as pose of object w/rt gripper at approach: ");
  xformUtils.printPose(approach_object_pose_wrt_gripper_);

  manip_properties_srv_.request.query_code =
      object_manipulation_properties::objectManipulationQueryRequest::DEPART_STRATEGY_OPTIONS_QUERY;
  manip_properties_client_.call(manip_properties_srv_);
  int n_depart_strategy_options = manip_properties_srv_.response.grasp_strategy_options.size();
  ROS_INFO("there are %d depart options for this gripper/object combo; choosing 1st option (default)",
           n_depart_strategy_options);
  if (n_depart_strategy_options < 1)
    return false;
  int depart_option = manip_properties_srv_.response.grasp_strategy_options[0];
  ROS_INFO("chosen depart strategy is code %d", depart_option);

  manip_properties_srv_.request.grasp_option = depart_option;

  manip_properties_srv_.request.query_code =
      object_manipulation_properties::objectManipulationQueryRequest::GET_DEPART_POSE_TRANSFORMS;
  manip_properties_client_.call(manip_properties_srv_);
  int n_depart_pose_options = manip_properties_srv_.response.gripper_pose_options.size();
  if (n_depart_pose_options < 1)
  {
    ROS_WARN("no depart pose options returned for gripper_ID %d and object_ID %d", gripper_id_, object_id);
    ROS_WARN("should not happen--apparent bug in manipulation properties service");
    return false;
  }
  depart_object_pose_wrt_gripper_ = manip_properties_srv_.response.gripper_pose_options[0];

  ROS_INFO("default depart pose, expressed as original pose of object w/rt gripper at depart: ");
  xformUtils.printPose(depart_object_pose_wrt_gripper_);

  ROS_INFO("computing dropoff stf: ");
  tf::StampedTransform object_stf =
      xformUtils.convert_poseStamped_to_stampedTransform(object_dropoff_pose_stamped, "object_frame");
  geometry_msgs::PoseStamped object_wrt_gripper_ps;
  object_wrt_gripper_ps.pose = grasp_object_pose_wrt_gripper_;
  object_wrt_gripper_ps.header.frame_id = "generic_gripper_frame";
  tf::StampedTransform object_wrt_gripper_stf =
      xformUtils.convert_poseStamped_to_stampedTransform(object_wrt_gripper_ps, "object_frame");
  ROS_INFO("object w/rt gripper stf: ");
  xformUtils.printStampedTf(object_wrt_gripper_stf);
  tf::StampedTransform gripper_wrt_object_stf = xformUtils.stamped_transform_inverse(object_wrt_gripper_stf);
  ROS_INFO("gripper w/rt object stf: ");
  xformUtils.printStampedTf(gripper_wrt_object_stf);

  tf::StampedTransform gripper_dropoff_stf;
  if (!xformUtils.multiply_stamped_tfs(object_stf, gripper_wrt_object_stf, gripper_dropoff_stf))
  {
    ROS_WARN("illegal stamped-transform multiply");
    return false;
  }

  dropoff_pose_ = xformUtils.get_pose_from_stamped_tf(gripper_dropoff_stf);
  ROS_INFO("computed gripper pose at dropoff location: ");
  xformUtils.printStampedPose(dropoff_pose_);

  ROS_INFO("computing approach stf: ");
  object_wrt_gripper_ps.pose = approach_object_pose_wrt_gripper_;
  object_wrt_gripper_ps.header.frame_id = "generic_gripper_frame";
  object_wrt_gripper_stf = xformUtils.convert_poseStamped_to_stampedTransform(object_wrt_gripper_ps, "object_frame");
  ROS_INFO("object w/rt gripper stf: ");
  xformUtils.printStampedTf(object_wrt_gripper_stf);
  gripper_wrt_object_stf = xformUtils.stamped_transform_inverse(object_wrt_gripper_stf);
  ROS_INFO("gripper w/rt object stf: ");
  xformUtils.printStampedTf(gripper_wrt_object_stf);
  tf::StampedTransform gripper_approach_stf;

  if (!xformUtils.multiply_stamped_tfs(object_stf, gripper_wrt_object_stf, gripper_approach_stf))
  {
    ROS_WARN("illegal stamped-transform multiply");
    return false;
  }

  approach_pose_ = xformUtils.get_pose_from_stamped_tf(gripper_approach_stf);
  ROS_INFO("computed gripper pose at approach location: ");
  xformUtils.printStampedPose(approach_pose_);

  ROS_INFO("computing depart stf: ");
  object_wrt_gripper_ps.pose = depart_object_pose_wrt_gripper_;
  object_wrt_gripper_ps.header.frame_id = "generic_gripper_frame";
  object_wrt_gripper_stf = xformUtils.convert_poseStamped_to_stampedTransform(object_wrt_gripper_ps, "object_frame");
  ROS_INFO("object w/rt gripper stf: ");
  xformUtils.printStampedTf(object_wrt_gripper_stf);
  gripper_wrt_object_stf = xformUtils.stamped_transform_inverse(object_wrt_gripper_stf);
  ROS_INFO("gripper w/rt object stf: ");
  xformUtils.printStampedTf(gripper_wrt_object_stf);

  tf::StampedTransform gripper_depart_stf;
  if (!xformUtils.multiply_stamped_tfs(object_stf, gripper_wrt_object_stf, gripper_depart_stf))
  {
    ROS_WARN("illegal stamped-transform multiply");
    return false;
  }

  depart_pose_ = xformUtils.get_pose_from_stamped_tf(gripper_depart_stf);
  ROS_INFO("computed gripper pose at depart location: ");
  xformUtils.printStampedPose(depart_pose_);

  return true;
}

int ObjectGrabber::grab_object(int object_id, geometry_msgs::PoseStamped object_pose_stamped)
{
  int rtn_val;
  bool success;
  if (!get_default_grab_poses(object_id, object_pose_stamped))
  {
    ROS_WARN("no valid grasp strategy; giving up");
    return object_grabber::object_grabberResult::NO_KNOWN_GRASP_OPTIONS_THIS_GRIPPER_AND_OBJECT;
  }

  ROS_WARN("prepare gripper state to anticipate grasp...");
  gripper_srv_.request.cmd_code = generic_gripper_services::genericGripperInterfaceRequest::RELEASE;
  gripper_client_.call(gripper_srv_);
  success = gripper_srv_.response.success;
  if (success)
  {
    ROS_INFO("gripper responded w/ success");
  }
  else
  {
    ROS_WARN("responded with failure");
  }

  ROS_WARN("object-grabber as planning joint-space move to approach pose");

  rtn_val = arm_motion_commander_.plan_jspace_path_current_to_cart_gripper_pose(approach_pose_);
  if (rtn_val != cartesian_planner::cart_moveResult::SUCCESS)
    return rtn_val;

  ROS_INFO("executing plan: ");
  rtn_val = arm_motion_commander_.execute_planned_path();
  if (rtn_val != cartesian_planner::cart_moveResult::SUCCESS)
    return rtn_val;

  ROS_INFO("planning motion of gripper to grasp pose at: ");
  xformUtils.printPose(grasp_pose_);
  rtn_val = arm_motion_commander_.plan_path_current_to_goal_gripper_pose(grasp_pose_);
  if (rtn_val != cartesian_planner::cart_moveResult::SUCCESS)
    return rtn_val;
  ROS_INFO("executing plan: ");
  rtn_val = arm_motion_commander_.execute_planned_path();
  ROS_WARN("poised to grasp object; invoke gripper grasp action here ...");

  gripper_srv_.request.cmd_code = generic_gripper_services::genericGripperInterfaceRequest::GRASP;
  gripper_client_.call(gripper_srv_);
  success = gripper_srv_.response.success;
  ros::Duration(1.0).sleep();
  if (success)
  {
    ROS_INFO("gripper responded w/ success");
  }
  else
  {
    ROS_WARN("responded with failure");
  }

  ROS_INFO("planning motion of gripper to depart pose at: ");
  xformUtils.printPose(depart_pose_);
  rtn_val = arm_motion_commander_.plan_path_current_to_goal_gripper_pose(depart_pose_);
  if (rtn_val != cartesian_planner::cart_moveResult::SUCCESS)
    return rtn_val;
  ROS_INFO("performing motion");
  rtn_val = arm_motion_commander_.execute_planned_path();

  return rtn_val;
}

int ObjectGrabber::straddle_object(int object_id, geometry_msgs::PoseStamped object_pose_stamped)
{
  int rtn_val;
  bool success;
  if (!get_default_grab_poses(object_id, object_pose_stamped))
  {
    ROS_WARN("no valid grasp strategy; giving up");
    return object_grabber::object_grabberResult::NO_KNOWN_GRASP_OPTIONS_THIS_GRIPPER_AND_OBJECT;
  }

  ROS_WARN("prepare gripper state to anticipate grasp...");
  gripper_srv_.request.cmd_code = generic_gripper_services::genericGripperInterfaceRequest::RELEASE;
  gripper_client_.call(gripper_srv_);
  success = gripper_srv_.response.success;
  if (success)
  {
    ROS_INFO("gripper responded w/ success");
  }
  else
  {
    ROS_WARN("responded with failure");
  }

  ROS_WARN("object-grabber action server is planning joint-space move to approach pose");

  rtn_val = arm_motion_commander_.plan_jspace_path_current_to_cart_gripper_pose(approach_pose_);
  if (rtn_val != cartesian_planner::cart_moveResult::SUCCESS)
    return rtn_val;

  ROS_INFO("executing plan: ");
  rtn_val = arm_motion_commander_.execute_planned_path();
  if (rtn_val != cartesian_planner::cart_moveResult::SUCCESS)
    return rtn_val;

  ROS_INFO("planning motion of gripper to grasp pose at: ");
  xformUtils.printPose(grasp_pose_);
  rtn_val = arm_motion_commander_.plan_path_current_to_goal_gripper_pose(grasp_pose_);
  if (rtn_val != cartesian_planner::cart_moveResult::SUCCESS)
    return rtn_val;
  ROS_INFO("executing plan: ");
  rtn_val = arm_motion_commander_.execute_planned_path();
  ROS_WARN("concluded motion; gripper should be in grasp pose");

  return rtn_val;
}

int ObjectGrabber::dropoff_object(int object_id, geometry_msgs::PoseStamped desired_object_pose_stamped)
{
  int rtn_val;
  bool success;

  if (!get_default_dropoff_poses(object_id, desired_object_pose_stamped))
  {
    ROS_WARN("no valid grasp strategy; giving up");
    return object_grabber::object_grabberResult::NO_KNOWN_GRASP_OPTIONS_THIS_GRIPPER_AND_OBJECT;
  }

  ROS_INFO("planning move to approach pose");
  ROS_INFO("planning motion of gripper to approach pose at: ");
  xformUtils.printPose(approach_pose_);
  rtn_val = arm_motion_commander_.plan_path_current_to_goal_gripper_pose(approach_pose_);
  if (rtn_val != cartesian_planner::cart_moveResult::SUCCESS)
    return rtn_val;

  ROS_INFO("executing plan: ");
  rtn_val = arm_motion_commander_.execute_planned_path();
  if (rtn_val != cartesian_planner::cart_moveResult::SUCCESS)
    return rtn_val;

  ROS_INFO("planning motion of gripper to dropoff pose at: ");
  xformUtils.printPose(dropoff_pose_);
  rtn_val = arm_motion_commander_.plan_path_current_to_goal_gripper_pose(dropoff_pose_);
  if (rtn_val != cartesian_planner::cart_moveResult::SUCCESS)
    return rtn_val;
  ROS_INFO("executing plan: ");
  rtn_val = arm_motion_commander_.execute_planned_path();
  ROS_INFO("poised to release object;  invoke gripper release action here ...");

  gripper_srv_.request.cmd_code = generic_gripper_services::genericGripperInterfaceRequest::RELEASE;
  gripper_client_.call(gripper_srv_);
  success = gripper_srv_.response.success;
  if (success)
  {
    ROS_INFO("gripper responded w/ success");
  }
  else
  {
    ROS_WARN("responded with failure");
  }
  ros::Duration(1.0).sleep();

  ROS_INFO("planning motion of gripper to depart pose at: ");
  xformUtils.printPose(depart_pose_);
  rtn_val = arm_motion_commander_.plan_path_current_to_goal_gripper_pose(depart_pose_);
  if (rtn_val != cartesian_planner::cart_moveResult::SUCCESS)
    return rtn_val;
  ROS_INFO("performing motion");
  rtn_val = arm_motion_commander_.execute_planned_path();

  return rtn_val;
}

void ObjectGrabber::executeCB(
    const actionlib::SimpleActionServer<object_grabber::object_grabberAction>::GoalConstPtr& goal)
{
  int action_code = goal->action_code;
  ROS_INFO("got action code %d", action_code);
  int object_grabber_rtn_code, rtn_val;
  int object_id;
  int grasp_option;

  int approach_strategy, lift_object_strategy, dropoff_strategy, dropoff_withdraw_strategy;
  bool have_default_grasp_plan;
  switch (action_code)
  {
    case object_grabber::object_grabberGoal::TEST_CODE:
      ROS_INFO("got test ping");
      arm_motion_commander_.send_test_goal();
      grab_result_.return_code = object_grabber::object_grabberResult::SUCCESS;
      object_grabber_as_.setSucceeded(grab_result_);
      break;

    case object_grabber::object_grabberGoal::MOVE_TO_WAITING_POSE:
      ROS_INFO("planning move to waiting pose");
      rtn_val = arm_motion_commander_.plan_move_to_waiting_pose();
      ROS_INFO("commanding plan execution");
      rtn_val = arm_motion_commander_.execute_planned_path();
      grab_result_.return_code = rtn_val;
      object_grabber_as_.setSucceeded(grab_result_);

      break;

    case object_grabber::object_grabberGoal::GRAB_OBJECT:
      ROS_INFO("GRAB_OBJECT: ");
      object_id = goal->object_id;
      grasp_option = goal->grasp_option;
      object_pose_stamped_ = goal->object_frame;

      if (grasp_option != object_grabber::object_grabberGoal::DEFAULT_GRASP_STRATEGY)
      {
        ROS_WARN("grasp strategy %d not implemented yet; using default strategy", grasp_option);
      }
      rtn_val = grab_object(object_id, object_pose_stamped_);
      ROS_INFO("grasp attempt concluded");
      grab_result_.return_code = rtn_val;
      object_grabber_as_.setSucceeded(grab_result_);
      break;

    case object_grabber::object_grabberGoal::STRADDLE_OBJECT:
      ROS_INFO("STRADDLE_OBJECT: ");
      object_id = goal->object_id;
      grasp_option = goal->grasp_option;
      object_pose_stamped_ = goal->object_frame;

      if (grasp_option != object_grabber::object_grabberGoal::DEFAULT_GRASP_STRATEGY)
      {
        ROS_WARN("grasp strategy %d not implemented yet; using default strategy", grasp_option);
      }
      rtn_val = straddle_object(object_id, object_pose_stamped_);
      ROS_INFO("straddle attempt concluded");
      grab_result_.return_code = rtn_val;
      object_grabber_as_.setSucceeded(grab_result_);
      break;

    case object_grabber::object_grabberGoal::DROPOFF_OBJECT:
      ROS_INFO("DROPOFF_OBJECT: ");
      object_id = goal->object_id;
      grasp_option = goal->grasp_option;
      object_pose_stamped_ = goal->object_frame;

      if (grasp_option != object_grabber::object_grabberGoal::DEFAULT_GRASP_STRATEGY)
      {
        ROS_WARN("grasp strategy %d not implemented yet; using default strategy", grasp_option);
      }
      rtn_val = dropoff_object(object_id, object_pose_stamped_);
      ROS_INFO("dropoff attempt concluded");
      grab_result_.return_code = rtn_val;
      object_grabber_as_.setSucceeded(grab_result_);
      break;

    case object_grabber::object_grabberGoal::CART_MOVE_CURRENT_TO_CART_GOAL:
      ROS_INFO("planning Cartesian move from current pose to goal pose");
      goal_pose_stamped_ = goal->object_frame;
      rtn_val = arm_motion_commander_.plan_path_current_to_goal_gripper_pose(goal_pose_stamped_);
      if (rtn_val != cartesian_planner::cart_moveResult::SUCCESS)
      {
        grab_result_.return_code = object_grabber::object_grabberResult::FAILED_CANNOT_REACH;
        object_grabber_as_.setSucceeded(grab_result_);
      }
      else
      {
        ROS_INFO("executing plan: ");
        rtn_val = arm_motion_commander_.execute_planned_path();
        grab_result_.return_code = rtn_val;
        object_grabber_as_.setSucceeded(grab_result_);
      }
      break;

    default:
      ROS_WARN("this object ID is not implemented");
      grab_result_.return_code = object_grabber::object_grabberResult::ACTION_CODE_UNKNOWN;
      object_grabber_as_.setAborted(grab_result_);
  }
}
