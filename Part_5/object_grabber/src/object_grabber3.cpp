

#include <object_grabber/object_grabber3.h>
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

  ROS_INFO("computing grasp stf: ");
  tf::StampedTransform object_stf =
      xformUtils.convert_poseStamped_to_stampedTransform(object_pose_stamped, "object_frame");
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

  tf::StampedTransform gripper_stf;
  if (!xformUtils.multiply_stamped_tfs(object_stf, gripper_wrt_object_stf, gripper_stf))
  {
    ROS_WARN("illegal stamped-transform multiply");
    return false;
  }

  grasp_pose_ = xformUtils.get_pose_from_stamped_tf(gripper_stf);
  ROS_INFO("computed gripper pose at grasp location: ");
  xformUtils.printStampedPose(grasp_pose_);

  ROS_INFO("computing approach stf: ");
  object_wrt_gripper_ps.pose = approach_object_pose_wrt_gripper_;
  object_wrt_gripper_ps.header.frame_id = "generic_gripper_frame";
  object_wrt_gripper_stf = xformUtils.convert_poseStamped_to_stampedTransform(object_wrt_gripper_ps, "object_frame");
  ROS_INFO("object w/rt gripper stf: ");
  xformUtils.printStampedTf(object_wrt_gripper_stf);
  gripper_wrt_object_stf = xformUtils.stamped_transform_inverse(object_wrt_gripper_stf);
  ROS_INFO("gripper w/rt object stf: ");
  xformUtils.printStampedTf(gripper_wrt_object_stf);

  if (!xformUtils.multiply_stamped_tfs(object_stf, gripper_wrt_object_stf, gripper_stf))
  {
    ROS_WARN("illegal stamped-transform multiply");
    return false;
  }

  approach_pose_ = xformUtils.get_pose_from_stamped_tf(gripper_stf);
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

  if (!xformUtils.multiply_stamped_tfs(object_stf, gripper_wrt_object_stf, gripper_stf))
  {
    ROS_WARN("illegal stamped-transform multiply");
    return false;
  }

  depart_pose_ = xformUtils.get_pose_from_stamped_tf(gripper_stf);
  ROS_INFO("computed gripper pose at depart location: ");
  xformUtils.printStampedPose(depart_pose_);

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
    return object_grabber::object_grabber3Result::NO_KNOWN_GRASP_OPTIONS_THIS_GRIPPER_AND_OBJECT;
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

  ROS_INFO("planning joint-space move to approach pose");

  ROS_INFO("planning motion of gripper to approach pose at: ");
  xformUtils.printPose(approach_pose_);
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

int ObjectGrabber::dropoff_object(int object_id, geometry_msgs::PoseStamped desired_object_pose_stamped)
{
  int rtn_val;
  bool success;

  if (!get_default_dropoff_poses(object_id, desired_object_pose_stamped))
  {
    ROS_WARN("no valid grasp strategy; giving up");
    return object_grabber::object_grabber3Result::NO_KNOWN_GRASP_OPTIONS_THIS_GRIPPER_AND_OBJECT;
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
    const actionlib::SimpleActionServer<object_grabber::object_grabber3Action>::GoalConstPtr& goal)
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
    case object_grabber::object_grabber3Goal::TEST_CODE:
      ROS_INFO("got test ping");
      arm_motion_commander_.send_test_goal();
      grab_result_.return_code = object_grabber::object_grabber3Result::SUCCESS;
      object_grabber_as_.setSucceeded(grab_result_);
      break;

    case object_grabber::object_grabber3Goal::MOVE_TO_WAITING_POSE:
      ROS_INFO("planning move to waiting pose");
      rtn_val = arm_motion_commander_.plan_move_to_pre_pose();
      ROS_INFO("commanding plan execution");
      rtn_val = arm_motion_commander_.execute_planned_path();
      grab_result_.return_code = rtn_val;
      object_grabber_as_.setSucceeded(grab_result_);

      break;

    case object_grabber::object_grabber3Goal::GRAB_OBJECT:
      ROS_INFO("GRAB_OBJECT: ");
      object_id = goal->object_id;
      grasp_option = goal->grasp_option;
      object_pose_stamped_ = goal->object_frame;

      if (grasp_option != object_grabber::object_grabber3Goal::DEFAULT_GRASP_STRATEGY)
      {
        ROS_WARN("grasp strategy %d not implemented yet; using default strategy", grasp_option);
      }
      rtn_val = grab_object(object_id, object_pose_stamped_);
      ROS_INFO("grasp attempt concluded");
      grab_result_.return_code = rtn_val;
      object_grabber_as_.setSucceeded(grab_result_);
      break;
    case object_grabber::object_grabber3Goal::DROPOFF_OBJECT:
      ROS_INFO("DROPOFF_OBJECT: ");
      object_id = goal->object_id;
      grasp_option = goal->grasp_option;
      object_pose_stamped_ = goal->object_frame;

      if (grasp_option != object_grabber::object_grabber3Goal::DEFAULT_GRASP_STRATEGY)
      {
        ROS_WARN("grasp strategy %d not implemented yet; using default strategy", grasp_option);
      }
      rtn_val = dropoff_object(object_id, object_pose_stamped_);
      ROS_INFO("dropoff attempt concluded");
      grab_result_.return_code = rtn_val;
      object_grabber_as_.setSucceeded(grab_result_);
      break;

    default:
      ROS_WARN("this object ID is not implemented");
      grab_result_.return_code = object_grabber::object_grabber3Result::ACTION_CODE_UNKNOWN;
      object_grabber_as_.setAborted(grab_result_);
  }
}

/*
int ObjectGrabber::move_flange_to(geometry_msgs::PoseStamped des_flange_pose_wrt_torso) {
    std::vector<Vectorq7x1> q_solns;

    int planner_rtn_code, execute_return_code;
    planner_rtn_code = armMotionCommander.rt_arm_plan_path_current_to_goal_flange_pose(des_flange_pose_wrt_torso);


    if (planner_rtn_code != cartesian_planner::baxter_cart_moveResult::SUCCESS) {
        ROS_WARN("cannot move to specified approach pose");
        return object_grabber::object_grabberResult::FAILED_CANNOT_REACH_POSE_CARTESIAN_MOVE;
    }


    ROS_INFO("sending command to execute planned path to approach pose:");
    execute_return_code = armMotionCommander.rt_arm_execute_planned_path();

    return object_grabber::object_grabberResult::SUCCESS;
}

int ObjectGrabber::jspace_move_flange_to(geometry_msgs::PoseStamped des_flange_pose_wrt_torso) {
    std::vector<Vectorq7x1> q_solns;

    int planner_rtn_code, execute_return_code;
    planner_rtn_code = armMotionCommander.rt_arm_plan_jspace_path_current_to_flange_pose(des_flange_pose_wrt_torso);

    if (planner_rtn_code != cartesian_planner::baxter_cart_moveResult::SUCCESS) {
        ROS_WARN("cannot move to specified approach pose");
        return object_grabber::object_grabberResult::FAILED_CANNOT_REACH_JSPACE_MOVE;
    }


    ROS_INFO("sending command to execute planned path to approach pose:");
    execute_return_code = armMotionCommander.rt_arm_execute_planned_path();

    return object_grabber::object_grabberResult::SUCCESS;
}

int ObjectGrabber::jspace_move_to_pre_pose(void) {

    int planner_rtn_code, execute_return_code;
    planner_rtn_code = armMotionCommander.plan_move_to_pre_pose();


    if (planner_rtn_code != cartesian_planner::baxter_cart_moveResult::SUCCESS) {
        ROS_WARN("cannot move to pre-pose");
        return object_grabber::object_grabberResult::FAILED_CANNOT_MOVE_TO_PRE_POSE;
    }


    ROS_INFO("sending command to execute planned path to pre-pose:");
    execute_return_code = armMotionCommander.rt_arm_execute_planned_path();

    return object_grabber::object_grabberResult::SUCCESS;
}



int ObjectGrabber::fine_move_flange_to(geometry_msgs::PoseStamped des_flange_pose_wrt_torso) {
    int planner_rtn_code, execute_return_code;
    ROS_INFO("planning hi-res move");
    planner_rtn_code = armMotionCommander.rt_arm_plan_fine_path_current_to_goal_flange_pose(des_flange_pose_wrt_torso);
    if (planner_rtn_code != cartesian_planner::baxter_cart_moveResult::SUCCESS) {
        ROS_WARN("desired fine motion is not feasible");
        return object_grabber::object_grabberResult::FAILED_CANNOT_MOVE_CARTESIAN_FINE;
    }

    ROS_INFO("sending command to execute planned hi-res path:");
    execute_return_code = armMotionCommander.rt_arm_execute_planned_path();

    return object_grabber::object_grabberResult::SUCCESS;
}




int ObjectGrabber::open_gripper(double open_val_test) {

    ROS_INFO("opening gripper");
    double dt = 0.01;
    double stopwatch = 0.0;

    baxterGripper.right_gripper_open();
    ros::spinOnce();
    ROS_INFO("right gripper pos = %f; waiting for pos>%f", baxterGripper.get_right_gripper_pos(), open_val_test);
    while ((baxterGripper.get_right_gripper_pos() < open_val_test)&&(stopwatch < GRIPPER_TIMEOUT)) {
        baxterGripper.right_gripper_open();
        ros::spinOnce();
        stopwatch += dt;
        ROS_INFO("gripper pos = %f", baxterGripper.get_right_gripper_pos());
        ros::Duration(dt).sleep();
    }
    if (baxterGripper.get_right_gripper_pos() > open_val_test) {
        ROS_INFO("gripper is open to > %f", open_val_test);
        return object_grabber::object_grabberResult::GRIPPER_IS_OPEN;
    } else {
        ROS_WARN("timeout expired without opening gripper");
        return object_grabber::object_grabberResult::GRIPPER_FAILURE;
    }
}




int ObjectGrabber::close_gripper(double close_val_test) {
    ROS_INFO("closing gripper");
    double dt = 0.01;
    double stopwatch = 0.0;
    baxterGripper.right_gripper_close();
    ros::spinOnce();


    while ((baxterGripper.get_right_gripper_pos() > close_val_test)&&(stopwatch < GRIPPER_TIMEOUT)) {
        stopwatch += dt;
        baxterGripper.right_gripper_close();
        ros::spinOnce();

        ros::Duration(dt).sleep();
    }

    if (baxterGripper.get_right_gripper_pos() < close_val_test) {
        ROS_INFO("gripper is closed to < %f", close_val_test);
        return object_grabber::object_grabberResult::GRIPPER_IS_CLOSED;
    } else {
        ROS_WARN("timeout expired without closing gripper");
        ROS_INFO("gripper position is %f", baxterGripper.get_right_gripper_pos());
        return object_grabber::object_grabberResult::GRIPPER_FAILURE;
    }
}



geometry_msgs::PoseStamped ObjectGrabber::convert_pose_to_system_ref_frame(geometry_msgs::PoseStamped pose_stamped) {

    geometry_msgs::PoseStamped pose_stamped_wrt_sys_ref_frame;
    bool valid_tf = false;
    while (!valid_tf) {
        try {
            tfListener.transformPose("system_ref_frame", pose_stamped, pose_stamped_wrt_sys_ref_frame);
            valid_tf = true;
        } catch (tf::TransformException &ex) {
            ROS_WARN("transform not valid...retrying");
            valid_tf = false;
            ros::Duration(0.1).sleep();
        }
    }
    ROS_INFO("desired pose_stamped_wrt_sys_ref_frame_ origin: %f, %f, %f",
pose_stamped_wrt_sys_ref_frame.pose.position.x,
            pose_stamped_wrt_sys_ref_frame.pose.position.y, pose_stamped_wrt_sys_ref_frame.pose.position.z);
    return pose_stamped_wrt_sys_ref_frame;
}





Eigen::Affine3d ObjectGrabber::block_grasp_transform(Eigen::Affine3d block_affine) {
    Eigen::Affine3d gripper_affine;



    Eigen::Vector3d x_axis, y_axis, z_axis;
    Eigen::Matrix3d R_object, R_gripper;
    R_object = block_affine.linear();
    x_axis = R_object.col(0);
    z_axis = -R_object.col(2);
    y_axis = z_axis.cross(x_axis);
    R_gripper.col(0) = x_axis;
    R_gripper.col(1) = y_axis;
    R_gripper.col(2) = z_axis;


    gripper_affine.linear() = R_gripper;
    gripper_affine.translation() = block_affine.translation();
    return gripper_affine;
}



geometry_msgs::PoseStamped ObjectGrabber::block_grasp_transform(geometry_msgs::PoseStamped block_pose) {
    Eigen::Affine3d block_affine, gripper_affine;
    geometry_msgs::PoseStamped gripper_pose;
    block_affine = xformUtils.transformPoseToEigenAffine3d(block_pose.pose);
    gripper_affine = block_grasp_transform(block_affine);
    gripper_pose.header = block_pose.header;
    gripper_pose.pose = xformUtils.transformEigenAffine3dToPose(gripper_affine);
    return gripper_pose;
}




geometry_msgs::PoseStamped ObjectGrabber::block_to_flange_grasp_transform(geometry_msgs::PoseStamped block_pose) {
    geometry_msgs::PoseStamped flange_pose_for_block_grasp;
    Eigen::Affine3d block_affine, gripper_affine, flange_affine;
    block_affine = block_affine = xformUtils.transformPoseToEigenAffine3d(block_pose.pose);
    gripper_affine = block_grasp_transform(block_affine);
    flange_affine = gripper_affine * a_right_gripper_frame_wrt_flange_.inverse();
    flange_pose_for_block_grasp.header = block_pose.header;
    flange_pose_for_block_grasp.pose = xformUtils.transformEigenAffine3dToPose(flange_affine);
    return flange_pose_for_block_grasp;
}





geometry_msgs::PoseStamped ObjectGrabber::object_to_flange_grasp_transform(int object_id, geometry_msgs::PoseStamped
object_pose) {
    geometry_msgs::PoseStamped flange_pose_for_object_grasp;
    Eigen::Affine3d object_affine, gripper_affine, flange_affine;
    Eigen::Vector3d origin;


    objectManipulationProperties.get_object_info(object_id_, grasp_transform_,
            approach_dist_, gripper_test_val_);


    object_affine = xformUtils.transformPoseToEigenAffine3d(object_pose.pose);






    gripper_affine = object_affine * grasp_transform_.inverse();
    origin = gripper_affine.translation();
    ROS_INFO("gripper origin set to: %f, %f, %f", origin[0], origin[1], origin[2]);
    origin = a_right_gripper_frame_wrt_flange_.translation();
    ROS_INFO("gripperframe origin wrt flange: %f %f %f", origin[0], origin[1], origin[2]);


    flange_affine = gripper_affine * a_right_gripper_frame_wrt_flange_.inverse();
    origin = flange_affine.translation();
    ROS_INFO("flange origin wrt torso: %f %f %f", origin[0], origin[1], origin[2]);


    flange_pose_for_object_grasp.header = object_pose.header;
    flange_pose_for_object_grasp.pose = xformUtils.transformEigenAffine3dToPose(flange_affine);
    return flange_pose_for_object_grasp;
}









int ObjectGrabber::grasp_approach_tool_z_axis(geometry_msgs::PoseStamped des_flange_grasp_pose,
        double approach_dist, double gripper_close_test_val) {
    int rtn_val;

    geometry_msgs::PoseStamped des_flange_approach_pose, des_flange_depart_pose;
    Eigen::Affine3d flange_grasp_affine, flange_depart_affine;

    flange_grasp_affine = xformUtils.transformPoseToEigenAffine3d(des_flange_grasp_pose.pose);


    Eigen::Vector3d approach_origin, flange_grasp_origin;
    flange_grasp_origin = flange_grasp_affine.translation();
    Eigen::Vector3d toolflange_z_axis;
    Eigen::Matrix3d R;
    R = flange_grasp_affine.linear();
    toolflange_z_axis = R.col(2);
    cout << "toolflange_z_axis: " << toolflange_z_axis.transpose() << endl;

    a_flange_grasp_ = flange_grasp_affine;

    a_flange_approach_ = a_flange_grasp_;

    a_flange_approach_.translation() = a_flange_grasp_.translation() - toolflange_z_axis*approach_dist;
    a_flange_depart_ = a_flange_approach_;
    ROS_INFO(" flange pose for object grasp: ");
    cout << "origin: " << a_flange_grasp_.translation().transpose() << endl;
    cout << "R matrix: " << endl;
    cout << a_flange_grasp_.linear() << endl;

    ROS_INFO("flange pose for approach: ");
    cout << "origin: " << a_flange_approach_.translation().transpose() << endl;
    cout << "R matrix: " << endl;
    cout << a_flange_approach_.linear() << endl;


    int gripper_status;
    gripper_status = open_gripper(95.0);
    if (object_grabber::object_grabberResult::GRIPPER_FAILURE == gripper_status) {
        return gripper_status;
    }


    int move_to_rtn_code;

    des_flange_approach_pose.header.frame_id = "torso";
    des_flange_approach_pose.pose = xformUtils.transformEigenAffine3dToPose(a_flange_approach_);



    move_to_rtn_code = jspace_move_flange_to(des_flange_approach_pose);
    if (move_to_rtn_code != object_grabber::object_grabberResult::SUCCESS) {

        return move_to_rtn_code;
    }



    move_to_rtn_code = fine_move_flange_to(des_flange_grasp_pose);
    if (move_to_rtn_code != object_grabber::object_grabberResult::SUCCESS) {
        return move_to_rtn_code;
    }


    gripper_status = close_gripper(gripper_close_test_val);
    if (object_grabber::object_grabberResult::GRIPPER_FAILURE == gripper_status) {
        return gripper_status;
    }







    des_flange_depart_pose = des_flange_approach_pose;
    move_to_rtn_code = move_flange_to(des_flange_depart_pose);
    if (move_to_rtn_code != object_grabber::object_grabberResult::SUCCESS) {
        return move_to_rtn_code;
    }

    return object_grabber::object_grabberResult::SUCCESS;
}











int ObjectGrabber::dropoff_from_above(geometry_msgs::PoseStamped des_flange_dropoff_pose, double approach_dist) {
    int move_to_rtn_code;
    geometry_msgs::PoseStamped des_flange_approach_pose;
    Eigen::Affine3d dropoff_flange_affine, approach_flange_affine;



    dropoff_flange_affine = xformUtils.transformPoseToEigenAffine3d(des_flange_dropoff_pose.pose);



    Eigen::Vector3d approach_flange_origin, dropoff_flange_origin;
    dropoff_flange_origin = dropoff_flange_affine.translation();
    Eigen::Vector3d toolflange_z_axis;
    Eigen::Matrix3d R;

    R = dropoff_flange_affine.linear();
    toolflange_z_axis = R.col(2);
    cout << "toolflange_z_axis: " << toolflange_z_axis.transpose() << endl;
    cout << "offset by " << approach_dist << " to descend to grasp pose" << endl;

    approach_flange_affine = dropoff_flange_affine;

    approach_flange_affine.translation() = dropoff_flange_affine.translation() - toolflange_z_axis*approach_dist;


    des_flange_approach_pose.header.frame_id = "torso";
    des_flange_approach_pose.pose = xformUtils.transformEigenAffine3dToPose(approach_flange_affine);
    des_flange_dropoff_pose.header.frame_id = "torso";
    des_flange_dropoff_pose.pose = xformUtils.transformEigenAffine3dToPose(dropoff_flange_affine);
    ROS_INFO("attempting move to approach pose");

    move_to_rtn_code = jspace_move_flange_to(des_flange_approach_pose);
    if (move_to_rtn_code != object_grabber::object_grabberResult::SUCCESS) {
        ROS_WARN("failure: return code = %d", move_to_rtn_code);
        return move_to_rtn_code;
    }

    ROS_INFO("attempting fine-move approach to drop-off");
    move_to_rtn_code = fine_move_flange_to(des_flange_dropoff_pose);
    if (move_to_rtn_code != object_grabber::object_grabberResult::SUCCESS) {
        ROS_WARN("failure: return code = %d", move_to_rtn_code);
        return move_to_rtn_code;
    }
    ros::Duration(0.5).sleep();

    ROS_INFO("releasing the part");
    int gripper_status;
    gripper_status = open_gripper(95.0);
    if (object_grabber::object_grabberResult::GRIPPER_FAILURE == gripper_status) {
        return gripper_status;
    }
    ros::Duration(0.5).sleep();

    ROS_INFO("computing/executing depart move");
    move_to_rtn_code = fine_move_flange_to(des_flange_approach_pose);
    if (move_to_rtn_code != object_grabber::object_grabberResult::SUCCESS) {
        ROS_WARN("failure: return code = %d", move_to_rtn_code);
        return move_to_rtn_code;
    }

    move_to_rtn_code = jspace_move_to_pre_pose();

    return object_grabber::object_grabberResult::SUCCESS;
}
















int ObjectGrabber::vertical_cylinder_power_grasp(geometry_msgs::PoseStamped object_pose) {
    geometry_msgs::PoseStamped des_gripper_grasp_pose, des_gripper_approach_pose, des_gripper_depart_pose;
    geometry_msgs::PoseStamped des_flange_grasp_pose, des_flange_approach_pose, des_flange_depart_pose;
    Eigen::Affine3d flange_approach_affine, flange_grasp_affine, flange_depart_affine;

    int rtn_val, execute_return_code;

    rtn_val = armMotionCommander.plan_move_to_pre_pose();
    if (rtn_val == cartesian_planner::baxter_cart_moveResult::SUCCESS) {

        ROS_INFO("sending command to execute planned path to pre-pose:");
        execute_return_code = armMotionCommander.rt_arm_execute_planned_path();
    } else {
        ROS_WARN("desired motion to pre-pose is not feasible");
        return object_grabber::object_grabberResult::FAILED_CANNOT_REACH;
    }


    rtn_val = armMotionCommander.rt_arm_request_q_data();

    Eigen::Affine3d object_affine;
    object_affine =
            xformUtils.transformPoseToEigenAffine3d(object_pose.pose);
    Eigen::Vector3d object_origin;
    object_origin = object_affine.translation();
    grasp_origin_ = object_origin;

    a_gripper_grasp_.translation() = grasp_origin_;


    flange_grasp_affine = a_gripper_grasp_ * a_right_gripper_frame_wrt_flange_.inverse();



    flange_approach_affine = flange_grasp_affine;
    flange_approach_affine.translation() = flange_grasp_affine.translation() - gripper_b_des_*L_approach_;


    flange_depart_affine = flange_grasp_affine;
    flange_depart_affine.translation() = flange_grasp_affine.translation() + gripper_n_des_*z_depart_;


    ROS_INFO("opening gripper");
    baxterGripper.right_gripper_open();
    ros::spinOnce();
    ROS_INFO("right gripper pos = %f; waiting for pos>95", baxterGripper.get_right_gripper_pos());
    while (baxterGripper.get_right_gripper_pos() < 95.0) {
        baxterGripper.right_gripper_open();
        ros::spinOnce();
        ROS_INFO("gripper pos = %f", baxterGripper.get_right_gripper_pos());
        ros::Duration(0.01).sleep();
    }


    int planner_rtn_code;
    des_flange_approach_pose.header.frame_id = "torso";
    des_flange_approach_pose.pose = xformUtils.transformEigenAffine3dToPose(flange_approach_affine);
    planner_rtn_code = armMotionCommander.rt_arm_plan_path_current_to_goal_flange_pose(des_flange_approach_pose);
    if (planner_rtn_code == cartesian_planner::baxter_cart_moveResult::SUCCESS) {

        ROS_INFO("sending command to execute planned path:");
        execute_return_code = armMotionCommander.rt_arm_execute_planned_path();
    } else {
        ROS_WARN("desired motion is not feasible");
        return object_grabber::object_grabberResult::FAILED_CANNOT_REACH;
    }


    ROS_INFO("planning hi-res approach to grasp pose");
    des_flange_grasp_pose.header.frame_id = "torso";
    des_flange_grasp_pose.pose = xformUtils.transformEigenAffine3dToPose(flange_grasp_affine);
    planner_rtn_code = armMotionCommander.rt_arm_plan_fine_path_current_to_goal_flange_pose(des_flange_grasp_pose);

    double time_stretch_factor = 3.0;
    planner_rtn_code = armMotionCommander.rt_arm_timestretch_planned_path(time_stretch_factor);

    if (planner_rtn_code == cartesian_planner::baxter_cart_moveResult::SUCCESS) {

        ROS_INFO("sending command to execute planned path:");
        execute_return_code = armMotionCommander.rt_arm_execute_planned_path();
    } else {
        ROS_WARN("desired motion is not feasible");
        return object_grabber::object_grabberResult::FAILED_CANNOT_REACH;
    }



    ROS_INFO("closing gripper");
    baxterGripper.right_gripper_close();
    ros::spinOnce();

    ROS_INFO("gripper pos = %f", baxterGripper.get_right_gripper_pos());
    while (baxterGripper.get_right_gripper_pos() > 90.0) {
        baxterGripper.right_gripper_close();
        ros::spinOnce();
        ROS_INFO("gripper pos = %f", baxterGripper.get_right_gripper_pos());
        ros::Duration(0.01).sleep();
    }
    ros::Duration(1).sleep();


    des_flange_depart_pose.header.frame_id = "torso";
    des_flange_depart_pose.pose = xformUtils.transformEigenAffine3dToPose(flange_depart_affine);
    planner_rtn_code = armMotionCommander.rt_arm_plan_path_current_to_goal_flange_pose(des_flange_depart_pose);
    if (planner_rtn_code == cartesian_planner::baxter_cart_moveResult::SUCCESS) {

        ROS_INFO("sending command to execute planned path:");
        execute_return_code = armMotionCommander.rt_arm_execute_planned_path();
    } else {
        ROS_WARN("desired motion is not feasible");
        return object_grabber::object_grabberResult::FAILED_CANNOT_REACH;
    }
    return object_grabber::object_grabberResult::SUCCESS;

}












void ObjectGrabber::executeCB(const actionlib::SimpleActionServer<object_grabber::object_grabberAction>::GoalConstPtr&
goal) {

    int action_code = goal->action_code;
    ROS_INFO("got action code %d", action_code);
    int object_grabber_rtn_code;
    switch (action_code) {
        case object_grabber::object_grabberGoal::GRAB_UPRIGHT_CYLINDER:
            ROS_INFO("case GRAB_UPRIGHT_CYLINDER");
            object_pose_stamped_ = goal->desired_frame;
            object_pose_stamped_wrt_torso_ = convert_pose_to_torso_frame(object_pose_stamped_);

            object_grabber_rtn_code = vertical_cylinder_power_grasp(object_pose_stamped_wrt_torso_);
            grab_result_.return_code = object_grabber_rtn_code;
            object_grabber_as_.setSucceeded(grab_result_);
            break;
        case object_grabber::object_grabberGoal::GRAB_W_TOOL_Z_APPROACH:
            ROS_INFO("case GRAB_W_TOOL_Z_APPROACH");
            object_pose_stamped_ = goal->desired_frame;
            object_id_ = goal->object_id;
            ROS_INFO("object_id = %d", object_id_);


            if (!objectManipulationProperties.get_object_info(object_id_, grasp_transform_,
                    approach_dist_, gripper_test_val_)) {
                ROS_WARN("object ID not recognized");
                grab_result_.return_code = object_grabber::object_grabberResult::ACTION_CODE_UNKNOWN;
                object_grabber_as_.setAborted(grab_result_);
            }

            object_pose_stamped_wrt_torso_ = convert_pose_to_torso_frame(object_pose_stamped_);
            des_flange_pose_stamped_wrt_torso_ = object_to_flange_grasp_transform(object_id_,
                    object_pose_stamped_wrt_torso_);

            object_grabber_rtn_code = grasp_approach_tool_z_axis(des_flange_pose_stamped_wrt_torso_,
                    approach_dist_, gripper_test_val_);

            grab_result_.return_code = object_grabber_rtn_code;
            grab_result_.des_flange_pose_stamped_wrt_torso = des_flange_pose_stamped_wrt_torso_;
            object_grabber_as_.setSucceeded(grab_result_);
            break;




        case object_grabber::object_grabberGoal::DROPOFF_ALONG_TOOL_Z:

            ROS_INFO("case DROPOFF_ALONG_TOOL_Z");
            object_pose_stamped_ = goal->desired_frame;
            object_pose_stamped_wrt_torso_ = convert_pose_to_torso_frame(object_pose_stamped_);
            object_id_ = goal->object_id;

            if (!objectManipulationProperties.get_object_info(object_id_, grasp_transform_,
                    approach_dist_, gripper_test_val_)) {
                ROS_WARN("object ID not recognized");
                grab_result_.return_code = object_grabber::object_grabberResult::ACTION_CODE_UNKNOWN;
                object_grabber_as_.setAborted(grab_result_);
            }


            des_flange_pose_stamped_wrt_torso_ = object_to_flange_grasp_transform(object_id_,
                    object_pose_stamped_wrt_torso_);


            object_grabber_rtn_code = dropoff_from_above(des_flange_pose_stamped_wrt_torso_, approach_dist_);
            grab_result_.return_code = object_grabber_rtn_code;
            grab_result_.des_flange_pose_stamped_wrt_torso = des_flange_pose_stamped_wrt_torso_;
            object_grabber_as_.setSucceeded(grab_result_);
            break;

        case object_grabber::object_grabberGoal::MOVE_FLANGE_TO:
            ROS_INFO("case MOVE_FLANGE_TO");
            des_flange_pose_stamped_ = goal->desired_frame;
            des_flange_pose_stamped_wrt_torso_ = convert_pose_to_torso_frame(des_flange_pose_stamped_);
            grab_result_.return_code = move_flange_to(des_flange_pose_stamped_wrt_torso_);
            grab_result_.des_flange_pose_stamped_wrt_torso = des_flange_pose_stamped_wrt_torso_;
            object_grabber_as_.setSucceeded(grab_result_);
            break;
        case object_grabber::object_grabberGoal::JSPACE_MOVE_FLANGE_TO:
            ROS_INFO("object-grabber: case JSPACE_MOVE_FLANGE_TO");
            des_flange_pose_stamped_ = goal->desired_frame;
            des_flange_pose_stamped_wrt_torso_ = convert_pose_to_torso_frame(des_flange_pose_stamped_);
            grab_result_.return_code = jspace_move_flange_to(des_flange_pose_stamped_wrt_torso_);
            grab_result_.des_flange_pose_stamped_wrt_torso = des_flange_pose_stamped_wrt_torso_;
            object_grabber_as_.setSucceeded(grab_result_);
            break;

            break;

        case object_grabber::object_grabberGoal::FINE_MOVE_FLANGE_TO:
            ROS_INFO("case FINE_MOVE_FLANGE_TO");
            des_flange_pose_stamped_ = goal->desired_frame;
            des_flange_pose_stamped_wrt_torso_ = convert_pose_to_torso_frame(des_flange_pose_stamped_);
            grab_result_.return_code = fine_move_flange_to(des_flange_pose_stamped_wrt_torso_);
            grab_result_.des_flange_pose_stamped_wrt_torso = des_flange_pose_stamped_wrt_torso_;
            object_grabber_as_.setSucceeded(grab_result_);
            break;
        case object_grabber::object_grabberGoal::OPEN_GRIPPER:
            open_gripper(95.0);
            break;
        case object_grabber::object_grabberGoal::CLOSE_GRIPPER:

            ROS_INFO("closing gripper to test val %f", gripper_test_val_);
            close_gripper(gripper_test_val_);
            break;

        case object_grabber::object_grabberGoal::MOVE_TO_PRE_POSE:
            grab_result_.return_code = jspace_move_to_pre_pose();
            object_grabber_as_.setSucceeded(grab_result_);
            break;

        default:
            ROS_WARN("this object ID is not implemented");
            grab_result_.return_code = object_grabber::object_grabberResult::ACTION_CODE_UNKNOWN;
            object_grabber_as_.setAborted(grab_result_);
    }

}

Eigen::Affine3d ObjectGrabber::get_right_tool_transform(void) {


    ROS_INFO("listening for gripper-to-toolflange transform:");
    tf::StampedTransform stf_gripper_wrt_flange;
    bool tferr = true;
    ROS_INFO("waiting for tf between right gripper and right tool flange...");
    while (tferr) {
        tferr = false;
        try {




            tfListener.lookupTransform("right_hand", "right_gripper", ros::Time(0), stf_gripper_wrt_flange);
        } catch (tf::TransformException &exception) {
            ROS_WARN("%s; retrying...", exception.what());
            tferr = true;
            ros::Duration(0.5).sleep();
            ros::spinOnce();
        }
    }
    ROS_INFO("right gripper to right tool flange tf is good");
    xformUtils.printStampedTf(stf_gripper_wrt_flange);
    tf::Transform tf_kinect_wrt_base = xformUtils.get_tf_from_stamped_tf(stf_gripper_wrt_flange);
    Eigen::Affine3d affine_gripper_wrt_flange = xformUtils.transformTFToAffine3d(tf_kinect_wrt_base);

    std::cout << "affine rotation: " << std::endl;
    std::cout << affine_gripper_wrt_flange.linear() << std::endl;
    std::cout << "affine offset: " << affine_gripper_wrt_flange.translation().transpose() << std::endl;
    return affine_gripper_wrt_flange;
}
 */