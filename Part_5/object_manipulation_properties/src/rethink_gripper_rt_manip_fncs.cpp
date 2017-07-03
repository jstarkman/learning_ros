#define ROS_INFO printf
#define ROS_WARN printf

void rethink_grasp_TOY_BLOCK_ID(
    int query_code, int grasp_option,
    object_manipulation_msgs::srv::Query::Response::SharedPtr response)
{
  std::vector<geometry_msgs::msg::Pose> object_grasp_poses_wrt_gripper;
  std::vector<geometry_msgs::msg::Pose> object_approach_poses_wrt_gripper;
  std::vector<geometry_msgs::msg::Pose> object_depart_poses_wrt_gripper;
  geometry_msgs::msg::Pose object_pose_wrt_gripper;

  Eigen::Matrix3d R_object_wrt_gripper;
  Eigen::Vector3d object_origin_wrt_gripper_frame;
  Eigen::Vector3d x_axis, y_axis, z_axis;
  XformUtils xformUtils;
  ROS_INFO("query, baxter gripper, toy_block; query code %d, grasp option %d", query_code, grasp_option);

  object_pose_wrt_gripper.position.x = 0.0;
  object_pose_wrt_gripper.position.y = 0.0;
  object_pose_wrt_gripper.position.z = 0.0;
  object_pose_wrt_gripper.orientation.x = 1.0;
  object_pose_wrt_gripper.orientation.y = 0.0;
  object_pose_wrt_gripper.orientation.z = 0.0;
  object_pose_wrt_gripper.orientation.w = 0.0;
  object_grasp_poses_wrt_gripper.clear();
  object_grasp_poses_wrt_gripper.push_back(object_pose_wrt_gripper);
  object_pose_wrt_gripper.orientation.x = 0.0;
  object_pose_wrt_gripper.orientation.y = 1.0;
  object_grasp_poses_wrt_gripper.push_back(object_pose_wrt_gripper);
  Eigen::Affine3d affine_object_wrt_gripper, affine_object_wrt_gripper_approach, affine_object_wrt_gripper_depart;

  switch (query_code)
  {
    case object_manipulation_msgs::srv::Query::Request::GRASP_STRATEGY_OPTIONS_QUERY:

      response->grasp_strategy_options.clear();

      response->grasp_strategy_options.push_back(
          object_manipulation_msgs::srv::Query::Response::GRASP_FROM_ABOVE);

      response->valid_reply = true;
      break;
    case object_manipulation_msgs::srv::Query::Request::APPROACH_STRATEGY_OPTIONS_QUERY:
      response->grasp_strategy_options.clear();

      response->grasp_strategy_options.push_back(
          object_manipulation_msgs::srv::Query::Response::APPROACH_Z_TOOL);
      response->valid_reply = true;
      break;
    case object_manipulation_msgs::srv::Query::Request::DEPART_STRATEGY_OPTIONS_QUERY:
      response->grasp_strategy_options.clear();

      response->grasp_strategy_options.push_back(
          object_manipulation_msgs::srv::Query::Response::DEPART_Z_TOOL);
      response->valid_reply = true;
      break;

    case object_manipulation_msgs::srv::Query::Request::GET_GRASP_POSE_TRANSFORMS:
      if (grasp_option == object_manipulation_msgs::srv::Query::Response::GRASP_FROM_ABOVE)
      {
        response->gripper_pose_options.clear();
        for (int i = 0; i < object_grasp_poses_wrt_gripper.size(); i++)
        {
          response->gripper_pose_options.push_back(object_pose_wrt_gripper);
        }

        response->valid_reply = true;
        break;
      }
      else
      {
        ROS_WARN("this grasp option not specified for RETHINK gripper and object TOY_BLOCK_id");
        response->valid_reply = false;
        break;
      }

    case object_manipulation_msgs::srv::Query::Request::GET_APPROACH_POSE_TRANSFORMS:
      ROS_INFO("get approach pose transforms...");
      if (grasp_option == object_manipulation_msgs::srv::Query::Request::APPROACH_Z_TOOL)
      {
        ROS_INFO("approach grasp along tool-z direction: ");
        object_approach_poses_wrt_gripper.clear();
        for (int i = 0; i < object_grasp_poses_wrt_gripper.size(); i++)
        {
          object_pose_wrt_gripper = object_grasp_poses_wrt_gripper[i];
          object_pose_wrt_gripper.position.z += 0.1;
          object_approach_poses_wrt_gripper.push_back(object_pose_wrt_gripper);
          response->gripper_pose_options.clear();
        }
        for (int i = 0; i < object_grasp_poses_wrt_gripper.size(); i++)
        {
          response->gripper_pose_options.push_back(object_pose_wrt_gripper);
        }
        response->valid_reply = true;
        break;
      }
      else
      {
        ROS_WARN("this grasp option not specified for RETHINK gripper and object TOY_BLOCK_id");
        response->valid_reply = false;
        break;
      }
    case object_manipulation_msgs::srv::Query::Request::GET_DEPART_POSE_TRANSFORMS:
      if (grasp_option == object_manipulation_msgs::srv::Query::Request::DEPART_Z_TOOL)
      {
        ROS_INFO("depart strategy along -tool-z direction");
        object_approach_poses_wrt_gripper.clear();
        for (int i = 0; i < object_grasp_poses_wrt_gripper.size(); i++)
        {
          object_pose_wrt_gripper = object_grasp_poses_wrt_gripper[i];
          object_pose_wrt_gripper.position.z += 0.1;
          object_approach_poses_wrt_gripper.push_back(object_pose_wrt_gripper);
        }
        response->gripper_pose_options.clear();
        for (int i = 0; i < object_grasp_poses_wrt_gripper.size(); i++)
        {
          response->gripper_pose_options.push_back(object_pose_wrt_gripper);
        }
        response->valid_reply = true;
        break;
      }
      else
      {
        ROS_WARN("this grasp option not specified for RETHINK gripper and object TOY_BLOCK_id");
        response->valid_reply = false;
        break;
      }

    default:
      ROS_WARN("grasp poses for rethink gripper unknown for object code TOY_BLOCK_id");
      response->valid_reply = false;
      break;
  }
}

void rethink_grasp_query(int object_id, int query_code, int grasp_option,
                         object_manipulation_msgs::srv::Query::Response::SharedPtr response)
{
  switch (object_id)
  {
    case ObjectIdCodes::TOY_BLOCK_ID:
      rethink_grasp_TOY_BLOCK_ID(query_code, grasp_option, response);
      break;

    default:
      response->valid_reply = false;
      break;
  }
}
