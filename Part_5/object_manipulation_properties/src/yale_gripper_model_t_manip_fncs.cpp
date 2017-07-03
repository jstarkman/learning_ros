void yale_gripper_model_t_grasp_query(
    int object_id, int query_code, int grasp_option,
    object_manipulation_msgs::srv::Query::Response::SharedPtr response)
{
  switch (object_id)
  {
    default:
      response->valid_reply = false;
      break;
  }
}
