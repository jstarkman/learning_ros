#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "object_manipulation_properties/gripper_id_codes.hpp"
#include "object_manipulation_properties/object_id_codes.hpp"
#include "object_manipulation_msgs/srv/query.hpp"
#include "rclcpp/rclcpp.hpp"
#include "xform_utils/xform_utils.hpp"

#include "ariac_vacuum_gripper_manip_fncs.cpp"
#include "rethink_gripper_rt_manip_fncs.cpp"
#include "sticky_fingers_manip_fncs.cpp"
#include "yale_gripper_model_t_manip_fncs.cpp"

using namespace std;

XformUtils xformUtils;

void callback(const std::shared_ptr<rmw_request_id_t> request_header,
              const std::shared_ptr<object_manipulation_msgs::srv::Query::Request> request,
              std::shared_ptr<object_manipulation_msgs::srv::Query::Response> response)
{
  int query_code = request->query_code;

  if (query_code == object_manipulation_msgs::srv::Query::Request::TEST_PING)
  {
    ROS_INFO("object manipulation query service received a test ping");
    response->valid_reply = true;
    return;
  }

  int object_id = request->object_id;
  int gripper_id = request->gripper_id;

  int grasp_option = request->grasp_option;
  ROS_INFO("grasp_option = %d ", grasp_option);

  if (query_code >= object_manipulation_msgs::srv::Query::Request::GET_GRASP_POSE_TRANSFORMS)
  {
    grasp_option = request->grasp_option;
  }

  switch (gripper_id)
  {
    case GripperIdCodes::RETHINK_ELECTRIC_GRIPPER_RT:
      ROS_INFO("rethink gripper query: object_id, query_code, grasp_option: %d, %d, %d", object_id, query_code,
               grasp_option);
      rethink_grasp_query(object_id, query_code, grasp_option, response);
      break;
    case GripperIdCodes::STICKY_FINGERS:
      sticky_fingers_grasp_query(object_id, query_code, grasp_option, response);
      break;
    case GripperIdCodes::YALE_GRIPPER_MODEL_T:
      yale_gripper_model_t_grasp_query(object_id, query_code, grasp_option, response);
      break;
    case GripperIdCodes::ARIAC_VACUUM_GRIPPER:
      ariac_vacuum_gripper_grasp_query(object_id, query_code, grasp_option, response);
      break;

    default:
      ROS_WARN("gripper ID not recognized");
      response->valid_reply = false;
      break;
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::node::Node::make_shared("object_manip_query_svc");
  auto server = node->create_service<object_manipulation_msgs::srv::Query>(
      "object_manip_query_svc", callback);

  rclcpp::spin(node);

  return 0;
}
