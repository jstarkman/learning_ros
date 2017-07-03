#include "generic_gripper_services/srv/generic_gripper_interface.hpp"
#include "rclcpp/rclcpp.hpp"
/* #include "std_srvs/srv/SetBool.hpp" NOT AVAILABLE IN ROS2 (YET?) */

const double MAX_WAIT_TIME = 3.0;
const double DT = 0.01;

ros::ServiceClient* g_client_ptr;

bool callback(generic_gripper_services::genericGripperInterfaceRequest& request,
              generic_gripper_services::genericGripperInterfaceResponse& response)
{
  int command_code = request.cmd_code;

  double wait_time = 0;

  std_srvs::SetBool sticky_srv;

  switch (command_code)
  {
    case generic_gripper_services::genericGripperInterfaceRequest::TEST_PING:
      ROS_INFO("gripper service received a test ping");
      response.success = true;
      return true;
      break;

    case generic_gripper_services::genericGripperInterfaceRequest::RELEASE:
      ROS_INFO("turning off vacuum--part release");
      sticky_srv.request.data = false;
      if (!g_client_ptr->call(sticky_srv))

      {
        ROS_WARN("unsuccessful sticky-fingers service call");
      }
      ros::spinOnce();

      response.success = true;
      return true;
    case generic_gripper_services::genericGripperInterfaceRequest::GRASP_W_PARAMS:
      ROS_WARN("grasp_w_params command not implemented");
      response.success = false;
      return true;

    case generic_gripper_services::genericGripperInterfaceRequest::GRASP:
      ROS_INFO("enabling vacuum to grasp part");
      sticky_srv.request.data = true;
      if (!g_client_ptr->call(sticky_srv))

      {
        ROS_WARN("unsuccessful service call to sticky fingers");
      }

      response.success = true;
      return true;
    case generic_gripper_services::genericGripperInterfaceRequest::TEST_GRASP:
      ROS_WARN("grasp test not implemented");

      response.success = false;
      return true;
    default:
      ROS_WARN("gripper command not recognized");
      response.success = false;
      return true;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "generic_gripper_interface_svc");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("generic_gripper_svc", callback);

  ros::ServiceClient client = n.serviceClient<std_srvs::SetBool>("/sticky_finger/wrist_3_link");

  g_client_ptr = &client;

  ROS_INFO("generic gripper service ready; this version customized for virtual vacuum gripper");
  ros::spin();

  return 0;
}
