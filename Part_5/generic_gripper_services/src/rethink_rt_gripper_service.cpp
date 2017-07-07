#include "generic_gripper_services/srv/generic_gripper_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "simple_baxter_gripper_interface/simple_baxter_gripper_interface.hpp"

#include "rcutils/logging_macros.h"
#define ROS_INFO RCUTILS_LOG_INFO
#define ROS_WARN RCUTILS_LOG_WARN

const double MAX_WAIT_TIME = 3.0;
const double DT = 0.01;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::node::Node::make_shared("generic_gripper_interface_svc");
  BaxterGripper baxterGripper(node);
  auto service = node->create_service<generic_gripper_services::srv::GenericGripperInterface>(
      "generic_gripper_svc",
      [&, node](const std::shared_ptr<rmw_request_id_t> request_header,
                const generic_gripper_services::srv::GenericGripperInterface::Request::SharedPtr request,
                generic_gripper_services::srv::GenericGripperInterface::Response::SharedPtr response) {
        int command_code = request->cmd_code;

        double wait_time = 0;

        switch (command_code)
        {
          case generic_gripper_services::srv::GenericGripperInterface::Request::TEST_PING:
            ROS_INFO("gripper service received a test ping");
            response->success = true;
            break;
          case generic_gripper_services::srv::GenericGripperInterface::Request::RELEASE:
            ROS_INFO("opening right gripper");
            baxterGripper.right_gripper_open();
            // rclcpp::spin_some(node);

            ROS_INFO("right gripper pos = %f; waiting for pos>95", baxterGripper.get_right_gripper_pos());
            while (baxterGripper.get_right_gripper_pos() < 95.0)
            {
              wait_time += DT;
              baxterGripper.right_gripper_open();
              // rclcpp::spin_some(node);

              rclcpp::WallRate(1 / DT).sleep();
              if (wait_time > MAX_WAIT_TIME)
              {
                ROS_WARN("giving up waiting on gripper opening");
                response->success = false;
                return;
              }
            }

            ROS_INFO("gripper is open");
            ROS_INFO("gripper pos = %f", baxterGripper.get_right_gripper_pos());
            response->success = true;
            break;
          case generic_gripper_services::srv::GenericGripperInterface::Request::GRASP_W_PARAMS:
            ROS_WARN("grasp_w_params command not implemented");
            response->success = false;
            break;
          case generic_gripper_services::srv::GenericGripperInterface::Request::GRASP:
            ROS_INFO("closing right gripper");
            baxterGripper.right_gripper_close();
            rclcpp::WallRate(1.0).sleep();
            response->success = true;
            break;
          case generic_gripper_services::srv::GenericGripperInterface::Request::TEST_GRASP:
            ROS_WARN("grasp test not implemented");
            response->success = false;
            break;
          default:
            ROS_WARN("gripper command not recognized");
            response->success = false;
            break;
        }
      });
  ROS_INFO("generic gripper service ready; this version customized for Rethink right gripper");
  rclcpp::spin(node);

  return 0;
}
