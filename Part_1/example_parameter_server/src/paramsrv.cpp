#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"

/**
 * Simple parameter server for mimicking ROS1-like behavior.
 */
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("paramsrv");
  // one day, this will start automatically
  auto parameter_service = std::make_shared<rclcpp::parameter_service::ParameterService>(node);

  auto parameters_client = std::make_shared<rclcpp::parameter_client::AsyncParametersClient>(node);
  auto results = parameters_client->set_parameters({
      rclcpp::parameter::ParameterVariant("mimic/ros1", true),
  });

  rclcpp::spin(node);
  return 0;
}
