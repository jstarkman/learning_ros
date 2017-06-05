#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::node::Node::make_shared("param_reader");
  double P_gain, D_gain, I_gain;

  auto parameters_client = std::make_shared<rclcpp::parameter_client::AsyncParametersClient>(node, "paramsrv");


  auto parameters = parameters_client->get_parameters({ "joint1_gains/p", "joint1_gains/i", "joint1_gains/d", });
  if (rclcpp::spin_until_future_complete(node, parameters) !=
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    printf("get_parameters service call failed. Exiting.\n");
    return -1;
  }
  for (auto & parameter : parameters.get()) {
    std::cout << "Parameter name: " << parameter.get_name() << std::endl;
    std::cout << "Parameter value (" << parameter.get_type_name() << "): " <<
      parameter.value_to_string() << std::endl;
  }
}
