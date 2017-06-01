#include <iostream>
#include <string>
#include "example_ros_service/srv/example_service_msg.hpp"
#include "rclcpp/rclcpp.hpp"

example_ros_service::srv::ExampleServiceMsg_Response::SharedPtr
send_blocking_request(rclcpp::node::Node::SharedPtr node,
                      rclcpp::client::Client<example_ros_service::srv::ExampleServiceMsg>::SharedPtr client,
                      example_ros_service::srv::ExampleServiceMsg_Request::SharedPtr request)
{
  std::cout << "Sending..." << std::endl;
  auto response = client->async_send_request(request);
  // wait for the response
  if (rclcpp::spin_until_future_complete(node, response) == rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    std::cout << "Received." << std::endl;
    return response.get();
  }
  else
  {
    return NULL;
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::node::Node::make_shared("example_ros_client");
  auto client = node->create_client<example_ros_service::srv::ExampleServiceMsg>("lookup_by_name");

  auto request = std::make_shared<example_ros_service::srv::ExampleServiceMsg::Request>();

  std::string in_name;
  while (rclcpp::ok())
  {
    std::cout << std::endl;
    std::cout << "enter a name (x to quit): ";
    std::cin >> in_name;
    if (in_name.compare("x") == 0)
    {
      return 0;
    }

    request->name = in_name;
    auto response = send_blocking_request(node, client, request);
    if (response)
    {
      if (response->on_the_list)
      {
        std::cout << request->name << " is known as " << response->nickname << std::endl;
        std::cout << "He is " << response->age << " years old" << std::endl;
        if (response->good_guy)
          std::cout << "He is reported to be a good guy" << std::endl;
        else
          std::cout << "Avoid him; he is not a good guy" << std::endl;
      }
      else
      {
        std::cout << request->name << " is not in my database" << std::endl;
      }
    }
    else
    {
      std::cerr << "Failed to call service lookup_by_name" << std::endl;
      return 1;
    }
  }
  return 0;
}
