#include <iostream>
#include <string>
#include "example_ros_service/srv/example_service_msg.hpp"
#include "rclcpp/rclcpp.hpp"

/* ugly hack until rosconsole works */
#define ROS_INFO printf

void callback(const std::shared_ptr<rmw_request_id_t> request_header,
              const std::shared_ptr<example_ros_service::srv::ExampleServiceMsg::Request> request,
              std::shared_ptr<example_ros_service::srv::ExampleServiceMsg::Response> response)
{
  (void)request_header;
  std::cout << "callback activated" << std::endl;
  std::string in_name(request->name);

  response->on_the_list = false;

  if (in_name.compare("Bob") == 0)
  {
    ROS_INFO("asked about Bob");
    response->age = 32;
    response->good_guy = false;
    response->on_the_list = true;
    response->nickname = "BobTheTerrible";
  }
  if (in_name.compare("Ted") == 0)
  {
    ROS_INFO("asked about Ted");
    response->age = 21;
    response->good_guy = true;
    response->on_the_list = true;
    response->nickname = "Ted the Benevolent";
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::node::Node::make_shared("example_ros_service");
  auto server = node->create_service<example_ros_service::srv::ExampleServiceMsg>("lookup_by_name", callback);

  std::cout << "Ready to look up names." << std::endl;

  rclcpp::spin(node);

  return 0;
}
