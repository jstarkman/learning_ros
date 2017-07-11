#ifndef EXAMPLE_ROS_CLASS_H_
#define EXAMPLE_ROS_CLASS_H_

#define _USE_MATH_DEFINES
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_srvs/srv/trigger.hpp"

// temporary hack until rosconsole works
// https://gcc.gnu.org/onlinedocs/cpp/Variadic-Macros.html
#define ROS_INFO printf

class ExampleRosClass
{
public:
  ExampleRosClass(rclcpp::node::Node::SharedPtr node);

private:
  rclcpp::node::Node::SharedPtr node_;

  rclcpp::subscription::Subscription<std_msgs::msg::Float32>::SharedPtr minimal_subscriber_;
  rclcpp::service::Service<std_srvs::srv::Trigger>::SharedPtr minimal_service_;
  rclcpp::publisher::Publisher<std_msgs::msg::Float32>::SharedPtr minimal_publisher_;
  std_msgs::msg::Float32::SharedPtr minimal_publisher_output_msg_;

  double val_from_subscriber_;
  double val_to_remember_;

  void initializeSubscribers();
  void initializePublishers();
  void initializeServices();

  void subscriberCallback(const std_msgs::msg::Float32::SharedPtr message_holder);

  void serviceCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                       const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> response);
};

#endif
