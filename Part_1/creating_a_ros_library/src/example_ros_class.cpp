#include "creating_a_ros_library/example_ros_class.hpp"

ExampleRosClass::ExampleRosClass(rclcpp::node::Node::SharedPtr node) : node_(node)
{
  ROS_INFO("in class constructor of ExampleRosClass");
  initializeSubscribers();
  initializePublishers();
  initializeServices();

  val_to_remember_ = 0.0;
}

void ExampleRosClass::initializeSubscribers()
{
  ROS_INFO("Initializing Subscribers");
  minimal_subscriber_ = node_->create_subscription<std_msgs::msg::Float32>(
      "example_class_input_topic", std::bind(&ExampleRosClass::subscriberCallback, this, std::placeholders::_1),
      rmw_qos_profile_default);
}

void ExampleRosClass::initializeServices()
{
  ROS_INFO("Initializing Services");
  minimal_service_ = node_->create_service<std_srvs::srv::Trigger>(
      "example_minimal_service", std::bind(&ExampleRosClass::serviceCallback, this, std::placeholders::_1,
                                           std::placeholders::_2, std::placeholders::_3));
}

void ExampleRosClass::initializePublishers()
{
  ROS_INFO("Initializing Publishers");
  minimal_publisher_output_msg_ = std::make_shared<std_msgs::msg::Float32>();
  minimal_publisher_ =
      node_->create_publisher<std_msgs::msg::Float32>("example_class_output_topic", rmw_qos_profile_default);
}

void ExampleRosClass::subscriberCallback(const std_msgs::msg::Float32::SharedPtr message_holder)
{
  val_from_subscriber_ = message_holder->data;
  ROS_INFO("myCallback activated: received value %f", val_from_subscriber_);

  val_to_remember_ += val_from_subscriber_;
  minimal_publisher_output_msg_->data = val_to_remember_;

  minimal_publisher_->publish(minimal_publisher_output_msg_);
}

void ExampleRosClass::serviceCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                                      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  ROS_INFO("service callback activated");
  response->success = true;
  response->message = "here is a response string";
}
