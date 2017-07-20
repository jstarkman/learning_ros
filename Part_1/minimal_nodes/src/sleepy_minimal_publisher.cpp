/* Useful libraries to have */
#include <iostream>
#include <memory>

/* Needed for all ROS2 nodes */
#include "rclcpp/rclcpp.hpp"

/* Note the structure:
 * File `package_name/msg/NameOfMessage.msg`
 * maps to header file
 * `package_name/msg/name_of_message.hpp`
 * which will be on the path if `package_name` is included by the build files.
 */
#include "std_msgs/msg/float64.hpp"

/* Standard entry point for executables. */
int main(int argc, char **argv)
{
  /* Prepare to use ROS2 components. */
  rclcpp::init(argc, argv);
  /* Create a node and store a shared pointer to it in `node`. */
  auto node = rclcpp::node::Node::make_shared("minimal_publisher");
  /* Use the node to create a publisher on "/topic1" with default QoS settings. */
  auto pub = node->create_publisher<std_msgs::msg::Float64>("topic1", rmw_qos_profile_default);

  /* Allocate a new message on the heap. */
  auto input_float = std::make_shared<std_msgs::msg::Float64>();
  input_float->data = 0.0;

  /* In hertz.  Based on std::chrono::steady_clock.  For more information,
   * see src/ros2/rclcpp/rclcpp/include/rclcpp/rate.hpp */
  rclcpp::WallRate naptime(1.0);

  /* Effectively `while(true)`, but exits when receives unhandled signal. */
  while (rclcpp::ok())
  {
    /* Increment message value. */
    input_float->data = input_float->data + 0.001;
    /* Publish the message. */
    pub->publish(input_float);
    /* Idle this process until the next wake-up from the timer. */
    naptime.sleep();
    /* Allow node to handle callbacks &c. */
    rclcpp::spin_some(node);
  }

  return 0;
}
