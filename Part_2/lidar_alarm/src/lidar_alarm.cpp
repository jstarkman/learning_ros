#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"

const double MIN_SAFE_DISTANCE = 1.0;

float ping_dist_in_front_ = 3.0;
int ping_index_ = -1;
double angle_min_ = 0.0;
double angle_max_ = 0.0;
double angle_increment_ = 0.0;
double range_min_ = 0.0;
double range_max_ = 0.0;
bool laser_alarm_ = false;

rclcpp::publisher::Publisher<std_msgs::msg::Bool>::SharedPtr lidar_alarm_publisher_;
rclcpp::publisher::Publisher<std_msgs::msg::Float32>::SharedPtr lidar_dist_publisher_;

// auto my_publisher = node->create_publisher<std_msgs::msg::Float64>(
//       "force_cmd", rmw_qos_profile_default);

void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr laser_scan)
{
  if (ping_index_ < 0)
  {
    angle_min_ = laser_scan->angle_min;
    angle_max_ = laser_scan->angle_max;
    angle_increment_ = laser_scan->angle_increment;
    range_min_ = laser_scan->range_min;
    range_max_ = laser_scan->range_max;

    ping_index_ = (int)((0.0 - angle_min_) / angle_increment_);
    std::cout << "LIDAR setup: ping_index = " << ping_index_ << std::endl;
  }

  ping_dist_in_front_ = laser_scan->ranges[ping_index_];
  std::cout << "ping dist in front = " << ping_dist_in_front_ << std::endl;
  if (ping_dist_in_front_ < MIN_SAFE_DISTANCE)
  {
    std::cout << "DANGER, WILL ROBINSON!!" << std::endl;
    laser_alarm_ = true;
  }
  else
  {
    laser_alarm_ = false;
  }
  std_msgs::msg::Bool lidar_alarm_msg;
  lidar_alarm_msg.data = laser_alarm_;
  lidar_alarm_publisher_->publish(lidar_alarm_msg);

  std_msgs::msg::Float32 lidar_dist_msg;
  lidar_dist_msg.data = ping_dist_in_front_;
  lidar_dist_publisher_->publish(lidar_dist_msg);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::node::Node::make_shared("lidar_alarm");
  lidar_alarm_publisher_ = node->create_publisher<std_msgs::msg::Bool>("lidar_alarm", rmw_qos_profile_default);
  lidar_dist_publisher_ = node->create_publisher<std_msgs::msg::Float32>("lidar_dist", rmw_qos_profile_default);
  auto lidar_subscriber =
      node->create_subscription<sensor_msgs::msg::LaserScan>("robot0/laser_0", laserCallback, rmw_qos_profile_default);
  rclcpp::spin(node);
  return 0;
}
