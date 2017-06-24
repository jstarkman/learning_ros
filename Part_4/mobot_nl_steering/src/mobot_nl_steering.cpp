#include "mobot_nl_steering.hpp"

SteeringController::SteeringController(rclcpp::Node::SharedPtr node) : node_(node)
{
  ROS_INFO("in class constructor of SteeringController");
  initializeSubscribers();
  initializePublishers();

  state_psi_ = 1000.0;

  ROS_INFO("waiting for valid state message...");
  rclcpp::WallRate timer(2);
  while (state_psi_ > 500.0)
  {
    timer.sleep();
    std::cout << ".";
    rclcpp::spin(node);
  }
  ROS_INFO("constructor: got a state message");

  des_state_speed_ = MAX_SPEED;
  des_state_omega_ = 0.0;

  des_state_x_ = 0.0;
  des_state_y_ = 0.0;
  des_state_psi_ = 0.0;

  current_speed_des_ = 0.0;
  current_omega_des_ = 0.0;

  twist_cmd_.linear.x = 0.0;
  twist_cmd_.linear.y = 0.0;
  twist_cmd_.linear.z = 0.0;
  twist_cmd_.angular.x = 0.0;
  twist_cmd_.angular.y = 0.0;
  twist_cmd_.angular.z = 0.0;
}

void SteeringController::initializeSubscribers()
{
  ROS_INFO("Initializing Subscribers: gazebo state and desState");

  current_state_subscriber_ = node_->create_subscription<geometry_msgs::msg::Pose>(
      "gazebo_mobot_pose", std::bind(&SteeringController::gazeboPoseCallback, this, std::placeholders::_1),
      rmw_qos_profile_default);

  des_state_subscriber_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      "/desState", std::bind(&SteeringController::desStateCallback, this, std::placeholders::_1),
      rmw_qos_profile_default);
}

void SteeringController::initializePublishers()
{
  ROS_INFO("Initializing Publishers");
  cmd_publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", rmw_qos_profile_default);
  heading_publisher_ = node_->create_publisher<std_msgs::msg::Float32>("heading", rmw_qos_profile_default);
  heading_cmd_publisher_ = node_->create_publisher<std_msgs::msg::Float32>("heading_cmd", rmw_qos_profile_default);
  lat_err_publisher_ = node_->create_publisher<std_msgs::msg::Float32>("lateral_err", rmw_qos_profile_default);
}

void SteeringController::gazeboPoseCallback(const geometry_msgs::msg::Pose::SharedPtr gazebo_pose)
{
  std::cout << "rx gazebo_pose!" << std::endl;
  state_x_ = gazebo_pose->position.x;
  state_y_ = gazebo_pose->position.y;
  state_quat_ = gazebo_pose->orientation;
  state_psi_ = convertPlanarQuat2Phi(state_quat_);
}

void SteeringController::desStateCallback(const nav_msgs::msg::Odometry::SharedPtr des_state_rcvd)
{
  des_state_speed_ = des_state_rcvd->twist.twist.linear.x;
  des_state_omega_ = des_state_rcvd->twist.twist.angular.z;

  des_state_x_ = des_state_rcvd->pose.pose.position.x;
  des_state_y_ = des_state_rcvd->pose.pose.position.y;
  des_state_pose_ = des_state_rcvd->pose.pose;
  des_state_quat_ = des_state_rcvd->pose.pose.orientation;

  des_state_psi_ = convertPlanarQuat2Phi(des_state_quat_);
}

double SteeringController::min_dang(double dang)
{
  while (dang > M_PI)
    dang -= 2.0 * M_PI;
  while (dang < -M_PI)
    dang += 2.0 * M_PI;
  return dang;
}

double SteeringController::sat(double x)
{
  if (x > 1.0)
  {
    return 1.0;
  }
  if (x < -1.0)
  {
    return -1.0;
  }
  return x;
}

double SteeringController::convertPlanarQuat2Phi(geometry_msgs::msg::Quaternion quaternion)
{
  double quat_z = quaternion.z;
  double quat_w = quaternion.w;
  double phi = 2.0 * atan2(quat_z, quat_w);
  return phi;
}

void SteeringController::mobot_nl_steering()
{
  double controller_speed;
  double controller_omega;

  double tx = cos(des_state_psi_);
  double ty = sin(des_state_psi_);
  double nx = -ty;
  double ny = tx;

  double dx = state_x_ - des_state_x_;
  double dy = state_y_ - des_state_y_;

  lateral_err_ = dx * nx + dy * ny;

  double trip_dist_err = dx * tx + dy * ty;

  double heading_err = min_dang(state_psi_ - des_state_psi_);
  double strategy_psi = psi_strategy(lateral_err_);
  controller_omega = omega_cmd_fnc(strategy_psi, state_psi_, des_state_psi_);

  controller_speed = MAX_SPEED;

  twist_cmd_.linear.x = controller_speed;
  twist_cmd_.angular.z = controller_omega;
  cmd_publisher_->publish(twist_cmd_);

  ROS_INFO("des_state_phi, heading err = %f, %f", des_state_psi_, heading_err);
  ROS_INFO("lateral err, trip dist err = %f, %f", lateral_err_, trip_dist_err);
  std_msgs::msg::Float32 float_msg;
  float_msg.data = lateral_err_;
  lat_err_publisher_->publish(float_msg);
  float_msg.data = state_psi_;
  heading_publisher_->publish(float_msg);
  float_msg.data = psi_cmd_;
  heading_cmd_publisher_->publish(float_msg);
}

double SteeringController::psi_strategy(double offset_err)
{
  double psi_strategy = -(M_PI / 2) * sat(offset_err / K_LAT_ERR_THRESH);
  return psi_strategy;
}

double SteeringController::omega_cmd_fnc(double psi_strategy, double psi_state, double psi_path)
{
  psi_cmd_ = psi_strategy + psi_path;
  double omega_cmd = K_PSI * (psi_cmd_ - psi_state);
  omega_cmd = MAX_OMEGA * sat(omega_cmd / MAX_OMEGA);
  return omega_cmd;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("steeringController");

  ROS_INFO("main: instantiating an object of type SteeringController");

  SteeringController steeringController(node);

  rclcpp::WallRate sleep_timer(UPDATE_RATE);

  ROS_INFO("starting steering algorithm");
  while (rclcpp::ok())
  {
    steeringController.mobot_nl_steering();
    rclcpp::spin(node);
    sleep_timer.sleep();
  }
  return 0;
}
