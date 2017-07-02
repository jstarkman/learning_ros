#include <math.h>
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"

const double M_virt = 1.0;
const double K_virt = 1000.0;
const double B_virt = 50.0;
const double v_ideal_sat = 1.0;
const double x_attractor = -0.2;

double g_link2_pos = 0.0;
double g_link2_vel = 0.0;

double g_force_z = 0.0;

double sat(double val, double sat_val)
{
  if (val > sat_val)
    return (sat_val);
  if (val < -sat_val)
    return (-sat_val);
  return val;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::node::Node::make_shared("nac_controller");

  auto cmd_publisher = node->create_publisher<std_msgs::msg::Float64>(
      "/one_DOF_robot/joint1_velocity_controller/command", rmw_qos_profile_default);

  auto v_cmd_float64 = std::make_shared<std_msgs::msg::Float64>();

  auto joint_state_sub =
      node->create_subscription<sensor_msgs::msg::JointState>("one_DOF_robot/joint_states",
                                                              [](const sensor_msgs::msg::JointState::SharedPtr js) {
                                                                g_link2_pos = js->position[0];
                                                                g_link2_vel = js->velocity[0];
                                                              },
                                                              rmw_qos_profile_default);

  auto ft_sensor_subscriber = node->create_subscription<geometry_msgs::msg::WrenchStamped>(
      "ft_sensor_topic", [](const geometry_msgs::msg::WrenchStamped::SharedPtr ft) { g_force_z = ft->wrench.force.z; },
      rmw_qos_profile_default);

  double acc_ideal = 0.0;
  double v_ideal = 0.0;

  double dt = 0.001;
  rclcpp::WallRate sample_rate(1 / dt);
  double f_net = 0.0;
  double f_virt = 0.0;

  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);
    f_virt = K_virt * (x_attractor - g_link2_pos) + B_virt * (0 - g_link2_vel);
    f_net = g_force_z + f_virt;
    acc_ideal = f_net / M_virt;
    v_ideal += acc_ideal * dt;
    v_ideal = sat(v_ideal, v_ideal_sat);
    v_cmd_float64->data = v_ideal;

    cmd_publisher->publish(v_cmd_float64);
    rclcpp::spin_some(node);
    sample_rate.sleep();
  }
}
