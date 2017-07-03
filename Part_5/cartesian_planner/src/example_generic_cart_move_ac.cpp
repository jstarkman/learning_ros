

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <cartesian_planner/cart_motion_commander.h>
#include <cartesian_planner/cart_moveAction.h>
#include <ros/ros.h>
#include <xform_utils/xform_utils.h>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "example_arm_cart_move_ac");
  ros::NodeHandle nh;
  ArmMotionCommander arm_motion_commander;
  XformUtils xformUtils;
  Eigen::VectorXd joint_angles;
  Eigen::Vector3d dp_displacement;
  int rtn_val;
  int njnts;
  geometry_msgs::PoseStamped tool_pose;

  arm_motion_commander.send_test_goal();

  ROS_INFO("commanding move to waiting pose");
  rtn_val = arm_motion_commander.plan_move_to_waiting_pose();

  rtn_val = arm_motion_commander.execute_planned_path();

  rtn_val = arm_motion_commander.request_q_data();

  rtn_val = arm_motion_commander.request_tool_pose();

  joint_angles = arm_motion_commander.get_joint_angles();
  njnts = joint_angles.size();

  for (int i = 0; i < njnts; i++)
    joint_angles[i] += 0.2;
  ROS_INFO("joint-space move, all joints +0.2 rad");

  rtn_val = arm_motion_commander.plan_jspace_path_current_to_qgoal(joint_angles);

  rtn_val = arm_motion_commander.execute_planned_path();

  rtn_val = arm_motion_commander.request_q_data();

  ROS_INFO("back to waiting pose");
  rtn_val = arm_motion_commander.plan_move_to_waiting_pose();
  rtn_val = arm_motion_commander.execute_planned_path();

  rtn_val = arm_motion_commander.request_tool_pose();
  tool_pose = arm_motion_commander.get_tool_pose_stamped();
  ROS_INFO("tool pose is: ");
  xformUtils.printPose(tool_pose);

  std::cout << "enter 1: ";
  int ans;
  std::cin >> ans;

  ROS_INFO("planning Cartesian move to goal pose w/ dpx = 0.2");

  tool_pose.pose.position.x += 0.2;

  rtn_val = arm_motion_commander.plan_path_current_to_goal_gripper_pose(tool_pose);

  rtn_val = arm_motion_commander.execute_planned_path();

  ROS_INFO("will plan vertical motion");
  std::cout << "enter desired delta-z: ";
  double delta_z;
  std::cin >> delta_z;
  ROS_INFO("moving dz = %f", delta_z);
  dp_displacement << 0, 0, delta_z;
  rtn_val = arm_motion_commander.plan_path_current_to_goal_dp_xyz(dp_displacement);
  if (rtn_val == cartesian_planner::cart_moveResult::SUCCESS)
  {
    rtn_val = arm_motion_commander.execute_planned_path();
  }
  return 0;
}
