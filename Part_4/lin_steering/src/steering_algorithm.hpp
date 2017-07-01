



#ifndef STEERING_ALGORITHM_H_
#define STEERING_ALGORITHM_H_


#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include <ros/ros.h>  


#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>



#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>







const double UPDATE_RATE = 50.0;  
const double K_PHI = 10.0;        
const double K_DISP = 3.0;
const double K_TRIP_DIST = 1.0;


const double MAX_SPEED = 1.0;  
const double MAX_OMEGA = 1.0;  


class SteeringController
{
public:
  SteeringController(ros::NodeHandle* nodehandle);  
                                                    
  
  void lin_steering_algorithm();  
                                  
  double convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion);
  double min_dang(double dang);
  double sat(double x);

private:
  
  ros::NodeHandle nh_;  
  
  ros::Subscriber odom_subscriber_;  
  ros::Subscriber des_state_subscriber_;

  ros::Publisher cmd_publisher_;   
  ros::Publisher cmd_publisher2_;  
  ros::Publisher steering_errs_publisher_;

  tf::TransformListener* tfListener_;
  tf::StampedTransform mapToOdom_;
  tf::StampedTransform baseLink_wrt_map_;
  tf::StampedTransform odomToMap_;
  geometry_msgs::Twist twist_cmd_;
  geometry_msgs::TwistStamped twist_cmd2_;
  double current_speed_des_;
  double current_omega_des_;

  
  nav_msgs::Odometry current_odom_;  
  geometry_msgs::Pose odom_pose_;
  double odom_vel_;
  double odom_omega_;
  double odom_x_;
  double odom_y_;
  double odom_phi_;
  geometry_msgs::Quaternion odom_quat_;
  

  
  nav_msgs::Odometry des_state_;
  geometry_msgs::Pose des_state_pose_;
  double des_state_vel_;
  double des_state_omega_;
  double des_state_x_;
  double des_state_y_;
  double des_state_phi_;
  geometry_msgs::Quaternion des_state_quat_;
  

  
  std_msgs::msg::Float32MultiArray steering_errs_;

  
  void initializeSubscribers();  
                                 
  void initializePublishers();
  void initializeServices();

  void odomCallback(const nav_msgs::Odometry& odom_rcvd);
  void desStateCallback(const nav_msgs::Odometry& des_state_rcvd);
};

#endif
