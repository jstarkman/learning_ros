





#include <odom_tf/odom_tf.h>
#include <xform_utils/xform_utils.h>
#include "steering_algorithm.h"

double g_odom_tf_x = 0.0;
double g_odom_tf_y = 0.0;
double g_odom_tf_phi = 0.0;

SteeringController::SteeringController(ros::NodeHandle* nodehandle) : nh_(*nodehandle)
{  
  ROS_INFO("in class constructor of SteeringController");
  initializeSubscribers();  
  initializePublishers();
  

  g_odom_tf_phi = 1000.0;  
                           
  ROS_INFO("waiting for valid odomTf update...");
  while (g_odom_tf_phi > 500.0)
  {
    ros::Duration(0.5).sleep();  
    std::cout << ".";
    ros::spinOnce();
  }
  ROS_INFO("constructor: got an odomTf update");

  
  des_state_ = current_odom_;  
  
  current_speed_des_ = 0.0;  
  current_omega_des_ = 0.0;
  des_state_.twist.twist.linear.x = current_speed_des_;  
  des_state_.twist.twist.angular.z = current_omega_des_;
  des_state_.header.stamp = ros::Time::now();

  
  twist_cmd_.linear.x = 0.0;
  twist_cmd_.linear.y = 0.0;
  twist_cmd_.linear.z = 0.0;
  twist_cmd_.angular.x = 0.0;
  twist_cmd_.angular.y = 0.0;
  twist_cmd_.angular.z = 0.0;

  twist_cmd2_.twist = twist_cmd_;               
  twist_cmd2_.header.stamp = ros::Time::now();  
}


void SteeringController::initializeSubscribers()
{
  ROS_INFO("Initializing Subscribers: odom and desState");
  
  odom_subscriber_ =
      nh_.subscribe("/drifty_odom", 1, &SteeringController::odomCallback, this);  
  
  des_state_subscriber_ =
      nh_.subscribe("/desState", 1, &SteeringController::desStateCallback, this);  
}


void SteeringController::initializePublishers()
{
  ROS_INFO("Initializing Publishers: cmd_vel and cmd_vel_stamped");
  cmd_publisher_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);  
  cmd_publisher2_ =
      nh_.advertise<geometry_msgs::TwistStamped>("cmd_vel_stamped", 1, true);  
  
}


void SteeringController::odomCallback(const nav_msgs::Odometry& odom_rcvd)
{
  
  
  current_odom_ = odom_rcvd;  
  
  odom_pose_ = odom_rcvd.pose.pose;
  odom_vel_ = odom_rcvd.twist.twist.linear.x;
  odom_omega_ = odom_rcvd.twist.twist.angular.z;
  odom_x_ = odom_rcvd.pose.pose.position.x;
  odom_y_ = odom_rcvd.pose.pose.position.y;
  odom_quat_ = odom_rcvd.pose.pose.orientation;
  
  odom_phi_ = convertPlanarQuat2Phi(odom_quat_);  
  g_odom_tf_x = odom_x_;
  g_odom_tf_y = odom_y_;
  g_odom_tf_phi = odom_phi_;
  
  
  
}

void SteeringController::desStateCallback(const nav_msgs::Odometry& des_state_rcvd)
{
  
  
  des_state_ = des_state_rcvd;  
  
  des_state_pose_ = des_state_rcvd.pose.pose;
  des_state_vel_ = des_state_rcvd.twist.twist.linear.x;
  des_state_omega_ = des_state_rcvd.twist.twist.angular.z;
  des_state_x_ = des_state_rcvd.pose.pose.position.x;
  des_state_y_ = des_state_rcvd.pose.pose.position.y;
  des_state_quat_ = des_state_rcvd.pose.pose.orientation;
  
  des_state_phi_ =
      convertPlanarQuat2Phi(des_state_quat_);  
  
  
  
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


double SteeringController::convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion)
{
  double quat_z = quaternion.z;
  double quat_w = quaternion.w;
  double phi = 2.0 * atan2(quat_z, quat_w);  
  return phi;
}

/*


bool SteeringController::serviceCallback(cwru_srv::simple_bool_service_messageRequest& request,
cwru_srv::simple_bool_service_messageResponse& response) {
    ROS_INFO("service callback activated");
    response.resp = true; 
    return true;
}
*/


void SteeringController::lin_steering_algorithm()
{
  double controller_speed;
  double controller_omega;
  
  
  
  double tx = cos(des_state_phi_);
  double ty = sin(des_state_phi_);
  double nx = -ty;
  double ny = tx;

  double heading_err;
  double lateral_err;
  double trip_dist_err;  

  
  
  double dx = des_state_x_ - g_odom_tf_x;
  double dy = des_state_y_ - g_odom_tf_y;

  
  
  
  lateral_err = dx * nx + dy * ny;
  trip_dist_err = dx * tx + dy * ty;

  
  heading_err =
      min_dang(des_state_phi_ - g_odom_tf_phi);  

  
  ROS_INFO("des_state_phi, odom_phi, heading err = %f, %f, %f", des_state_phi_, odom_phi_, heading_err);
  ROS_INFO("lateral err, trip dist err = %f, %f", lateral_err, trip_dist_err);
  
  
  
  
  
  
  
  

  
  

  
  controller_speed = des_state_vel_ + K_TRIP_DIST * trip_dist_err;  
  
  controller_omega = des_state_omega_ + K_PHI * heading_err + K_DISP * lateral_err;

  controller_omega = MAX_OMEGA * sat(controller_omega / MAX_OMEGA);  

  
  twist_cmd_.linear.x = controller_speed;
  twist_cmd_.angular.z = controller_omega;
  twist_cmd2_.twist = twist_cmd_;               
  twist_cmd2_.header.stamp = ros::Time::now();  
  cmd_publisher_.publish(twist_cmd_);
  cmd_publisher2_.publish(twist_cmd2_);
}

int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "steeringController");  

  ros::NodeHandle nh;  
  XformUtils xform_utils;

  OdomTf odomTf(&nh);  
  while (!odomTf.odom_tf_is_ready())
  {
    ROS_WARN("waiting on odomTf warm-up");
    ros::Duration(0.5).sleep();
    ros::spinOnce();
  }

  ROS_INFO("main: instantiating an object of type SteeringController");
  SteeringController steeringController(
      &nh);  

  ros::Rate sleep_timer(UPDATE_RATE);  

ROS_INFO:
  ("starting steering algorithm");
  
  geometry_msgs::PoseStamped est_st_pose_base_wrt_map;

  while (ros::ok())
  {
    ros::spinOnce();
    
    est_st_pose_base_wrt_map = xform_utils.get_pose_from_stamped_tf(odomTf.stfEstBaseWrtMap_);
    g_odom_tf_x = est_st_pose_base_wrt_map.pose.position.x;
    g_odom_tf_y = est_st_pose_base_wrt_map.pose.position.y;
    g_odom_tf_phi = xform_utils.convertPlanarQuat2Phi(est_st_pose_base_wrt_map.pose.orientation);
    steeringController.lin_steering_algorithm();  

    sleep_timer.sleep();
  }
  return 0;
}
