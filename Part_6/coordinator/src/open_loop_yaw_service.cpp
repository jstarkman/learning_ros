

#include <coordinator/OpenLoopNavSvc.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <iostream>
#include <string>
using namespace std;

double MAX_SPEED = 1.0;
double ACCEL = 1.0;
double T_UP_MAX = MAX_SPEED / ACCEL;
double T_DOWN_MAX = T_UP_MAX;
double RAMP_UP_DIST = 0.5 * T_UP_MAX * MAX_SPEED;
double TRIANGULAR_DIST = 2.0 * RAMP_UP_DIST;
double DT = 0.01;

ros::Publisher g_twist_commander;
geometry_msgs::Twist g_twist;

double sgn(double x)
{
  if (x > 0.0)
  {
    return 1.0;
  }
  else if (x < 0.0)
  {
    return -1.0;
  }
  else
  {
    return 0.0;
  }
}

bool callback(coordinator::OpenLoopNavSvcRequest& request, coordinator::OpenLoopNavSvcResponse& response)
{
  ROS_INFO("open-loop yaw service callback activated");
  double move_dist = request.move_distance;
  ROS_INFO("requested yaw rotation:  %f", move_dist);

  double sign = sgn(move_dist);
  double t_const;
  double t_up_triangular;
  double t = 0.0;
  ros::Rate timer(1.0 / DT);

  if (fabs(move_dist) <= TRIANGULAR_DIST)
  {
    t_up_triangular = sqrt(fabs(move_dist) / ACCEL);
    g_twist.angular.z = 0;
    while (t < t_up_triangular)
    {
      t += DT;
      g_twist.angular.z += sign * ACCEL * DT;
      g_twist_commander.publish(g_twist);
      timer.sleep();
    }

    while (t < 2.0 * t_up_triangular)
    {
      t += DT;
      g_twist.angular.z -= sign * ACCEL * DT;
      g_twist_commander.publish(g_twist);
      timer.sleep();
    }

    g_twist.angular.z = 0.0;
    g_twist_commander.publish(g_twist);
    return true;
  }

  else
  {
    t_const = (fabs(move_dist) - 2.0 * RAMP_UP_DIST) / MAX_SPEED;
    g_twist.angular.z = MAX_SPEED;
    while (t < T_UP_MAX)
    {
      t += DT;
      g_twist.angular.z += sign * ACCEL * DT;
      g_twist_commander.publish(g_twist);
      timer.sleep();
    }

    while (t < T_UP_MAX + t_const)
    {
      t += DT;
      g_twist.angular.z = sign * MAX_SPEED;
      g_twist_commander.publish(g_twist);
      timer.sleep();
    }

    while (t < T_UP_MAX + t_const + T_DOWN_MAX)
    {
      t += DT;
      g_twist.angular.z -= sign * ACCEL * DT;
      g_twist_commander.publish(g_twist);
      timer.sleep();
    }
    g_twist.angular.z = 0.0;
    g_twist_commander.publish(g_twist);
  }
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "open_loop_yaw_service");
  ros::NodeHandle n;
  g_twist_commander = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

  g_twist.angular.x = 0;
  g_twist.angular.y = 0;
  g_twist.angular.z = 0;
  g_twist.linear.x = 0.0;
  g_twist.linear.y = 0.0;
  g_twist.linear.z = 0.0;
  ros::ServiceServer service = n.advertiseService("open_loop_yaw_service", callback);
  ros::spin();

  return 0;
}
