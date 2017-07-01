

#include <example_interactive_marker/ImNodeSvcMsg.h>
#include <geometry_msgs/Point.h>
#include <interactive_markers/interactive_marker_server.h>
#include <ros/ros.h>
#include <iostream>

const int IM_GET_CURRENT_MARKER_POSE = 0;
const int IM_SET_NEW_MARKER_POSE = 1;

geometry_msgs::Point g_current_point;
geometry_msgs::Quaternion g_current_quaternion;
ros::Time g_marker_time;

interactive_markers::InteractiveMarkerServer* g_IM_server;
visualization_msgs::InteractiveMarkerFeedback* g_IM_feedback;

bool IM6DofSvcCB(example_interactive_marker::ImNodeSvcMsgRequest& request,
                 example_interactive_marker::ImNodeSvcMsgResponse& response)
{
  if (request.cmd_mode == IM_GET_CURRENT_MARKER_POSE)
  {
    ROS_INFO("IM6DofSvcCB: rcvd request for query--GET_CURRENT_MARKER_POSE");
    response.poseStamped_IM_current.header.stamp = g_marker_time;
    response.poseStamped_IM_current.header.frame_id = "world";
    response.poseStamped_IM_current.pose.position = g_current_point;
    response.poseStamped_IM_current.pose.orientation = g_current_quaternion;
    return true;
  }

  if (request.cmd_mode == IM_SET_NEW_MARKER_POSE)
  {
    geometry_msgs::PoseStamped poseStamped_IM_desired;
    ROS_INFO("IM6DofSvcCB: rcvd request for action--SET_NEW_MARKER_POSE");
    g_current_point = request.poseStamped_IM_desired.pose.position;
    g_current_quaternion = request.poseStamped_IM_desired.pose.orientation;
    g_marker_time = ros::Time::now();
    poseStamped_IM_desired = request.poseStamped_IM_desired;
    poseStamped_IM_desired.header.stamp = g_marker_time;
    response.poseStamped_IM_current = poseStamped_IM_desired;

    response.poseStamped_IM_current.header.stamp = g_marker_time;
    response.poseStamped_IM_current.header.frame_id = "torso";
    response.poseStamped_IM_current.pose.position = g_current_point;
    response.poseStamped_IM_current.pose.orientation = g_current_quaternion;
    g_IM_server->setPose("des_hand_pose", poseStamped_IM_desired.pose);
    g_IM_server->applyChanges();
    return true;
  }
  ROS_WARN("IM6DofSvcCB: case not recognized");
  return false;
}

void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  ROS_INFO_STREAM(feedback->marker_name << " is now at " << feedback->pose.position.x << ", "
                                        << feedback->pose.position.y << ", " << feedback->pose.position.z);
  g_current_quaternion = feedback->pose.orientation;
  g_current_point = feedback->pose.position;
  g_marker_time = ros::Time::now();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_marker");
  ros::NodeHandle nh;

  interactive_markers::InteractiveMarkerServer server("rt_hand_marker");
  g_IM_server = &server;
  ros::ServiceServer IM_6dof_interface_service = nh.advertiseService("IM6DofSvc", &IM6DofSvcCB);

  visualization_msgs::InteractiveMarker int_marker;

  int_marker.header.frame_id = "world";
  int_marker.name = "des_hand_pose";
  int_marker.description = "Interactive Marker";

  geometry_msgs::Point temp_point_start;
  /** specify/push-in the origin for this marker */
  temp_point_start.x = 0.5;
  temp_point_start.y = -0.5;
  temp_point_start.z = 0.2;
  g_current_point.x = temp_point_start.x;
  g_current_point.y = temp_point_start.y;
  g_current_point.z = temp_point_start.z;

  visualization_msgs::Marker arrow_marker_x;
  geometry_msgs::Point temp_point;

  arrow_marker_x.type = visualization_msgs::Marker::ARROW;

  temp_point.x = temp_point.y = temp_point.z = 0;
  arrow_marker_x.points.push_back(temp_point);

  temp_point = temp_point_start;
  temp_point.x = 0.2;
  temp_point.y = 0.0;
  temp_point.z = 0.0;
  arrow_marker_x.points.push_back(temp_point);

  arrow_marker_x.scale.x = 0.01;
  arrow_marker_x.scale.y = 0.01;
  arrow_marker_x.scale.z = 0.01;

  arrow_marker_x.color.r = 1.0;
  arrow_marker_x.color.g = 0.0;
  arrow_marker_x.color.b = 0.0;
  arrow_marker_x.color.a = 1.0;

  visualization_msgs::Marker arrow_marker_y;
  arrow_marker_y.type = visualization_msgs::Marker::ARROW;

  temp_point.x = temp_point.y = temp_point.z = 0;
  arrow_marker_y.points.push_back(temp_point);

  temp_point.x = 0.0;
  temp_point.y = 0.2;
  temp_point.z = 0.0;
  arrow_marker_y.points.push_back(temp_point);

  arrow_marker_y.scale.x = 0.01;
  arrow_marker_y.scale.y = 0.01;
  arrow_marker_y.scale.z = 0.01;

  arrow_marker_y.color.r = 0.0;
  arrow_marker_y.color.g = 1.0;
  arrow_marker_y.color.b = 0.0;
  arrow_marker_y.color.a = 1.0;

  visualization_msgs::Marker arrow_marker_z;
  arrow_marker_z.type = visualization_msgs::Marker::ARROW;

  temp_point.x = temp_point.y = temp_point.z = 0;
  arrow_marker_z.points.push_back(temp_point);

  temp_point.x = 0.0;
  temp_point.y = 0.0;
  temp_point.z = 0.2;
  arrow_marker_z.points.push_back(temp_point);

  arrow_marker_z.scale.x = 0.01;
  arrow_marker_z.scale.y = 0.01;
  arrow_marker_z.scale.z = 0.01;

  arrow_marker_z.color.r = 0.0;
  arrow_marker_z.color.g = 0.0;
  arrow_marker_z.color.b = 1.0;
  arrow_marker_z.color.a = 1.0;
  /**/

  visualization_msgs::InteractiveMarkerControl IM_control;
  IM_control.always_visible = true;

  IM_control.markers.push_back(arrow_marker_x);
  IM_control.markers.push_back(arrow_marker_y);
  IM_control.markers.push_back(arrow_marker_z);

  int_marker.controls.push_back(IM_control);

  visualization_msgs::InteractiveMarkerControl translate_control_x;
  translate_control_x.name = "move_x";
  translate_control_x.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;

  /** Create the Z-Axis Control*/
  visualization_msgs::InteractiveMarkerControl translate_control_z;
  translate_control_z.name = "move_z";
  translate_control_z.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  translate_control_z.orientation.x = 0;
  translate_control_z.orientation.y = 1;
  translate_control_z.orientation.z = 0;
  translate_control_z.orientation.w = 1;

  /** Create the Y-Axis Control*/
  visualization_msgs::InteractiveMarkerControl translate_control_y;
  translate_control_y.name = "move_y";
  translate_control_y.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  translate_control_y.orientation.x = 0;
  translate_control_y.orientation.y = 0;
  translate_control_y.orientation.z = 1;
  translate_control_y.orientation.w = 1;

  /**/
  visualization_msgs::InteractiveMarkerControl rotx_control;
  rotx_control.always_visible = true;
  rotx_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  rotx_control.orientation.x = 1;
  rotx_control.orientation.y = 0;
  rotx_control.orientation.z = 0;
  rotx_control.orientation.w = 1;
  rotx_control.name = "rot_x";

  visualization_msgs::InteractiveMarkerControl rotz_control;
  rotz_control.always_visible = true;
  rotz_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  rotz_control.orientation.x = 0;
  rotz_control.orientation.y = 1;
  rotz_control.orientation.z = 0;
  rotz_control.orientation.w = 1;
  rotz_control.name = "rot_z";

  visualization_msgs::InteractiveMarkerControl roty_control;
  roty_control.always_visible = true;
  roty_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  roty_control.orientation.x = 0;
  roty_control.orientation.y = 0;
  roty_control.orientation.z = 1;
  roty_control.orientation.w = 1;
  roty_control.name = "rot_y";
  /**/

  int_marker.controls.push_back(translate_control_x);
  int_marker.controls.push_back(translate_control_y);
  int_marker.controls.push_back(translate_control_z);
  int_marker.controls.push_back(rotx_control);
  int_marker.controls.push_back(rotz_control);
  int_marker.controls.push_back(roty_control);

  /** Scale Down: this makes all of the arrows/disks for the user controls smaller than the default size */
  int_marker.scale = 0.2;

  int_marker.pose.position.x = temp_point_start.x;
  int_marker.pose.position.y = temp_point_start.y;
  int_marker.pose.position.z = temp_point_start.z;

  g_IM_server->insert(int_marker, &processFeedback);

  g_IM_server->applyChanges();

  ROS_INFO("going into spin...");
  ros::spin();
}
