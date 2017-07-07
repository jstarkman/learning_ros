#ifndef OBJECT_MANIPULATION_PROPERTIES_H
#define OBJECT_MANIPULATION_PROPERTIES_H

#include <vector>

#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "object_manipulation_msgs/srv/query.hpp"
#include "object_manipulation_properties/gripper_id_codes.hpp"
#include "object_manipulation_properties/object_id_codes.hpp"
#include "rclcpp/rclcpp.hpp"
#include "xform_utils/xform_utils.hpp"

#include "rcutils/logging_macros.h"
#define ROS_INFO RCUTILS_LOG_INFO
#define ROS_WARN RCUTILS_LOG_WARN

class ObjectManipulationProperties
{
private:
  XformUtils xformUtils;

public:
  ObjectManipulationProperties(void);
  bool get_object_info(int object_id, Eigen::Affine3d &grasp_transform, double &approach_dist,
                       double &gripper_close_test);
};
#endif
