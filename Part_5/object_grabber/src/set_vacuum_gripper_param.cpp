
#include <object_manipulation_properties/gripper_ID_codes.h>
#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gripper_ID_setter");
  ros::NodeHandle nh;
  int gripper_id = GripperIdCodes::STICKY_FINGERS;
  nh.setParam("gripper_ID", gripper_id);
}
