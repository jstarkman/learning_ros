#include "object_manipulation_properties/object_manipulation_properties.hpp"

int main(int argc, char** argv)
{
  // ros::init(argc, argv, "object_manip_prop_test_main");
  ObjectManipulationProperties objectManipulationProperties;

  Eigen::Affine3d grasp_transform;
  double approach_dist, gripper_test_val;
  int object = ObjectIdCodes::TOY_BLOCK_ID;

  std::cout << "enter object ID code (try 1000): ";
  std::cin >> object;

  if (objectManipulationProperties.get_object_info(object, grasp_transform, approach_dist, gripper_test_val))
  {
    std::cout << "approach_dist = " << approach_dist << std::endl;
    std::cout << "gripper test val= " << gripper_test_val << std::endl;
    std::cout << "transform origin: " << grasp_transform.translation().transpose() << std::endl;
  }
  else
  {
    ROS_WARN("object ID not recognized");
  }
}
