#include "example_tf_listener.hpp"
#include "example_tf_listener_fncs.cpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("demoTfListener");
  ROS_INFO("main: instantiating an object of type DemoTfListener");
  DemoTfListener demoTfListener(node);

  geometry_msgs::msg::TransformStamped stfBaseToLink2, stfBaseToLink1, stfLink1ToLink2;
  geometry_msgs::msg::TransformStamped testStfBaseToLink2;

  tf2::Transform tfBaseToLink1, tfLink1ToLink2, tfBaseToLink2, altTfBaseToLink2;

  tf2::TimePoint tp(std::chrono::seconds(0));

  stfBaseToLink1 = demoTfListener.tfBuffer_->lookupTransform("base_link", "link1", tp);
  std::cout << std::endl << "base to link1: " << std::endl;
  demoTfListener.printStampedTf(stfBaseToLink1);
  tfBaseToLink1 = demoTfListener.get_tf_from_stamped_tf(stfBaseToLink1);

  stfLink1ToLink2 = demoTfListener.tfBuffer_->lookupTransform("link1", "link2", tp);
  std::cout << std::endl << "link1 to link2: " << std::endl;
  demoTfListener.printStampedTf(stfLink1ToLink2);
  tfLink1ToLink2 = demoTfListener.get_tf_from_stamped_tf(stfLink1ToLink2);

  stfBaseToLink2 = demoTfListener.tfBuffer_->lookupTransform("base_link", "link2", tp);
  std::cout << std::endl << "base to link2: " << std::endl;
  demoTfListener.printStampedTf(stfBaseToLink2);
  tfBaseToLink2 = demoTfListener.get_tf_from_stamped_tf(stfBaseToLink2);

  std::cout << std::endl << "extracted tf: " << std::endl;
  demoTfListener.printTf(tfBaseToLink2);

  altTfBaseToLink2 = tfBaseToLink1 * tfLink1ToLink2;
  std::cout << std::endl << "result of multiply tfBaseToLink1*tfLink1ToLink2: " << std::endl;
  demoTfListener.printTf(altTfBaseToLink2);

  if (demoTfListener.multiply_stamped_tfs(stfBaseToLink1, stfLink1ToLink2, testStfBaseToLink2))
  {
    std::cout << std::endl << "testStfBaseToLink2:" << std::endl;
    demoTfListener.printStampedTf(testStfBaseToLink2);
  }
  std::cout << std::endl << "attempt multiply of stamped transforms in wrong order:" << std::endl;
  demoTfListener.multiply_stamped_tfs(stfLink1ToLink2, stfBaseToLink1, testStfBaseToLink2);

  geometry_msgs::msg::PoseStamped stPose, stPose_wrt_base;
  stPose = demoTfListener.get_pose_from_transform(stfLink1ToLink2);
  std::cout << std::endl << "pose link2 w/rt link1, from stfLink1ToLink2" << std::endl;
  demoTfListener.printStampedPose(stPose);

  // TODO blocked until tf2 API is more ROS2-friendly (needs action servers?)
  //   demoTfListener.tfBuffer_->transformPose("base_link", stPose, stPose_wrt_base);
  //   std::cout << std::endl << "pose of link2 transformed to base frame:" << std::endl;
  //   demoTfListener.printStampedPose(stPose_wrt_base);

  return 0;
}