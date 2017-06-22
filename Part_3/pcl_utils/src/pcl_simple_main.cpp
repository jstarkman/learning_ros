

#include <pcl_utils/pcl_utils.h>

using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcl_simple_main");
  ros::NodeHandle nh;
  ROS_INFO("instantiating a pclUtils object");
  PclUtils pclUtils(&nh);
  ROS_INFO("going into spin; try selecting points in rviz...");
  ros::spin();
  /*

  Eigen::Vector3f plane_normal;
  double plane_dist;
  while (ros::ok()) {
          if (pclUtils.got_selected_points() ) {
            ROS_INFO("transforming selected points");


            pclUtils.reset_got_selected_points();

            pclUtils.fit_xformed_selected_pts_to_plane(plane_normal, plane_dist);
            ROS_INFO_STREAM(" normal: "<<plane_normal.transpose()<<"; dist = "<<plane_dist);
          }
            ros::Duration(0.5).sleep();
          ros::spinOnce();
  }
  ROS_INFO("my work is done here!");
  */
}
