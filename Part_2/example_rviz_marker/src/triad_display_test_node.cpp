






#include<ros/ros.h>
#include<geometry_msgs/PoseStamped.h>
#include<math.h>



geometry_msgs::Quaternion convertPlanarPsi2Quaternion(double psi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(psi / 2.0);
    quaternion.w = cos(psi / 2.0);
    return (quaternion);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "triad_display_test_node"); 
    ros::NodeHandle nh; 
    ros::Publisher pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("triad_display_pose", 1, true);
    geometry_msgs::PoseStamped desired_triad_pose;
    desired_triad_pose.pose.position.x = 0.0;
    desired_triad_pose.pose.position.y = 0.0;
    desired_triad_pose.pose.position.z = 0.0;
    desired_triad_pose.pose.orientation.x = 0.0;
    desired_triad_pose.pose.orientation.y = 0.0;
    desired_triad_pose.pose.orientation.z = 0.0;
    desired_triad_pose.pose.orientation.w = 1.0;
    desired_triad_pose.header.stamp = ros::Time::now();
    desired_triad_pose.header.frame_id = "world";
    
    double amp = 1.0;
    double phase = 0.0;
    double omega = 1.0;
    double vz = 0.05;
    double dt = 0.01;
    double x, y, z;
    z = 0;
    while (ros::ok()) {
        
        phase += omega*dt;
        if (phase > 2.0 * M_PI) phase -= 2.0 * M_PI;
        x = amp * sin(phase);
        y = amp * cos(phase);
        z += vz*dt;
        desired_triad_pose.pose.position.x = x;
        desired_triad_pose.pose.position.y = y;
        desired_triad_pose.pose.position.z = z;
        desired_triad_pose.pose.orientation = convertPlanarPsi2Quaternion(-phase);
        desired_triad_pose.header.stamp = ros::Time::now();
        
        pose_publisher.publish(desired_triad_pose);
        ros::Duration(dt).sleep();
    }
}
