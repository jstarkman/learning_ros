#include <ros/ros.h> 
#include <visualization_msgs/Marker.h> 
#include <geometry_msgs/Point.h> 
#include <string.h>
#include <stdio.h>  
#include <example_rviz_marker/SimpleFloatSrvMsg.h> 
using namespace std;


double g_z_height = 0.0;
bool g_trigger = true;





bool displaySvcCB(example_rviz_marker::SimpleFloatSrvMsgRequest& request,
	example_rviz_marker::SimpleFloatSrvMsgResponse& response) {
    g_z_height = request.request_float32;
    ROS_INFO("example_rviz_marker: received request for height %f", g_z_height);
    g_trigger = true; 
    response.resp=true;
    return true;
}

<<<<<<< Updated upstream
void init_marker_vals(visualization_msgs::Marker &marker) {
    marker.header.frame_id = "/world"; // reference frame for marker coords
=======
int main(int argc, char **argv) {
    ros::init(argc, argv, "example_rviz_marker");
    ros::NodeHandle nh;
    ros::Publisher vis_pub = nh.advertise<visualization_msgs::Marker>("example_marker_topic", 0);
    visualization_msgs::Marker marker; 
    geometry_msgs::Point point; 
    
    
    ros::ServiceServer service = nh.advertiseService("rviz_marker_svc", displaySvcCB);


    marker.header.frame_id = "/world"; 
>>>>>>> Stashed changes
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = 0;
    
    
    marker.type = visualization_msgs::Marker::SPHERE_LIST; 
    marker.action = visualization_msgs::Marker::ADD;
    

    
    
    
    
    
    

    
    
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.02;
    marker.scale.y = 0.02;
    marker.scale.z = 0.02;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;     
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "example_rviz_marker");
    ros::NodeHandle nh;
    ros::Publisher vis_pub = nh.advertise<visualization_msgs::Marker>("example_marker_topic", 0);
    visualization_msgs::Marker marker; // instantiate a marker object
    geometry_msgs::Point point; // points will be used to specify where the markers go
    
    //set up a service to compute marker locations on request
    ros::ServiceServer service = nh.advertiseService("rviz_marker_svc", displaySvcCB);

    init_marker_vals(marker);
    
    double z_des;

    
    double x_min = -1.0;
    double x_max = 1.0;
    double y_min = -1.0;
    double y_max = 1.0;
    double dx_des = 0.1;
    double dy_des = 0.1;

    while (ros::ok()) {
        if (g_trigger) {  
            g_trigger = false; 
            z_des = g_z_height; 
            ROS_INFO("constructing plane of markers at height %f",z_des);
    	    marker.header.stamp = ros::Time();
            marker.points.clear(); 

            for (double x_des = x_min; x_des < x_max; x_des += dx_des) {
                for (double y_des = y_min; y_des < y_max; y_des += dy_des) {
                        point.x = x_des;
                        point.y = y_des;
                        point.z = z_des;
                        marker.points.push_back(point);
                }
            }
        }
     ros::Duration(0.1).sleep();
    
    vis_pub.publish(marker);
    ros::spinOnce();       
    }
    return 0;
}



