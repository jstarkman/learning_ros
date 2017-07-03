

#include <actionlib/client/simple_action_client.h>
#include <object_manipulation_properties/object_ID_codes.h>
#include <part_fetcher/PartFetcherAction.h>
#include <ros/ros.h>

void set_example_object_frames(geometry_msgs::PoseStamped& object_poseStamped,
                               geometry_msgs::PoseStamped& object_dropoff_poseStamped)
{
  object_poseStamped.header.frame_id = "system_ref_frame";
  object_poseStamped.pose.position.x = -0.85;
  object_poseStamped.pose.position.y = 0.85;
  object_poseStamped.pose.position.z = 0.5622;
  object_poseStamped.pose.orientation.x = 0;
  object_poseStamped.pose.orientation.y = 0;
  object_poseStamped.pose.orientation.z = 0.0;
  object_poseStamped.pose.orientation.w = 1;
  object_poseStamped.header.stamp = ros::Time::now();

  object_dropoff_poseStamped = object_poseStamped;
  object_dropoff_poseStamped.pose.position.x = 1;
  object_dropoff_poseStamped.pose.position.y = 0;
  object_dropoff_poseStamped.pose.position.z = 0.793;
}

void doneCb(const actionlib::SimpleClientGoalState& state, const part_fetcher::PartFetcherResultConstPtr& result)
{
  ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
  ROS_INFO("got result rtn_val = %d", result->rtn_code);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "part_fetcher_client_node");
  ros::NodeHandle nh;

  part_fetcher::PartFetcherGoal goal;
  geometry_msgs::PoseStamped object_pickup_poseStamped;
  geometry_msgs::PoseStamped object_dropoff_poseStamped;

  set_example_object_frames(object_pickup_poseStamped, object_dropoff_poseStamped);

  actionlib::SimpleActionClient<part_fetcher::PartFetcherAction> action_client("part_fetcher", true);

  ROS_INFO("waiting for server: ");
  bool server_exists = action_client.waitForServer(ros::Duration(5.0));

  if (!server_exists)
  {
    ROS_WARN("could not connect to server; halting");
    return 0;
  }

  ROS_INFO("connected to action server");

  goal.object_id = ObjectIdCodes::TOY_BLOCK_ID;
  goal.object_frame = object_pickup_poseStamped;
  goal.desired_frame = object_dropoff_poseStamped;

  action_client.sendGoal(goal, &doneCb);

  bool finished_before_timeout = action_client.waitForResult(ros::Duration(20.0));

  if (!finished_before_timeout)
  {
    ROS_WARN("giving up waiting on result ");
    return 0;
  }

  return 0;
}
