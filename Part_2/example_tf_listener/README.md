# example_tf_listener

This code illustrates use of a transform listener, using the mobot model to refer to tf frames.
Shows how to interpret and multiply transforms, and how to transform poses.

## Example usage: ROS1 original

start gazebo: `roslaunch gazebo_ros empty_world.launch`

launch the mobot: `roslaunch mobot_urdf mobot_w_arm_and_jnt_pub.launch`

start a state publisher: `rosrun robot_state_publisher robot_state_publisher`

start the transform listener: `rosrun example_tf_listener example_tf_listener`

### ROS2

Since Gazebo is not currently available for ROS2 (by bridge or natively), this package does not serve much of a purpose.  To run it anyways, execute `example_tf_listener`.

Reference: https://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20listener%20%28C%2B%2B%29

Note that the original `tf` has been deprecated since ROS1 Hydro and is not available for ROS2.
