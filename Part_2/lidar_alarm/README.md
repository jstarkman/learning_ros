# lidar_alarm

A simple, illustrative program to show how to subscribe to a LIDAR signal and interpret the signal.
This code subscribes to topic `robot0/laser_0`, which is published by the Simple 2-D Robot simulator.
The signal interpretation in this example merely looks at a single ping--straight ahead.  If this
ping distance is less than some danger threshold, the lidar listener publishers a warning signal on
topic `lidar_alarm`.  The distance of the forward ping is also published, on topic `lidar_dist`.

To make this example code more useful, interpretation of the LIDAR data should be more general than a
single ping--e.g., examining a forward-motion corridor for evaluating safe forward motion with consideration
of the width of the robot.

## Example usage

Start up the STDR simulator (broken in ROS2):

`roslaunch stdr_launchers server_with_map_and_gui_plus_robot.launch`

Start the lidar alarm node:

`$ lidar_alarm`

Have the controller code monitor the `lidar_alarm` topic and do something intelligent with the information.

## Example ROS1 solution

[This repo](https://github.com/jstarkman/p2_jas497) shows an example of how to make the LIDAR alarm more intelligent.  It also includes a wall follower.
