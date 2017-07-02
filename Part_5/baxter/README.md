# Baxter

Baxter itself runs ROS1.  To talk to it via the bridge, edit
`/path/to/your/ros2_ws/src/ros2/ros1_bridge/CMakeLists.txt`
such that it contains the new line below.

```cmake
include_directories(include ${generated_path})
# new line below (change the path accordingly)
include_directories(include /path/to/your/ros_ws/devel/include/)
```

# Unported

The following packages are unported:

* baxter_jnt_traj_ctlr_client (actionlib)
* baxter_launch_files (no code)
* baxter_playfile_nodes (actionlib)
* baxter_trajectory_streamer (actionlib)
