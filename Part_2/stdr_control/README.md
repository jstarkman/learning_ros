# stdr_control

A minimal example node to illustrate control of the STDR mobile robot with open-loop commands.

## Example usage

This requires full ROS1 and ROS2 installs.  Run each of the following in its own terminal.
Lines beginning with `ros1` have had a ROS1 setup file sourced, while lines beginning with
`ros2` have had `local_setup.*sh` sourced.

```
ros1$ roscore
ros1 and ros2$ dynamic_bridge --bridge-all-topics
ros1$ roslaunch stdr_launchers server_with_map_and_gui_plus_robot.launch
ros2$ lidar_alarm
ros2$ reactive_commander
```

### `dynamic_bridge`

This is from the package `ros1_bridge`.  Building that package has been known to require
large amounts of memory, and as such one should follow the special instructions given
[here](https://github.com/ros2/ros1_bridge).
