# learning_ros

This repository accompanies the text *A Systematic Approach to Learning Robot Programming with ROS*.
Code examples reside in folders corresponding to chapters.

This entire repository should be cloned under: `~/ros2_ws/src/` (assuming your ROS2 workspace is named `ros2_ws` and resides within your home directory).  To do so, navigate to `~/ros2_ws/src/` from a terminal and enter:
```
git clone https://github.com/wsnewman/learning_ros2.git
# currently: git clone https://bitbucket.org/jstarkman/learning_ros_jas.git
# or (with SSH): git clone git@bitbucket.org:jstarkman/learning_ros_jas.git
```
and also clone the external packages used with:
```
git clone https://github.com/wsnewman/learning_ros_external_packages.git
```

Then, from a terminal, navigate to `~/ros2_ws/` and compile the code with the commands:

```
. install/local_setup.bash
ament build -s # installs with symlinks
```

If you are installing ROS for the first time, see the instructions here:
[installation scripts](//github.com/wsnewman/learning_ros_setup_scripts)

The scripts located at this site automate installation of ROS (consistent with the version and packages used with the learning-ROS code examples).  These scripts also install a variety of useful tools.

If you are installing ROS2 for the first time, see the instructions [here](https://github.com/ros2/ros2/wiki/Linux-Install-Binary).  You may also wish to build ROS2 from source.
