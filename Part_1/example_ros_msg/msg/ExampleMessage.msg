# here are some primitives that can be used:
# int8, int16, int32, int64 (plus uint*)
# float32, float64
# string
# other msg files (with packages)
# (e.g., ``builtin_interfaces/Time timestamp_name_here``)
# variable-length array[]
# fixed-length array[C]
# upper-bound variable-length array[<=5]

# https://github.com/ros2/common_interfaces/blob/master/std_msgs/msg/Header.msg
std_msgs/Header header
# example of a ROS2-only default value
int32 demo_int -1
float64 demo_double
