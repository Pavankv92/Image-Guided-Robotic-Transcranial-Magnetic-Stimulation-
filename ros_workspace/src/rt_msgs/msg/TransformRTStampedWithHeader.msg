# A joint state message with an additional rt timestamp and error useful for
# tracking camera poses.

std_msgs/Header header
string child_frame_id
time rt_stamp
std_msgs/Float64 error

geometry_msgs/Transform transform
