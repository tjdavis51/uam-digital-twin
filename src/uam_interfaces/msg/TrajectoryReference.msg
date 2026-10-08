# header.stamp is the desired evaluation time; frame_id is odom.
std_msgs/Header header
uint64 sequence_id
uint32 frame_epoch
float64 trajectory_time_s
builtin_interfaces/Time valid_until
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 acceleration
float64 yaw
float64 yaw_rate
