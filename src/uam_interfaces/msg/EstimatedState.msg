# Translation sample time in normalized ROS time; all positions/velocities in odom.
std_msgs/Header header
builtin_interfaces/Time received_at
builtin_interfaces/Time attitude_sample_time
builtin_interfaces/Time angular_velocity_sample_time
builtin_interfaces/Time validity_sample_time
uint64 sequence_id
uint32 frame_epoch
string child_frame_id
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
# xyzw quaternion maps base_link FLU vectors into odom ENU.
geometry_msgs/Quaternion orientation
# Body FLU radians/second.
geometry_msgs/Vector3 angular_velocity
bool position_xy_valid
bool position_z_valid
bool velocity_xy_valid
bool velocity_z_valid
bool attitude_valid
bool angular_velocity_valid
bool time_sync_valid
uint8 odometry_reset_counter
uint8 position_xy_reset_counter
uint8 position_z_reset_counter
uint8 velocity_xy_reset_counter
uint8 velocity_z_reset_counter
uint8 heading_reset_counter
