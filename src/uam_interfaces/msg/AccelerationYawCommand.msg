# Generation time; frame_id is odom. Expired commands must not be refreshed.
std_msgs/Header header
builtin_interfaces/Time valid_until
uint64 sequence_id
uint64 state_sequence_id
uint64 reference_sequence_id
uint32 frame_epoch
# Inertial acceleration in m/s^2; no gravity term.
geometry_msgs/Vector3 acceleration
float64 yaw
float64 yaw_rate
