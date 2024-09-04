# Wing orientation data to correct the windvane orientation.
std_msgs/Header header
sail_interfaces/SailPosition source_sail       # enum
sail_interfaces/SailAngleType angle_type
float32 orientation     # sail orientation, degrees clockwise from dead ahead