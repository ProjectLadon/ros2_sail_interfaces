# ROS2 message to go with the WIND_DATA mavlink message (part of the ladon_robotics dialect)
std_msgs/Header header
sail_interfaces/SailPosition source_sail   # enum
sail_interfaces/WindType wind_type     # enum
float32 speed       # wind speed, m/s
float32 direction   # wind direction, degrees