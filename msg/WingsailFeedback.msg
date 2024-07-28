# ROS2 message to go with the WINGSAIL_FEEDBACK mavlink message (part of the ladon_robotics dialect)
std_msgs/Header header

uint8 source_sail       # enum
float32 sail_angle      # degrees
float32 wind_angle      # degrees
uint8 flap_active       # bitmask
float32[8] flap_angle   # degrees