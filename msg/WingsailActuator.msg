# ROS2 message to go with the WINGSAIL_ACTUATOR mavlink message (part of the ladon_robotics dialect)
std_msgs/Header header
sail_interfaces/SailPosition target_sail       # enum
sail_interfaces/SailAngleType sail_angle_type   # enum
float32 sail_angle      # angle in degrees
uint8 flap_active       # bitmask
float32[8] flap_angle   # deflection in degrees