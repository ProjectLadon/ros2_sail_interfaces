# ROS2 message to go with the WINGSAIL_ACTUATOR mavlink message (part of the ladon_robotics dialect)
std_msgs/Header header

uint8 SAIL_POS_ID_SINGLE        = 0     # A single sail on the boat 
uint8 SAIL_POS_ID_1             = 1     # Wingsail #1 
uint8 SAIL_POS_ID_2             = 2     # Wingsail #2 
uint8 SAIL_POS_ID_3             = 3     # Wingsail #3 
uint8 SAIL_POS_ID_4             = 4     # Wingsail #4 
uint8 SAIL_POS_ID_5             = 5     # Wingsail #5 
uint8 SAIL_POS_ID_6             = 6     # Wingsail #6 
uint8 SAIL_POS_ID_7             = 7     # Wingsail #7 
uint8 SAIL_POS_ID_8             = 8     # Wingsail #8 
uint8 SAIL_POS_ID_FORE          = 20    # The forward most sail on a boat 
uint8 SAIL_POS_ID_MAIN          = 21    # The middle sail on a boat 
uint8 SAIL_POS_ID_MIZZEN        = 22    # The aft most sail on a boat 
uint8 SAIL_POS_HULL             = 101   # Indicates a source fixed to the hull
uint8 SAIL_POS_MIXED            = 201   # Indicates a mixed value
uint8 SAIL_POS_ID_NONE          = 255   # No wingsail defined 

uint8 SAIL_ANGLE_TYPE_AOA       = 1     # The angle is an angle of attack 
uint8 SAIL_ANGLE_TYPE_ROT       = 2     # The angle is a rotation angle

uint8 target_sail       # enum
uint8 sail_angle_type   # enum
float32 sail_angle      # angle in degrees
uint8 flap_active       # bitmask
float32[8] flap_angle   # deflection in degrees