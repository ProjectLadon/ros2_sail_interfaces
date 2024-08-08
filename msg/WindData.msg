# ROS2 message to go with the WIND_DATA mavlink message (part of the ladon_robotics dialect)
std_msgs/Header header

uint8 WIND_DATA_TYPE_RAW        = 0     # The wind is raw and unprocessed from the sensor
uint8 WIND_DATA_TYPE_APPARENT   = 1     # The wind data is relative to the boat 
uint8 WIND_DATA_TYPE_TRUE       = 2     # The wind data is relative to the ground 
uint8 WIND_DATA_TYPE_INVALID    = 255   # The wind is invalid 

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

uint8 source_sail   # enum
uint8 wind_type     # enum
float32 speed       # wind speed, m/s
float32 direction   # wind direction, degrees