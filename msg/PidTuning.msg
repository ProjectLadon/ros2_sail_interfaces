# PID Tuning parameters
# Includes all parameters used by the following libraries:
# https://github.com/br3ttb/Arduino-PID-Library/
# https://github.com/Tellicious/ArduPID-Library
# https://github.com/PowerBroker2/ArduPID

uint8 id                # loop identifier for nodes with multiple PID controllers
string label            # loop label for nodes with multiple PID controllers 
float32 k_p             # proportional gain 
float32 k_i             # integral gain 
float32 k_d             # differential gain 
float32 n               # differential filter term (from https://github.com/Tellicious/ArduPID-Library)
float32 max_i           # integral saturation limit (absolute value)
float32 max_out         # maximum output value 
float32 min_out         # minimum output value 
float32 max_deadband    # maximum deadband value 
float32 min_deadband    # minimum deadband value 
float32 output_bias     # constant output bias 
int32 sample_time       # sample time, in milliseconds
bool direct             # if true, the output increases with increasing error; if false, decreases with increasing error 
bool active             # if true, the PID loop is active
bool prop_on_err        # if true, calculate proportional output on the error term (from https://github.com/br3ttb/Arduino-PID-Library/)

