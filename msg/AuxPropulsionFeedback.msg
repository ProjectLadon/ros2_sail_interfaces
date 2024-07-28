
uint8 DEPLOY_STATE_UNKNOWN      = 0
uint8 DEPLOY_STATE_RETRACTED    = 1
uint8 DEPLOY_STATE_RETRACTING   = 2
uint8 DEPLOY_STATE_DEPLOYING    = 3
uint8 DEPLOY_STATE_DEPLOYED     = 4
uint8 DEPLOY_STATE_FAILURE      = 5
uint8 DEPLOY_STATE_MAX          = 255 

uint8 motor_number
uint8 deploy_state
float32 throttle
float32 motor_speed