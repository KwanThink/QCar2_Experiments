## Driving command for QCar2 to directly control the Steering angle and Motor throttle
#std_msgs/Header header

# Names of whether to drive steering or throttle. 
# Supported names:
#   "steering_angle"
#   "desired_speed"
#   "desired_acceleration"

# Units are:
#   "rad"   for "steering_angle"
#   "m/s"   for "desired_speed"
#   "m/s^2" for "desired_acceleration"
string[] motor_names

# Values for the "command_names".
# The order must be identical to the "command_names".
# Units are:
#   "rad"   for "steering_angle"
#   "m/s"   for "desired_speed"
#   "m/s^2" for "desired_acceleration" 
float64[] values
