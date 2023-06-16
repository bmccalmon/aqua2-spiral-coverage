# © Copyright Independent Robotics Inc. 2022
# Message used to give targets to the RoboDevel autopilot
# surge & heave are pass through given to gait
# target rpy are used in autopilot angles mode
# target yaw & depth are used in autopilot depth mode
# units for RPY are global-frame degrees, and depth is in meters

float32 surge
float32 heave
float32 target_roll
float32 target_pitch
float32 target_yaw
float32 target_depth