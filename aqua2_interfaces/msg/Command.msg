# © Copyright Independent Robotics Inc. 2022
# For sending raw command values to RoboDevel to translate into motion without using the autopilot.
# These values do not directly correspond to real-world values, ie there is no clear mapping to m/s or any other unit.
float32 speed # unitless pseudo-rate, [0, 1]
float32 yaw   # unitless pseudo-rate, [-1, 1] # +1: yaw right
float32 pitch # unitless pseudo-rate, [-1, 1] # +1: pitch forwards down
float32 roll  # unitless pseudo-rate, [-1, 1] # +1: roll right
float32 heave # unitless pseudo-rate, [-1, 1] # +1: heave up