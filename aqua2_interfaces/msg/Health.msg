# © Copyright Independent Robotics Inc. 2022
# Health Monitor Message
# Contains sensor readings that give insights on status of low level hardware
std_msgs/Header header

# Voltage used by entire robot, sensor on motor driver board
float32 voltage

# Current used by entire robot, sensor on motor driver board
float32 current

# Voltage of entire robot averaged over preset time window
float32 avgvoltage

# Current of entire robot averaged over preset time window
float32 avgcurrent

# Power of entire robot averaged over preset time window
float32 avgpower

# Battery status
float32 battery_dod

# Temperature sensor for motor driver board on the robot
float32 mdbtemp

# temperature sensors for leg motors
float32[6] leg_temps