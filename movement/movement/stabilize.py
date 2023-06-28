#!/usr/bin/env python3
# Stabilizes Aqua2
import time
import geometry

import rclpy

from aqua2_interfaces.srv import GetBool
from aqua2_interfaces.srv import GetString
from aqua2_interfaces.srv import SetString
from aqua2_interfaces.srv import SetInt
from aqua2_interfaces.msg import Command
from aqua2_interfaces.msg import AutopilotCommand
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseWithCovarianceStamped

# sets Aqua2's autopilot mode
def set_autopilot(node, mode):
	"""
	0: off
	2: rpy
	4: depth and yaw
	"""
	if mode != 0 and mode != 2 and mode != 4:
	    node.get_logger().warning("Autopilot mode " + str(mode) + " is not a valid mode. Try 0 (off), 2 (rpy), or 4 (depth/yaw)")
	    return
	client = node.create_client(SetInt, "/aqua/autopilot/set_autopilot_mode")
	request = SetInt.Request()
	request.value = mode
	future = client.call_async(request)
	rclpy.spin_until_future_complete(node, future)

# checklist to ensure all systems are running properly
def checklist(node):
 
    # check if Aqua2 is calibrated
    def check_calibration():
        client = node.create_client(GetBool, "/aqua/system/is_calibrated")
        request = GetBool.Request()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        result = future.result()
        return result.value

    # calibrate Aqua2
    def calibrate():
        client = node.create_client(Empty, "/aqua/system/calibrate")
        request = Empty.Request()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)

    # check if Aqua2 is in swim mode
    def check_swim():
        time.sleep(0.5)
        client = node.create_client(GetString, "/aqua/system/get_mode")
        request = GetString.Request()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        result = future.result()
        return result.value

    # set to swim mode
    def set_swim():
        client = node.create_client(SetString, "/aqua/system/set_mode")
        request = SetString.Request()
        request.value = "swimmode"
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)
    
    # checks and returns what mode Aqua2 is in
    def check_autopilot():
        return 0

    # runs checks
    if check_calibration() == False:
        timer = 0.0
        v_timer = 0.0
        calibrate()
        node.get_logger().info("Calibrating...")
        while check_calibration() == False:
            time.sleep(0.1)
            timer = timer + 0.1
            v_timer = v_timer + 0.1
            if timer > 10.0:
                node.get_logger().warning("Taking longer than usual... Please check robot.")
                timer = 0
                calibrate()
        node.get_logger().info("Calibration completed in " + str(v_timer) + " seconds.")

    if check_swim() != "swimmode":
        set_swim()
        node.get_logger().info("Setting to swim mode...")
        while check_swim() == False:
            continue

    if check_autopilot() == 0:
        set_autopilot(node, 2)
        node.get_logger().info("Setting to autopilot mode 2")

def stabilize(node):
    quit = False

    def callback(msg):
        nonlocal quit

        orientation = msg.pose.pose.orientation
        roll, pitch, yaw = geometry.euler_from_quaternion(orientation.x, orientation.y, orientation.z, orientation.w)
        zone = 0.00872665 # radians
        roll_ideal = (roll < zone and roll > (zone * -1))
        pitch_ideal = (pitch < zone and pitch > (zone * -1))
        if roll_ideal and pitch_ideal:
            quit = True

        publisher = node.create_publisher(AutopilotCommand, "/aqua/autopilot/command", 10)
        msg = AutopilotCommand()
        msg.target_roll = 0.0
        msg.target_pitch = 0.0
        publisher.publish(msg)

    subscriber = node.create_subscription(PoseWithCovarianceStamped, "/aqua/dvl_pose_estimate", callback, 10)

    while not quit:
        rclpy.spin_once(node)

def main():
    rclpy.init()
    node = rclpy.create_node("stabilize")
    
    # startup Aqua2
    node.get_logger().info("STARTING CHECKLIST")
    checklist(node)
    
    # stabalize Aqua2
    node.get_logger().info("STARTING INITIAL STABILIZATION")
    stabilize(node)

    node.destroy_node()
    rclpy.shutdown()
    node.get_logger().info("Successfully stabilized")

if __name__ == "__main__":
    main()
