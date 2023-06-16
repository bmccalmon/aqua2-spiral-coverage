# Stabalizes Aqua2
import time

import rclpy

from aqua2_interfaces.srv import GetBool
from aqua2_interfaces.srv import GetString
from aqua2_interfaces.srv import SetString
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseWithCovarianceStamped

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
    
    # runs checks
    if check_calibration() == False:
        timer = 0.0
        calibrate()
        node.get_logger().info("Calibrating...")
        while check_calibration() == False:
            time.sleep(0.1)
            timer = timer + 0.1    
            if timer > 5.0:
                node.get_logger().info("Taking longer than usual... Please check robot.")
                timer = 0
                calibrate()
        node.get_logger().info("Calibration completed in " + str(timer) + " seconds.")

    if check_swim() != "swimmode":
        set_swim()
        node.get_logger().info("Setting to swim mode...")
        while check_swim() == False:
            continue

def determine_ranges(node):
    
    def callback(msg):
        orientation = msg.pose.pose.orientation

    subscriber = node.create_subscription(PoseWithCovarianceStamped, "/aqua/dvl_pose_estimate", callback, 10)

    rclpy.spin_once(node)

def main():
    rclpy.init()
    node = rclpy.create_node("stabalize")
    
    # startup Aqua2
    checklist(node)
    
    #determine_ranges(node)

    node.destroy_node()
    rclpy.shutdown()
    node.get_logger().info("Successfully shutdown")

if __name__ == "__main__":
    main()
