#!/usr/bin/env python3
# Stage 1: Circle the reef
import stabilize
import geometry

import rclpy

from movement_interfaces.msg import Deviation
from aqua2_interfaces.msg import AutopilotCommand
from geometry_msgs.msg import PoseWithCovarianceStamped

def checklist(node):
    node.get_logger().info("Setting to autopilot mode 2")
    stabilize.set_autopilot(node, 2)

def follow_boundary(node):
    x_deviation = 0.0
    y_deviation = 0.0

    publisher = node.create_publisher(AutopilotCommand, "/aqua/autopilot/command", 10)
    auto_msg = AutopilotCommand()

    def deviation_callback(msg):
        nonlocal x_deviation
        nonlocal y_deviation
        x_deviation = msg.x
        y_deviation = msg.y

    def main_callback(msg):
        nonlocal publisher
        nonlocal auto_msg
        nonlocal x_deviation
        nonlocal y_deviation
        orientation = msg.pose.pose.orientation
        roll, pitch, yaw = geometry.euler_from_quaternion(orientation.x, orientation.y, orientation.z, orientation.w)
        current_yaw = geometry.degrees_from_radians(yaw)

        # calculate yaw
        k = 0.001
        yaw = (k) * (x_deviation ** 3)
        if yaw >= 45:
            yaw = 45
        elif yaw <= -45:
            yaw = -45
        new_yaw = geometry.get_target_angle(current_yaw, yaw)

        # calculate surge
        max_surge = 0.3
        k = 0.0001
        b = 0.0008
        surge = (max_surge) - (k * (x_deviation ** 2))
                #+ b * (y_deviation ** 2))
        if surge < 0.0:
            surge = 0.0

        # plug into command
        auto_msg.target_yaw = new_yaw
        auto_msg.target_pitch = -0.3
        auto_msg.surge = surge

        #print("Surge: " + str(surge) + " x deviation: " + str(x_deviation) + " y deviation: " + str(y_deviation) + " Yaw: " + str(yaw))
        print(f"Surge: {surge: <25} x deviation: {x_deviation: <25} y deviation: {y_deviation: <25} Yaw: {yaw}")
        publisher.publish(auto_msg)

    subscriber_deviation = node.create_subscription(Deviation, "boundary_info", deviation_callback, 10)
    subscriber_pose = node.create_subscription(PoseWithCovarianceStamped, "/aqua/dvl_pose_estimate", main_callback, 10)

    rclpy.spin(node)

def main():
    rclpy.init()
    node = rclpy.create_node("cover_reef")

    node.get_logger().info("STARTING CHECKLIST")
    checklist(node)

    node.get_logger().info("Following boundary...")
    follow_boundary(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
