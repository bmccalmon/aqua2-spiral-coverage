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
    x_deviation = 0.1
    y_deviation = 0.1

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
        orientation = msg.pose.pose.orientation
        roll, pitch, yaw = geometry.euler_from_quaternion(orientation.x, orientation.y, orientation.z, orientation.w)
        yaw = geometry.degrees_from_radians(yaw)
        add_yaw = (x_deviation * 1) + (y_deviation * -10)
        max_turn_degree = 10.0
        if add_yaw > max_turn_degree:
            add_yaw = max_turn_degree
        elif add_yaw < -1 * max_turn_degree:
            add_yaw = -1 * max_turn_degree
        target_yaw = geometry.get_target_angle(yaw, add_yaw)
        auto_msg.target_yaw = target_yaw
        auto_msg.target_pitch = -0.3
        auto_msg.target_roll = 0.0
        surge = 1/abs(x_deviation*50)
        if surge > 0.05:
            surge = 0.05
        auto_msg.surge = surge
        print("Current surge: " + str(surge))
        print("Deviation x: " + str(x_deviation) + ", y: " + str(y_deviation))
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
