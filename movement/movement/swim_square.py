#!/usr/bin/env python3
# Make the robot swim in a square
import stabilize
import geometry

import rclpy
import time

from aqua2_interfaces.msg import AutopilotCommand
from geometry_msgs.msg import PoseWithCovarianceStamped

def checklist(node):
    node.get_logger().info("Setting to autopilot mode 2")
    stabilize.set_autopilot(node, 2)

def swim_square(node, edge_length, depth):
    should_quit = False
    should_turn = False
    new_target_yaw = True
    target_yaw = 0.0
    edge_timer = 0.0

    def callback(msg):
        nonlocal should_quit
        nonlocal should_turn
        nonlocal new_target_yaw
        nonlocal target_yaw
        nonlocal edge_timer

        orientation = msg.pose.pose.orientation
        roll, pitch, yaw = geometry.euler_from_quaternion(orientation.x, orientation.y, orientation.z, orientation.w)

        publisher = node.create_publisher(AutopilotCommand, "/aqua/autopilot/command", 10)
        msg = AutopilotCommand()

        if should_turn == False:
            # if needing to go straight
            node.get_logger().info("Going straight...")
            msg.surge = 1.0
            msg.target_pitch = -0.3
            msg.target_roll = 0.0
            #msg.target_depth = depth
            msg.target_yaw = target_yaw
            publisher.publish(msg)

            time.sleep(0.1)
            edge_timer = edge_timer + 0.1

            if edge_timer > edge_length:
                edge_timer = 0
                should_turn = True
        else:
            # if went straight for enough time and needing to turn
            node.get_logger().info("Turning...")
            yaw_degrees = geometry.degrees_from_radians(yaw)
            
            if new_target_yaw:
                target_yaw = geometry.get_target_angle(yaw_degrees, 90)
                new_target_yaw = False
            
            msg.target_yaw = target_yaw
            msg.target_pitch = -0.3
            msg.target_roll = 0.0
            #msg.target_depth = depth
            publisher.publish(msg)

            time.sleep(0.1)

            buffer = 2 # degrees
            yaw_ideal = (yaw_degrees < (target_yaw + buffer) and yaw_degrees > (target_yaw - buffer))

            if yaw_ideal:
                should_turn = False
                new_target_yaw = True

    subscriber = node.create_subscription(PoseWithCovarianceStamped, "/aqua/dvl_pose_estimate", callback, 10)
    
    while not should_quit:
        rclpy.spin_once(node)

def main():
    rclpy.init()
    node = rclpy.create_node("swim_square")
    
    node.get_logger().info("STARTING CHECKLIST")
    checklist(node)

    node.get_logger().info("Swimming in a square...")
    swim_square(node, 10.0, 1.0)

    node.get_logger().info("Successfully swam in a square")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
