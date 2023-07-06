#!/usr/bin/env python3
import geometry
import sys
import rclpy

from aqua2_interfaces.msg import AutopilotCommand
from geometry_msgs.msg import PoseWithCovarianceStamped

# Swim to the given x,y coordinate
def go_to_pos(node, x, y):
    """
    Steps:
        1. Given an x,y coordinate, find the angle needed to face it
        2. Given a current and target angle in degrees, rotate until the desired angle is reached
        2. Swim to the coordinate, adjusting as needed
    """
    should_quit = False

    publisher = node.create_publisher(AutopilotCommand, "/aqua/autopilot/command", 10)
    auto_msg = AutopilotCommand()

    def callback(msg):
        nonlocal publisher, auto_msg, should_quit, x, y
        position = msg.pose.pose.position
        target_angle = geometry.find_angle_to_face([round(position.x, 2), round(position.y, 2)], [x, y])
        auto_msg.target_yaw = target_angle
        
        auto_msg.surge = 0.3
        auto_msg.target_pitch = -0.3

        print(f"Current position: ({round(position.x,2): <10}, {round(position.y,2): <10}) Wanted position: ({x}, {y}) Needed angle: {target_angle: <20} degrees")
        
        buffer = 1.0
        x_ideal = (position.x < (x + buffer) and position.x > (x - buffer))
        y_ideal = (position.y < (y + buffer) and position.y > (y - buffer))
        if x_ideal and y_ideal:
            should_quit = True

        publisher.publish(auto_msg)

    subscriber = node.create_subscription(PoseWithCovarianceStamped, "/aqua/dvl_pose_estimate", callback, 10)

    while not should_quit:
        rclpy.spin_once(node)

def main():
    rclpy.init()
    node = rclpy.create_node("swim_to")

    if len(sys.argv) != 3:
        print("Correct usage: python3 swim_to <x> <y>")
        return

    go_to_pos(node, float(sys.argv[1]), float(sys.argv[2]))

if __name__ == "__main__":
    main()
