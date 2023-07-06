#!/usr/bin/env python3
# Stage 1: Circle the reef and record coordinates
import stabilize
import geometry

import rclpy
from movement_interfaces.msg import Deviation
from aqua2_interfaces.msg import AutopilotCommand
from geometry_msgs.msg import PoseWithCovarianceStamped

from collections import deque
import math
import matplotlib.pyplot as plt
import time

def checklist(node):
    stabilize.checklist(node)

def follow_boundary(node):
    boundary_points = deque() # can be used as a queue and stack
    x_deviation = y_deviation = 0.0
    x_pos = y_pos = 0.0

    publisher = node.create_publisher(AutopilotCommand, "/aqua/autopilot/command", 10)
    auto_msg = AutopilotCommand()

    # For debug
    def plot_points(points):
        # Extract x and y coordinates from the points list
        x_coords = [point[0] for point in points]
        y_coords = [point[1] for point in points]

        # Create a scatter plot
        plt.scatter(x_coords, y_coords)

        # Set the aspect ratio
        plt.xlim(-25, 35)
        plt.ylim(-25, 35)

        # Add labels and title
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Reef Boundary')

        # Display the plot
        plt.show()
        while True:
            pass

    halfway = False
    def store_points():
        nonlocal boundary_points, x_pos, y_pos, halfway
        print(f"x: {x_pos: <25} y: {y_pos: <25}")
        print(boundary_points)
        point = [round(x_pos, 2), round(y_pos, 2)]
        # append coordinate point to the deque
        if len(boundary_points) == 0:
            boundary_points.append(point)
            return
        distance = abs(math.sqrt(((point[0] - boundary_points[-1][0]) ** 2) + ((point[1] - boundary_points[-1][1]) ** 2)))
        if distance > 2.0:
            boundary_points.append(point)
        if halfway == False and boundary_points[-1][0] < boundary_points[0][0]:
            halfway = True
        if halfway == True and boundary_points[-1][0] > boundary_points[0][0]:
            # circle complete
            plot_points(boundary_points)
        print(f"Comparing {boundary_points[-1]} to {point}")
        print(f"Number of points: {len(boundary_points)}")

    def deviation_callback(msg):
        nonlocal x_deviation, y_deviation
        x_deviation = msg.x
        y_deviation = msg.y

    def main_callback(msg):
        nonlocal publisher, auto_msg, x_deviation, y_deviation, x_pos, y_pos
        
        position = msg.pose.pose.position
        x_pos = position.x
        y_pos = position.y

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
        #k = 0.0008
        surge = (max_surge) - (k * (x_deviation ** 2))
        if surge < 0.0:
            surge = 0.0

        # plug into command
        auto_msg.target_yaw = new_yaw
        auto_msg.target_pitch = -0.4
        auto_msg.surge = surge

        store_points()

        #print(f"Surge: {surge: <25} x deviation: {x_deviation: <25} y deviation: {y_deviation: <25} Yaw: {yaw}")
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
