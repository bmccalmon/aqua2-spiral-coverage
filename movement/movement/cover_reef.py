#!/usr/bin/env python3
# Stage 1: Circle the reef and record coordinates
import stabilize
import geometry
import swim_to
import visualize
import save_image

import rclpy
from movement_interfaces.msg import Deviation
from aqua2_interfaces.msg import AutopilotCommand
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

from collections import deque
import math
import matplotlib.pyplot as plt
import time
import pickle

def checklist(node):
    stabilize.checklist(node)

def follow_boundary(node):
    boundary_points = deque() # can be used as a queue and stack
    x_deviation = y_deviation = 0.0
    x_pos = y_pos = 0.0

    publisher = node.create_publisher(AutopilotCommand, "/aqua/autopilot/command", 10)
    auto_msg = AutopilotCommand()

    halfway = False
    lap_complete = False
    def store_points():
        nonlocal boundary_points, x_pos, y_pos, halfway, lap_complete
        #print(f"x: {x_pos: <25} y: {y_pos: <25}")
        #print(boundary_points)
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
        if halfway == True and x_pos > boundary_points[0][0]:
            # circle complete
            lap_complete = True
        #print(f"Comparing {boundary_points[-1]} to {point}")
        #print(f"Number of points: {len(boundary_points)}")

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
        auto_msg.target_pitch = -0.3 # -0.5
        auto_msg.surge = surge

        store_points()

        #print(f"Surge: {surge: <25} x deviation: {x_deviation: <25} y deviation: {y_deviation: <25} Yaw: {yaw}")
        publisher.publish(auto_msg)

    subscriber_deviation = node.create_subscription(Deviation, "boundary_info", deviation_callback, 10)
    subscriber_pose = node.create_subscription(Odometry, "/aqua/simulator/pose", main_callback, 10)
    #subscriber_pose = node.create_subscription(PoseWithCovarianceStamped, "/aqua/dvl_pose_estimate", main_callback, 10)

    while not lap_complete:
        rclpy.spin_once(node)

    with open("sim_map.pickle", "wb") as file:
        pickle.dump(boundary_points, file)

    return boundary_points

def spiral_inside(node):
    with open("sim_map.pickle", "rb") as file:
        boundary = pickle.load(file)
    # Generate a list of rings to follow
    rings = geometry.get_rings(boundary, 64, 7)
    #rings.popleft() # remove outer ring since we already traversed it
    #visualize.plot_points(rings)
    node = node
    while len(rings) > 0:
        while len(rings[0]) > 0:
            target = rings[0].popleft()
            swim_to.go_to_pos(node, target[0], target[1])
            node.destroy_node()
            node = rclpy.create_node("cover_reef")
        node.get_logger().info(f"Finished ring. {len(rings)} left.")
        rings.popleft()
    node.get_logger().info("Finished spiraling")

def main():
    rclpy.init()
    node = rclpy.create_node("cover_reef")

    node.get_logger().info("Starting checklist...")
    checklist(node)

    node.get_logger().info("Following boundary...")
    #boundary_points = follow_boundary(node)

    # Restart node
    node.destroy_node()
    node = rclpy.create_node("cover_reef")

    node.get_logger().info("Spiraling inside...")
    #spiral_inside(node, boundary_points)
    spiral_inside(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
