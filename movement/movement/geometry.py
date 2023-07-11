#!/usr/bin/env python3
# Useful functions related to geometry that can be used
import math
from collections import deque

# Given a deque of vertices and a scale, return a new deque with a decreased radius
# Example usage: shrink_polygon(boundary_points, 0.9)
def shrink_polygon(vertices, scale):
    # Deep copy deque vertices
    vertices_copy = deque()
    i = 0
    while i < len(vertices):
        vertices_copy.append([vertices[i][0], vertices[i][1]])
        i += 1 
    # Calculate the centroid of the polygon
    sum_x = sum_y = 0
    i = 0
    while i < len(vertices_copy):
        sum_x += vertices_copy[i][0]
        sum_y += vertices_copy[i][1]
        i += 1
    centroid_x = sum_x / len(vertices_copy)
    centroid_y = sum_y / len(vertices_copy)
    centroid = [centroid_x, centroid_y]
    # Shift every point by the centroid, scale, then shift back
    i = 0
    while i < len(vertices_copy):
        # Shift by centroid
        vertices_copy[i][0] -= centroid_x
        vertices_copy[i][1] -= centroid_y
        # Scale
        vertices_copy[i][0] *= scale
        vertices_copy[i][1] *= scale
        # Shift back
        vertices_copy[i][0] += centroid_x
        vertices_copy[i][1] += centroid_y
        i += 1
    return vertices_copy

# Given a polygon, return a deque of smaller polygons that ensures complete coverage of the original polygon
def get_rings(polygon):
    rings = deque()
    # TODO: Based on the boundary, determine how many rings to generate and the space in between each
    rings.append(polygon)
    rings.append(shrink_polygon(polygon, 0.666))
    rings.append(shrink_polygon(polygon, 0.333))
    return rings

# given x, y, z, and w values from a quaternion, convert to euler angles
def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z # in radians

# given a radian value r, convert to degrees
def degrees_from_radians(r):
    return r * (180/math.pi)

# given a current angle c and an addend a (in degrees), return a new angle in degrees
def get_target_angle(c, a):
    sum = c + a
    if sum >= 180:
        sum = sum - 360
    elif sum <= -180:
        sum = sum + 360
    return sum

# Given a current and target position, return the angle needed to face it
def find_angle_to_face(c, t):
    dx = t[0] - c[0]
    dy = t[1] - c[1]
    angle = math.atan2(dy, dx)
    return degrees_from_radians(angle)
