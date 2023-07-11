#!/usr/bin/env python3
# Useful functions related to geometry that can be used
import math
from collections import deque

# Given a deque of points, return its centroid
def find_centroid(points):
    sum_x = sum_y = 0
    for point in points:
        sum_x += point[0]
        sum_y += point[1]
    return [sum_x / len(points), sum_y / len(points)]

# Given two points, return the distance between them
def find_distance(a, b):
    return math.sqrt(((a[0] - b[0]) ** 2) + ((a[1] - b[1]) ** 2))

# Given a deque of vertices and a distance, return a new deque with a decreased radius
# Example usage: shrink_polygon(boundary_points, 5)
def shrink_polygon(vertices, distance):
    # Deep copy deque vertices
    vertices_copy = deque()
    for point in vertices:
        vertices_copy.append([point[0], point[1]])
    # Calculate the centroid of the polygon
    centroid = find_centroid(vertices_copy)
    # Shift every point by the centroid, scale, then shift back
    for point in vertices_copy:
        # Calculate the approximate radius at the current point and, using that, estimate the scale factor needed to satisfy the wanted distance
        radius = find_distance(centroid, point)
        scale = 1 - (distance / radius)
        # Shift by centroid
        point[0] -= centroid[0]
        point[1] -= centroid[1]
        # Scale
        point[0] *= scale
        point[1] *= scale
        # Shift back
        point[0] += centroid[0]
        point[1] += centroid[1]
    return vertices_copy

# Given a polygon, return a deque of smaller polygons that ensures complete coverage of the original polygon
def get_rings(polygon, fov, height):
    rings = deque()
    rings.append(polygon)
    distance = 3
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
