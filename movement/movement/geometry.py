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

# Given a deque of points representing a polygon, find the point that is the furthest from the polygon's centroid
def furthest_from_centroid(polygon):
    centroid = find_centroid(polygon)
    furthest_point = polygon[0]
    for point in polygon:
        if find_distance(point, centroid) > find_distance(furthest_point, centroid):
            furthest_point = point
    return furthest_point

# Given a deque of vertices and a distance, return a new deque with a decreased radius
# Example usage: shrink_polygon(boundary_points, 5)
"""
There are two ways to go about solving this problem. The first, as is implemented below, is to calculate the scale factor based on the point that is furthest from the centroid.
The second is to calculate a different scale factor for each point, based on their own distance from the centroid. The prior creates a much smoother polygon but tends to create
more overlap at some places. The latter ensures each point is the same distance from their counterparts throughout the polygon.
"""
def shrink_polygon(vertices, distance):
    vertices_copy = deque()
    centroid = find_centroid(vertices)
    # find the point furthest from the centroid
    furthest_point = furthest_from_centroid(vertices)
    # use that to determine the scale factor
    radius = find_distance(furthest_point, centroid)
    scale = 1 - (distance / radius)
    # shrink every point
    for point in vertices:
        new_point = [point[0], point[1]]
        # Shift by centroid
        new_point[0] -= centroid[0]
        new_point[1] -= centroid[1]
        # Scale
        new_point[0] *= scale
        new_point[1] *= scale
        # Shift back
        new_point[0] += centroid[0]
        new_point[1] += centroid[1]
        # Append point
        vertices_copy.append(new_point)
    return vertices_copy

# Given an unmodified deque of rings, modify their start and end points to promote smooth transitions
def add_transitions(rings):
    # TODO: Even smoother transitions
    total_points_moved = 0
    for points in rings:
        for i in range(total_points_moved):
            points.append(points.popleft())
        total_points_moved += 1

# Given a polygon, fov, and height above the terrain (for the camera width) return a deque of smaller polygons that ensures complete coverage of the original polygon
def get_rings(polygon, fov, height):
    rings = deque()
    rings.append(polygon)
    distance = height * (math.sin(fov) / 2)
    while True:
        inner_ring = shrink_polygon(rings[-1], distance)
        if find_distance(furthest_from_centroid(rings[-1]), find_centroid(rings[-1])) > distance / 2:
            rings.append(inner_ring)
        else:
            break
    add_transitions(rings)
    return rings

# Given x, y, z, and w values from a quaternion, convert to euler angles
def euler_from_quaternion(x, y, z, w):
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

# Given a radian value r, convert to degrees
def degrees_from_radians(r):
    return r * (180/math.pi)

# Given a current angle c and an addend a (in degrees), return a new angle in degrees
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
