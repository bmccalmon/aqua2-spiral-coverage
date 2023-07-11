#!/usr/bin/env python3
# Functions to visualize points
import matplotlib.pyplot as plt
from collections import deque
import geometry

import pickle
import math

# Given a list consisting of a list of of points, create and show a graph
def plot_points(polygon_list):
    i = 0
    while i < len(polygon_list):
        x_coords = [point[0] for point in polygon_list[i]]
        y_coords = [point[1] for point in polygon_list[i]]
        plt.plot(x_coords, y_coords)
        i += 1
    plt.xlim(-25, 35)
    plt.ylim(-25, 35)
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title(f"{len(polygon_list)} polygons")
    plt.show()

def main():
    """
    outer = deque([[0,0],[5,5],[10,0],[10,30],[5,25],[0,0]])
    inner = geometry.shrink_polygon(outer, 3)
    plot_points([outer, inner])
    return
    """

    polygon_list = []
    with open("sim_map.pickle", "rb") as file:
        polygon_list.append(pickle.load(file))
    polygon_list.append(geometry.shrink_polygon(polygon_list[0], 1))
    polygon_list.append(geometry.shrink_polygon(polygon_list[1], 1))
    polygon_list.append(geometry.shrink_polygon(polygon_list[2], 1))
    polygon_list.append(geometry.shrink_polygon(polygon_list[3], 1))
    polygon_list.append(geometry.shrink_polygon(polygon_list[4], 1))
    polygon_list.append(geometry.shrink_polygon(polygon_list[5], 1))
    polygon_list.append(geometry.shrink_polygon(polygon_list[6], 1))
    polygon_list.append(geometry.shrink_polygon(polygon_list[7], 1))
    polygon_list.append(geometry.shrink_polygon(polygon_list[8], 1))
    plot_points(polygon_list)

if __name__ == "__main__":
    main()
