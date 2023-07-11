#!/usr/bin/env python3
# Functions to visualize points
import matplotlib.pyplot as plt
from collections import deque
import geometry

import pickle

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
    polygon_list = []
    with open("sim_map.pickle", "rb") as file:
        polygon_list.append(pickle.load(file))
    polygon_list.append(geometry.shrink_polygon(polygon_list[0], 0.666))
    polygon_list.append(geometry.shrink_polygon(polygon_list[0], 0.333))
    plot_points(polygon_list)

if __name__ == "__main__":
    main()
