#!/usr/bin/env python3
# Functions to visualize points
import matplotlib.pyplot as plt
from collections import deque
import geometry

import pickle
import math

# Given a list consisting of a list of of points, create and show a graph
def plot_points(polygon_list):
    #i = 0
    #while i < len(polygon_list):
    #    x_coords = [point[0] for point in polygon_list[i]]
    #    y_coords = [point[1] for point in polygon_list[i]]
    #    plt.plot(x_coords, y_coords, color = 'black')
    #    i += 1
    merged = deque()
    for d in polygon_list:
        merged += d
    x_coords = [point[0] for point in merged]
    y_coords = [point[1] for point in merged]
    plt.plot(x_coords, y_coords, color = 'black')
    plt.xlim(5, 35)
    plt.ylim(10, 40)
    #plt.xlim(-35, 35)
    #plt.ylim(-35, 35)
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title(f"{len(polygon_list)} polygons")
    plt.show()

def main():
    with open("sim_map.pickle", "rb") as file:
        boundary = pickle.load(file)
    #boundary = deque([[0,0],[5,-5],[10,0],[10,30],[5,25],[0,0]])
    #boundary.append(boundary[0])
    #boundary = deque([[2,-2],[-2,-2],[-2,2],[2,2],[2,-1.5]])
    rings = geometry.get_rings(boundary, 64, 9)
    #rings[1].append(rings[1][0])
    #rings[2].append(rings[2][0])
    #rings[3].append(rings[3][0])
    plot_points(rings)

if __name__ == "__main__":
    main()
