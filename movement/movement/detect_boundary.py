#!/usr/bin/env python3
# Analyze images containing a reef boundary and publish deviation info
import rclpy
import cv2
import numpy as np
import sys

from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from movement_interfaces.msg import Deviation

# Given a contour, return its x,y centroid
def find_centroid(contour):
    # contour[pixel][0][x/y]
    sum_x = sum_y = 0
    layer_size = len(contour)
    i = 0
    while i < layer_size:
        sum_x = sum_x + contour[i][0][0]
        sum_y = sum_y + contour[i][0][1]
        i += 1
    centroid_x = sum_x / layer_size
    centroid_y = sum_y / layer_size
    return centroid_x, centroid_y

# Given an array of contours, return the contour that is most likely to be the boundary
def best_contour(contours):
    # find the largest contours
    largest_contours = []
    i = 0
    while i < len(contours):
        if len(contours[i]) > 100:
            largest_contours.append(contours[i])
        i += 1
    # find the contour with the least x centroid
    best_contour = largest_contours[0]
    i = 0
    while i < len(largest_contours):
        i_centroid = find_centroid(largest_contours[i])[0]
        if i_centroid < find_centroid(best_contour)[0] and i_centroid > 70:
            best_contour = largest_contours[i]
        i += 1
    #print(str(find_centroid(best_contour)[0]))
    return best_contour

# Given a cv2 image, return the x,y deviation of the boundary line's centroid from the center of the image
def find_deviation(img, show_pictures):
    # take the top half of the image
    h, w = img.shape[:2]
    ts = 0
    te = h // 2
    img = img[ts:te, :]
    
    t_lower = 100
    t_upper = 150
    # 100, 150 - works but get distracted by the fish

    edge = cv2.Canny(img, t_lower, t_upper)

    contours, _ = cv2.findContours(edge, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    if show_pictures:
        manual_canvas = np.zeros_like(img)

    # find best contour and calculate centroid
    contour = best_contour(contours)
    centroid_x, centroid_y = find_centroid(contour)

    if show_pictures:
        i = 0
        while i < len(contour):
            manual_canvas[contour[i][0][1]][contour[i][0][0]] = (255, 255, 255)
            i += 1
        manual_canvas[int(centroid_y)][int(centroid_x)] = (0, 0, 255)

    # get center of image
    h, w, _ = img.shape
    center_x = w / 2
    center_y = h / 2
    if show_pictures:
        manual_canvas[int(center_y)][int(center_x)] = (0, 255, 0)

    # find deviation
    deviation_x = centroid_x - center_x
    deviation_y = centroid_y - center_y

    if show_pictures:
        print("Deviation x: " + str(deviation_x) + ", y: " + str(deviation_y))
        cv2.imshow("Original", img)
        cv2.imshow("Edges", edge)
        cv2.imshow("Biggest Line", manual_canvas)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    
    return deviation_x, deviation_y

def main():
    if len(sys.argv) > 1:
        img = cv2.imread(sys.argv[1])
        x_deviation, y_deviation = find_deviation(img, True)
        return

    rclpy.init()
    node = rclpy.create_node("detect_boundary")
    bridge = CvBridge()

    def callback(msg):
        nonlocal bridge
        cv_image = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="passthrough")
        x_deviation, y_deviation = find_deviation(cv_image, False)
        publisher = node.create_publisher(Deviation, "boundary_info", 10)
        msg = Deviation()
        msg.x = x_deviation
        msg.y = y_deviation
        publisher.publish(msg)

    subscriber = node.create_subscription(CompressedImage, "/aqua/camera/back/image_raw/compressed", callback, 10)
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
