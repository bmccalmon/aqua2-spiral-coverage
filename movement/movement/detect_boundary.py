#!/usr/bin/env python3
# Analyze images containing a reef boundary and publish deviation info
import rclpy
import cv2
import numpy as np

from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from movement_interfaces.msg import Deviation

# Given a cv2 image, return the x,y deviation of the boundary line's centroid from the center of the image
def find_deviation(img):
    t_lower = 150
    t_upper = 200

    edge = cv2.Canny(img, t_lower, t_upper)

    contours, _ = cv2.findContours(edge, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    # manually draw contour and centroid
    contour_size = len(contours)
    j = 0 # index of preferred layer
    layer_size = len(contours[j])
    # we want the line with the most pixels as that is more likely to be the boundary of the reef
    i = 0
    while i < contour_size:
        if len(contours[i]) > layer_size:
            j = i
            layer_size = len(contours[j])
        i = i + 1
    sum_x = 0
    sum_y = 0
    i = 0
    while i < layer_size:
        #manual_canvas[y][x]
        sum_x = sum_x + contours[j][i][0][0]
        sum_y = sum_y + contours[j][i][0][1]
        i = i + 1
    centroid_x = sum_x / layer_size
    centroid_y = sum_y / layer_size

    # get center of image
    h, w, _ = img.shape
    center_x = w / 2
    center_y = h / 2

    # find deviation
    deviation_x = centroid_x - center_x
    deviation_y = centroid_y - center_y
    
    return deviation_x, deviation_y

def main():
    rclpy.init()
    node = rclpy.create_node("detect_boundary")
    bridge = CvBridge()

    def callback(msg):
        nonlocal bridge
        cv_image = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="passthrough")
        x_deviation, y_deviation = find_deviation(cv_image)
        node.get_logger().info("Deviation x: " + str(x_deviation) + ", y: " + str(y_deviation))
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
