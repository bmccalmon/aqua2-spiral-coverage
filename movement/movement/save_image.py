#!/usr/bin/env python3
# Saves an image from the back camera to the  project's debug folder
import rclpy
import cv2 as cv
import datetime
import sys

from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage

def main():
    rclpy.init()
    node = rclpy.create_node("save_image")

    image_count = 0
    number_of_images = 1

    if len(sys.argv) > 1:
        number_of_images = int(sys.argv[1])

    def callback(msg):
        nonlocal image_count
        image_count += 1
        bridge = CvBridge()
        cv_image = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="passthrough")
        cv.imwrite("debug/images/" + str(datetime.datetime.now()) + ".jpg", cv_image)

    subscriber = node.create_subscription(CompressedImage, "/aqua/camera/back/image_raw/compressed", callback, 10)

    while image_count < number_of_images:
        rclpy.spin_once(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
