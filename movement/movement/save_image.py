#!/usr/bin/env python3
# Saves an image from the back camera to the  project's debug folder
import rclpy
import cv2 as cv
import datetime
import sys
import time

from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage

def main():
    rclpy.init()
    node = rclpy.create_node("save_image")

    cooldown = 0
    once = True

    if len(sys.argv) > 1:
        cooldown = int(sys.argv[1])
        once = False
        node.get_logger().info(f"Continuous mode activated. Cooldown: {cooldown} seconds.")

    def callback(msg):
        nonlocal cooldown
        bridge = CvBridge()
        cv_image = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="passthrough")
        cv.imwrite("debug/images/result/" + str(datetime.datetime.now()) + ".jpg", cv_image)
        time.sleep(cooldown)

    subscriber = node.create_subscription(CompressedImage, "/aqua/camera/back/image_raw/compressed", callback, 1)

    while True:
        rclpy.spin_once(node)
        if once:
            break

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
