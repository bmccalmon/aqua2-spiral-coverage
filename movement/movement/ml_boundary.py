#!/usr/bin/env python3
# Analyze images containing a reef boundary and publish deviation info
import rclpy
import cv2
import numpy as np
import sys
from keras.models import load_model

import geometry
import et_boundary

from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from movement_interfaces.msg import Deviation

# Given an image, return a numpy array of smaller images with dimensions chunk_size
def load_chunks(image, chunk_size):
    height, width, _ = image.shape
    num_rows = height // chunk_size
    num_cols = width // chunk_size

    chunks = []

    for row in range(num_rows):
        for col in range(num_cols):
            y_start = row * chunk_size
            y_end = y_start + chunk_size
            x_start = col * chunk_size
            x_end = x_start + chunk_size

            sub_image = image[y_start:y_end, x_start:x_end]
            if not (sub_image.shape[0] == chunk_size and sub_image.shape[1] == chunk_size):
                continue

            chunks.append(sub_image)

    chunks = np.array(chunks)

    return chunks

# Given a placeholder image and binary info for each chunk, return a B/W image
def fill_chunks(chunk_info, img, chunk_size):
    b_img = np.zeros_like(img)
    h, w = b_img.shape[:2]
    
    num_rows = h // chunk_size
    num_cols = w // chunk_size

    ci = 0
    for row in range(num_rows):
        for col in range(num_cols):
            y_start = row * chunk_size
            y_end = y_start + chunk_size
            x_start = col * chunk_size
            x_end = x_start + chunk_size

            # fill in with color; sand=white, rock=black
            color_to_fill = 0
            if np.argmax(chunk_info[ci]) == 0:
                color_to_fill = 255
            b_img[y_start:y_end, x_start:x_end] = color_to_fill
            ci += 1

    return b_img

# Given a cv2 image, return the x,y deviation of the boundary line's centroid from the center of the image
def find_deviation(img, show_pictures, model):
    # slice the image
    chunk_size = 20
    chunks = load_chunks(img, chunk_size)
    
    # prepare for the model
    chunks = chunks / 255

    # pass through model
    predictions = model.predict(chunks)

    # construct a new image
    bw_img = fill_chunks(predictions, img, 20)

    # testing contour detection
    return et_boundary.find_deviation(bw_img, show_pictures)

    # take top half of the image
    h, w = bw_img.shape[:2]
    ts = 0
    te = h // 2
    bw_img = bw_img[ts:te, :]
    bw_img = cv2.cvtColor(bw_img, cv2.COLOR_BGR2GRAY)
    h, w = bw_img.shape[:2]
    
    # find the boundary points
    boundary_points = []
    # start from right, find where they meet
    for hi in range(h):
        wi = w - 1
        if bw_img[hi][wi] == 255:
            continue
        while wi >= 0:
            if bw_img[hi][wi] == 255:
                # sometimes there are chunk errors, check for that
                tw = wi - chunk_size - 1
                if bw_img[hi][tw] == 255:
                    boundary_points.append((wi, hi))
            wi -= 1
    # start from top, find where they meet
    # find left-most x value
    left_most = w
    for x,_ in boundary_points:
        if x < left_most:
            left_most = x
    wi = left_most
    while wi < w:
        if bw_img[0][wi] == 0:
            wi += 1
            continue
        for hi in range(h):
            if bw_img[hi][wi] == 0:
                #th = hi + chunk_size + 1
                #if bw_img[th][wi] == 0:
                boundary_points.append((wi, hi))
                break
        wi += 1

    # find the centroid
    centroid = geometry.find_centroid(boundary_points)
   
    # find the deviation from the center
    center_x = w / 2
    center_y = h / 2

    deviation_x = centroid[0] - center_x
    deviation_y = centroid[1] - center_y

    if show_pictures:
        bw_img = cv2.cvtColor(bw_img, cv2.COLOR_GRAY2RGB)
        for x,y in boundary_points:
            bw_img[y][x] = (0, 0, 255)
        cv2.imshow("bw", bw_img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    return deviation_x, deviation_y

def main():
    # load the ML model
    model = load_model("machine_learning/reef_cnn.h5")

    if len(sys.argv) > 1:
        img = cv2.imread(sys.argv[1])
        x_deviation, y_deviation = find_deviation(img, True, model)
        return

    rclpy.init()
    node = rclpy.create_node("detect_boundary")
    bridge = CvBridge()
    publisher = node.create_publisher(Deviation, "boundary_info", 1)

    def callback(msg):
        nonlocal bridge, publisher
        cv_image = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="passthrough")
        x_deviation, y_deviation = find_deviation(cv_image, False, model)
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
