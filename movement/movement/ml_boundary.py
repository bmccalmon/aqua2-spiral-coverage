#!/usr/bin/env python3
# Analyze images containing a reef boundary and publish deviation info
import rclpy
import cv2
import numpy as np
import sys
from keras.models import load_model
import time
import datetime

import geometry

from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from movement_interfaces.msg import Deviation

# Given a contour, return its x,y centroid
def find_contour_centroid(contour):
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

# Given a grayscale binary image, return the x,y deviation of the boundary line's centroid from the center of the image
def find_deviation(img, show_pictures = False):
    # take the top half of the image
    if not show_pictures:
        h, w = img.shape[:2]
        ts = 0
        te = h // 2
        img = img[ts:te, :]
    
    t_lower = 100
    t_upper = 150

    edge = cv2.Canny(img, t_lower, t_upper)

    contours, _ = cv2.findContours(edge, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    if show_pictures:
        manual_canvas = np.zeros_like(img)

    # find the largest contour
    if len(contours) == 0:
        return 0.0, 0.0
    largest_contour = contours[0]
    for contour in contours:
        if len(contour) > len(largest_contour):
            largest_contour = contour
    contour = largest_contour

    # calculate the line's centroid
    centroid_x, centroid_y = find_contour_centroid(contour)

    if show_pictures:
        i = 0
        while i < len(contour):
            #manual_canvas[contour[i][0][1]][contour[i][0][0]] = (0, 0, 255) # (255, 255, 255)
            cv2.line(manual_canvas, contour[i][0], contour[i][0], (0, 0, 255), 3)
            i += 1
        #manual_canvas[int(centroid_y)][int(centroid_x)] = (0, 0, 255)

    # get center of image
    h, w, _ = img.shape
    center_x = w / 2
    center_y = h / 2
    #if show_pictures:
        #manual_canvas[int(center_y)][int(center_x)] = (0, 255, 0)

    # find deviation
    deviation_x = centroid_x - center_x
    deviation_y = centroid_y - center_y

    if show_pictures:
        print("Deviation x: " + str(deviation_x) + ", y: " + str(deviation_y))
        cv2.imshow("Original", img)
        cv2.imshow("Edges", edge)
        cv2.imshow("Biggest Line", manual_canvas)
        cv2.imwrite(f"debug/images/examples/{datetime.datetime.now()}.jpg", manual_canvas)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    return deviation_x, deviation_y

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
def fill_chunks(chunk_info, img, chunk_size, show_pictures = False):
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
            if show_pictures:
                color_to_fill = (0, 128, 0)
            if np.argmax(chunk_info[ci]) == 0:
                color_to_fill = 255
                if show_pictures:
                    color_to_fill = (128, 0, 0) # (205, 250, 255)
            b_img[y_start:y_end, x_start:x_end] = color_to_fill
            ci += 1

    return b_img

# Given a cv2 image, segment the image into sand and reef
def segment_image(img, model, show_pictures = False):
    # slice the image
    chunk_size = 20
    chunks = load_chunks(img, chunk_size)
    
    # prepare for the model
    chunks = chunks / 255

    # pass through model
    predictions = model.predict(chunks)

    # construct a new image
    bw_img = fill_chunks(predictions, img, 20, show_pictures)

    if show_pictures:
        cv2.addWeighted(img, 1, bw_img, 0.5, 0.7, bw_img)
        cv2.imshow("Model results", bw_img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    # testing contour detection
    return bw_img

def main():
    # load the ML model
    model = load_model("machine_learning/real_cnn.h5")

    if len(sys.argv) > 1:
        img = cv2.imread(sys.argv[1])
        img = segment_image(img, model, True)
        x_deviation, y_deviation = find_deviation(img, False)
        # save img
        if len(sys.argv) > 2 and sys.argv[2] == "-s":
            cv2.imwrite(f"debug/images/examples/{datetime.datetime.now()}.jpg", img)
        return

    rclpy.init()
    node = rclpy.create_node("detect_boundary")
    bridge = CvBridge()
    publisher = node.create_publisher(Deviation, "boundary_info", 1)

    def callback(msg):
        nonlocal bridge, publisher
        cv_image = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="passthrough")
        # segment image into sand and reef
        cv_image = segment_image(cv_image, model)
        # find the boundary using edge and contour detection
        x_deviation, y_deviation = find_deviation(cv_image)
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
