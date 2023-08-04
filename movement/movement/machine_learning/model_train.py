#!/usr/bin/env python3
from keras.datasets import mnist
from keras.utils import to_categorical
from keras.models import Sequential
from keras.layers import Dense, Conv2D, Flatten
from keras.models import load_model
import numpy as np
import cv2
import os

def count_files(path):
    print(f"Counting the number of files in the path '{path}'...")
    count = 0
    fd = os.listdir(path)
    for i in fd:
        ip = os.path.join(path, i)
        if os.path.isfile(ip):
            count += 1
    return count 

# Return training and test data
def load_data():
    x_train = [] # shape (x, 20, 20, 3)
    y_train = [] # shape (x,)

    x_test = []
    y_test = []

    # sand
    sand_files_n = count_files("real_world/train/sand")
    for i in range(sand_files_n):
        # train
        img = cv2.imread(f"real_world/train/sand/{str(i+1)}.png")
        x_train.append(img)
        y_train.append(0)
    sand_files_n = count_files("real_world/test/sand")
    for i in range(sand_files_n):
        # test
        img = cv2.imread(f"real_world/test/sand/{str(i+1)}.png")
        x_test.append(img)
        y_test.append(0)
    
    # rock
    rock_files_n = count_files("real_world/train/rock")
    for i in range(rock_files_n):
        # train
        img = cv2.imread(f"real_world/train/rock/{str(i+1)}.png")
        x_train.append(img)
        y_train.append(1)
    rock_files_n = count_files("real_world/test/rock")
    for i in range(rock_files_n):
        # test
        img = cv2.imread(f"real_world/test/rock/{str(i+1)}.png")
        x_test.append(img)
        y_test.append(1)

    x_train = np.array(x_train)
    y_train = np.array(y_train)

    x_test = np.array(x_test)
    y_test = np.array(y_test)
    
    return (x_train, y_train), (x_test, y_test)

def main():
    (x_train, y_train), (x_test, y_test) = load_data()
    x_train = (x_train / 255)
    x_test = (x_test / 255)
    y_train = to_categorical(y_train)
    y_test = to_categorical(y_test)

    model = Sequential()

    model.add(Conv2D(64, kernel_size=3, activation="relu", input_shape=(20,20,3)))
    model.add(Conv2D(32, kernel_size=3, activation="relu"))
    model.add(Flatten())
    model.add(Dense(2, activation="softmax"))

    model.compile(optimizer="adam", loss="categorical_crossentropy", metrics=["accuracy"])

    model.fit(x_train, y_train, validation_data = (x_test, y_test), epochs = 30)

    model.save("real_cnn.h5")

if __name__ == "__main__":
    main()
