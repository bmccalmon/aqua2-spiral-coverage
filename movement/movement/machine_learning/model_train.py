#!/usr/bin/env python3
from keras.datasets import mnist
from keras.utils import to_categorical
from keras.models import Sequential
from keras.layers import Dense, Conv2D, Flatten
from keras.models import load_model
import numpy as np
import cv2

# Return training data
def load_data():
    pass

def main():
    (x_train, y_train), (x_test, y_test) = load_data()
    x_train = (x_train / 255)
    x_test = (x_test / 255)
    y_train = to_categorical(y_train)
    y_test = to_categorical(y_test)

    model = Sequential()

    model.add(Conv2D(64, kernel_size=3, activation="relu", input_shape=(28,28,3)))
    model.add(Conv2D(32, kernel_size=3, activation="relu"))
    model.add(Flatten())
    model.add(Dense(10, activation="softmax"))

    model.compile(optimizer="adam", loss="categorical_crossentropy", metrics=["accuracy"])

    model.fit(x_train, y_train, validation_data = (x_test, y_test), epochs = 3)

    model.save("reef_cnn.h5")

if __name__ == "__main__":
    main()
