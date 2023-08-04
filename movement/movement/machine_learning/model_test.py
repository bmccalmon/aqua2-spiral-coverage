#!/usr/bin/env python3
import model_train
from keras.models import load_model
import numpy as np
import cv2
from sklearn.metrics import accuracy_score, precision_score, recall_score, f1_score

import numpy as np
from sklearn.metrics import accuracy_score, precision_score, recall_score, f1_score
from keras.utils import to_categorical

def main():
    (x_train, y_train), (x_test, y_test) = model_train.load_data()
    x_test = (x_test / 255)
    y_test = to_categorical(y_test, num_classes=2)

    model = load_model("real_cnn.h5")
    score = model.evaluate(x_test, y_test, verbose = 0)
    print(f"Accuracy: {score[1]}")

if __name__ == "__main__":
    main()
