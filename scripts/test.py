#!/usr/bin/env python

import rospy
import cv2 as cv
import dlib
import numpy as np

rospy.init_node("home_edu_test", anonymous=True)

alpha = 1.0
beta = 50

_face_detector = dlib.get_frontal_face_detector()
image = cv.imread("/home/mustar/pcms/src/home_edu/scripts/temp.jpg", cv.IMREAD_COLOR)
image = cv.resize(image, (640, 480))
for y in range(image.shape[0]):
    for x in range(image.shape[1]):
        for c in range(image.shape[2]):
            image[y, x, c] = np.clip(alpha*image[y, x, c] + beta, 0, 255)


dets, _, _ = _face_detector.run(image, False)

for i, d in enumerate(dets):
    x1 = d.left()
    y1 = d.top()
    x2 = d.right()
    y2 = d.bottom()
    cv.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)

cv.imshow('frame', image)
cv.waitKey(0)
cv.destroyAllWindows()
