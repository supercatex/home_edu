#!/usr/bin/env python

import rospy
import cv2 as cv
import dlib

rospy.init_node("home_edu_test", anonymous=True)

_face_detector = dlib.get_frontal_face_detector()

image = cv.imread("./temp.jpg", cv.IMREAD_COLOR)

while not rospy.is_shutdown():
    dets, _, _ = _face_detector.run(image, False)
    
    for i, d in enumerate(dets):
        x1 = d.left()
        y1 = d.top()
        x2 = d.right()
        y2 = d.bottom()
        cv.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
    
    cv.imshow('frame', image)
    cv.waitKey(0)
