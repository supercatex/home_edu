#!/usr/bin/env python

import rospy
import time 
from core import Astra as astra
import cv2 as cv
from os.path import abspath

rospy.init_node("pcms_test", anonymous=True)

rate = rospy.Rate(20)
face_cascade = cv.CascadeClassifier("/home/mustar/pcms/src/home_edu/scripts/libs/haarcascade_upperbody.xml")
c = astra("top_camera")

while not rospy.is_shutdown():
    frame = c.rgb_image
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    mouths = face_cascade.detectMultiScale(gray, minSize=(200, 200))
    for (x, y, w, h) in mouths:
        cv.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        
    cv.imshow('frame', frame)
    if cv.waitKey(1) in [ord('q'), 27]:
        break
        
cv.destroyAllWindows()
