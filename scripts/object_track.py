#!/usr/bin/env python
from core import Astra as astra
import cv2 as cv
import rospy

rospy.init_node("home_edu_object_track", anonymous=True)
rate = rospy.Rate(20)

c = astra("camera")



cv.namedWindow("frame")
cv.setMouseCallback("frame", c.mouse_callback)

while not rospy.is_shutdown():
    frame = c.depth_image

    cv.imshow('frame', frame)

    if cv.waitKey(1) == ord('q'):
        break

cv.destroyAllWindows()
