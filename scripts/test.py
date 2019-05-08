#!/usr/bin/env python

import cv2 as cv

from core import Astra as astra
import rospy


rospy.init_node("pcms_test", anonymous=True)
rate = rospy.Rate(20)

c = astra("cam2")
cv.namedWindow("image")
cv.setMouseCallback("image", c.mouse_callback)
image_path = "/home/mustar/pcms/src/home_edu/scripts/temp.jpg"

while rospy.is_shutdown:
    if c.rgb_image is None:
        continue

    frame = c.rgb_image
    cv.imshow('frame', frame)
    rate.sleep()

    if cv.waitKey(1) == ord('q'):
        image = frame.copy()
        image = cv.resize(image, (1280, 960))
        cv.imwrite("/home/mustar/pcms/src/home_edu/scripts/temp.jpg", image)
        break
    # image = cv.imread(image_path, cv.IMREAD_COLOR)
    # image = cv.resize(image, (1280, 960))
    # cv.imwrite(image_path, image)
