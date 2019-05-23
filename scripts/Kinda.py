#!/usr/bin/env python
import rospy
from core import Astra
import cv2


rospy.init_node("home_edu_Kinda")

camera = Astra()

while True:
    frame = camera.rgb_image
    depth = camera.depth_image

    print(depth[240, 320])

    cv2.imshow("frame", frame)
    key = cv2.waitKey(1)
    if key == ord('q'):
        break
