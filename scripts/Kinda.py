#!/usr/bin/env python
import rospy
from core import Astra
from core import Kobuki
import cv2


rospy.init_node("home_edu_Kinda")
rate = rospy.Rate(20)

chassis = Kobuki()
camera = Astra()

while True:
    # chassis.move(0, 0.3)
    rgb = camera.rgb_image
    depth = camera.depth_image

    cv2.imshow("frame", rgb)
    cv2.imshow("depth", depth)

    print(depth[240, 320])

    key = cv2.waitKey(1)
    if key in [ord('q'), 27]:
        break
