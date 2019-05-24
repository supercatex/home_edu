#!/usr/bin/env python
from core import Kobuki as kobuki
from core import Astra as astra
import cv2 as cv
import rospy
import numpy as np
from math import sqrt

rospy.init_node("home_edu_object_track", anonymous=True)
rate = rospy.Rate(20)

chassis = kobuki()

c = astra("camera")


def calc_kp(object_dis, known_dis, kp):
    error = (object_dis - known_dis)
    return error * kp


def calc_ph(x, y, center_point):
    # This function will only works if you have imported the maths library
    return sqrt((center_point[1] - x) ** 2 + (center_point[0] - y) ** 2) # center_point format: (y, x)


p1 = 1.0 / 2600.0
p2 = 1.0 / 1400.0
negative_p = -(1.0 / 2600.0)
horizan = 595
forward_speed = 0
most_center_point = (0, 0)
most_center_dis = 0
center_point = (240, 320)

cv.namedWindow("frame")
cv.setMouseCallback("frame", c.mouse_callback)

while not rospy.is_shutdown():
    frame = c.depth_image

    zeros = np.nonzero(frame)

    if len(zeros[0]) > 0:
        val = np.min(frame[zeros])
        n = np.where(frame == val)

        for i in n:
            for locations in range(len(i)):
                x, y = n[0][locations], n[1][locations]
                Dist = calc_ph(x, y, center_point)
                if most_center_point == (0, 0) and most_center_dis == 0:
                    most_center_point = x, y
                    most_center_dis = Dist
                if Dist < most_center_dis:
                    most_center_point = x, y
                    most_center_dis = Dist

        minLoc = (most_center_point[0], most_center_point[1])

        error = (val - horizan)

        if not val > 1370:
            if error > 0:
                forward_speed = calc_kp(val, horizan, p1)

            else:
                forward_speed = calc_kp(val, horizan, p2)

        chassis.move(forward_speed, 0)
        print("Darkness point: %s, Location: %s, Distance: %s, Speed: %s" % (val, minLoc, val, forward_speed))

        cv.circle(frame, minLoc, 41, (0, 255, 0), 2)

    cv.imshow('frame', frame)

    if cv.waitKey(1) == ord('q'):
        break

cv.destroyAllWindows()
