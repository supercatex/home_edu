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


def genderate_mask(size):
    zero = np.zeros(size)
    for y in range(size[0]):
        for x in range(size[1]):
            dist = calc_ph(x, y, center_point)
            zero[y, x] = dist
    biggest = np.max(zero)

    mask = 1.0 - zero / biggest
    return mask


def calc_kp(x1, y1, kp):
    error = (x1 - y1)
    return error * kp


def calc_ph(x, y, center_point):
    # This function will only works if you have imported the maths library
    return sqrt((center_point[1] - x) ** 2 + (center_point[0] - y) ** 2)  # center_point format: (y, x)


p1 = 1.0 / 1300.0
p2 = 1.0 / 1300.0
turn_p = -(1.0 / 320.0)
horizan = 595
forward_speed = 0
most_center_point = (0, 0)
most_center_dis = 0
center_point = (240, 320)
center_x = center_point[1]
turn_speed = 0
size = (480, 640)

mask = genderate_mask(size)

cv.namedWindow("frame")
cv.setMouseCallback("frame", c.mouse_callback)

while not rospy.is_shutdown():
    frame, image = c.depth_image, c.rgb_image

    # frame = np.int0(mask * frame)

    zeros = np.nonzero(frame)

    if len(zeros[0]) > 0:
        val = np.min(frame[zeros])
        n = np.where(np.logical_and(frame <= val, frame > 0))

        most_center_point = (0, 0)
        most_center_dis = 0
        for locations in range(len(n[0])):
            x, y = n[1][locations], n[0][locations]
            Dist = calc_ph(x, y, center_point)
            if not Dist > 250:
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

        if not minLoc[0] == 0:
            turn_speed = calc_kp(minLoc[0], center_x, turn_p)

        chassis.move(forward_speed, turn_speed)
        print("Darkness point: %s, Location: %s, Distance: %s, Speed: %s, Turn: %s, To center: %s" % (val, minLoc, val, forward_speed, turn_speed, Dist))

        cv.circle(image, minLoc, 60, (0, 255, 0), 2)

    cv.imshow('origin', frame)
    cv.imshow('frame', image)

    if cv.waitKey(1) == ord('q'):
        break

cv.destroyAllWindows()
