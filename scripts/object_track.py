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

c = astra("top_camera")


def genderate_mask(size):
    zero = np.zeros(size)
    for y in range(size[0]):
        for x in range(size[1]):
            dist = calc_ph(x, y, center_point)
            zero[y, x] = dist
    biggest = np.max(zero)

    mask = zero / biggest
    return mask


def calc_kp(x1, y1, kp):
    error = (x1 - y1)
    return error * kp


def calc_ph(x, y, center_point):
    # This function will only works if you have imported the maths library
    return sqrt((center_point[1] - x) ** 2 + (center_point[0] - y) ** 2)  # center_point format: (y, x)


# Kps for turning and forward
p1 = 1.0 / 900.0
turn_p = -(1.0 / 150.0)

# The nearest distance for the robot between the operator
horizan = 595

# Set the forward and turn speed
forward_speed = 0
turn_speed = 0

# Set this two variable for finding the most center point
most_center_point = (0, 0)
most_center_dis = 0
center_point = (240, 320)

# The center point
center_x = center_point[1]

# size of the image
size = (480, 640)

# Color range of human skin
min = np.array([0, 48, 80], np.uint8)
max = np.array([18, 255, 255], np.uint8)

Dist = 0.0

mask = genderate_mask(size)

cv.namedWindow("frame")
cv.setMouseCallback("frame", c.mouse_callback)

while not rospy.is_shutdown():
    depth, image = c.depth_image, c.rgb_image

    frame = depth[(480 / 6):(480 / 6 * 5), (640 / 4):(640 / 4 * 3)].copy()

    blurred_frame = cv.GaussianBlur(image, (5, 5), 0)

    # Convert RGB image to BGR image

    hsv = cv.cvtColor(image, cv.COLOR_RGB2HSV)

    # Set mask of skin
    mask_skin = cv.inRange(hsv, min, max)

    # frame = np.int0(mask * frame)

    nonzeros = np.nonzero(frame)

    kernal = cv.getStructuringElement(cv.MORPH_ELLIPSE, (11, 11))
    mask_skin = cv.dilate(mask_skin, kernal, iterations=2)
    mask_skin = cv.erode(mask_skin, kernal, iterations=2)

    if len(nonzeros[0]) > 0:
        val = np.min(frame[nonzeros])
        n = np.where(np.logical_and(frame <= val+30, frame > 0))
        # print(n)

        most_center_point = (0, 0)
        most_center_dis = 0
        for locations in range(len(n[0])):
            x, y = n[1][locations], n[0][locations]
            Dist = calc_ph(x, y, center_point)
            if Dist < 250:
                if most_center_point == (0, 0) and most_center_dis == 0:
                    most_center_point = x, y
                    most_center_dis = Dist
                if Dist < most_center_dis:
                    most_center_point = x, y
                    most_center_dis = Dist
            else:
                pass

        minLoc = (most_center_point[0] + (640 / 4), most_center_point[1])

        error = (val - horizan)

        notzero = len(nonzeros[0])

        if val < 1300 or (len(frame) - notzero > notzero):
            forward_speed = calc_kp(val, horizan, p1)

            if not minLoc[0] == 0:
                turn_speed = calc_kp(minLoc[0], center_x, turn_p)
        else:
            forward_speed = 0
            turn_speed = 0

        chassis.move(forward_speed, turn_speed)
        print("Darkness point: %s, Location: %s, Distance: %s, Speed: %s, Turn: %s, To center: %s, error: %s" % (val, minLoc, val, forward_speed, turn_speed, Dist, error))

        cv.circle(image, minLoc, 60, (0, 255, 0), 2)
        cv.circle(image, minLoc, 6, (0, 255, 255), 2)

    cv.imshow("frame", image)
    cv.imshow("depth", frame)
    cv.imshow('mask', mask)

    if cv.waitKey(1) in [ord('q'), 27]:
        break

cv.destroyAllWindows()
