#!/usr/bin/env python

import cv2 as cv

from core import Astra as astra
import rospy

rospy.init_node("pcms_test", anonymous=True)

rate = rospy.Rate(20)

c = astra("top_camera")

frame = c.rgb_image


