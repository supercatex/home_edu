#!/usr/bin/env python
from core import Astra as astra
from core import Manipulator as Manipulator
import numpy as np
import cv2
import rospy
import math
import time

def detect(depth_image, color_image):
	
	print (color_image.shape)

if __name__ == "__main__":
	rospy.init_node("home_edu_manipulator_track", anonymous=True)
	rate = rospy.Rate(20)

	c = astra("top_camera")

	while not rospy.is_shutdown():
 
		frame, image = c.depth_image, c.rgb_image

		detect(frame, image)

		if cv2.waitKey(1) in [ord('q'), 27]:
			break

cv2.destroyAllWindows()