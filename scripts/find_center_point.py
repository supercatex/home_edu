#!/usr/bin/env python
from core import Astra as astra
import numpy as np
import cv2
import rospy
import math
import time

def find_center(depth_image, color_image):
	
	h, w, c = color_image.shape
	
	x = w / 2
	
	y = h / 2
	
	z = depth_image[x, y]
	
	rgb = color_image[x, y]
	
	center = (x, y, z, rgb[0], rgb[1], rgb[2])
	
	point_old = math.sqrt(x * x + y * y + z * z + rgb[0] * rgb[0] + rgb[1] * rgb[1] + rgb[2] * rgb[2])
	
	return point_old
	
def generate_point(depth_image, color_image, point_old, point_new):
		
		flag = False
		
		max_range = 100
		
		e = 0
		
		if abs(point_old - point_new) > 2000:
			
			while not flag and e < max_range:
				depth = depth_image[max(y - e, 0):min(y + e, 480), max(x - e, 0):min(x + e, 640)].copy()
				indices = np.nonzero(depth)
				if abs(point_old - point_new) < 2000:
					real_z = np.min(depth[indices])
					flag = True
				else:
					e = e + 1
		else:
			
			return point_new
		
		

if __name__ == "__main__":
	rospy.init_node("home_edu_manipulator_track", anonymous=True)
	rate = rospy.Rate(20)

	c = astra("top_camera")

	point_old = 100
	
	while not rospy.is_shutdown():
 
		frame, image = c.depth_image, c.rgb_image

		point_new = find_center(frame, image)
	
		final_point = generate_point(frame, image, point_old, point_new)
		
		point_old = point_new
		
		if cv2.waitKey(1) in [ord('q'), 27]:
			break

cv2.destroyAllWindows()