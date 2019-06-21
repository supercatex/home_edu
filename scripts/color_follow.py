#!/usr/bin/env python
from core import Astra as astra
import numpy as np
import cv2
import rospy
import math
import time


class color_follow(object):
	def __init__(self):
		self.sensetivity = 50
		self.lower = []
		self.upper = []
	def first_center_point(depth_image, color_image):
		h, w, c = color_image.shape
		x = w / 2
		y = h / 2 - 20
		z = depth_image[x, y]
		rgb = color_image[x, y]
		center = (x, y, z, rgb[0], rgb[1], rgb[2])
		return x, y, rgb
		
	def first_center_color(rgb):
		self.sensetivity = s #sensetivity
		r = rgb[0]
		g = rgb[1]
		b = rgb[2]
		self.lower = [b - s, g - s, r - s]
		self.upper = [b + s, g + s, r + s]

	def find_color(self, center, depth_image, color_image):
		min_size = 100
		x, y, z, r, g, b = center
		lower = np.array(self.lower, dtype="uint8")
		upper = np.array(self.upper, dtype="uint8")
		mask = cv2.inRange(rgb_image, lower, upper)
		ret, thresh = cv2.threshold(mask, 40, 255, 0)
		_, contours, _ = cv.findContours(
			mask,
			cv.RETR_EXTERNAL,
			cv.CHAIN_APPROX_SIMPLE
		)
		results = []
		for cnt in contours:
			area = cv.contourArea(cnt)
			if area > min_size:
				results.append(cnt)
		results.sort(key=cv.contourArea, reverse=True)
		return results
		
	def find_center(self, cnt):
		if cnt[0] is None
		m = cv.moments(cnt)
		if m["m00"] != 0:
			x = int(np.round(m["m10"] / m["m00"]))
			y = int(np.round(m["m01"] / m["m00"]))
			return x, y
		return 0, 0
		
	def generate_point(self, depth_image, color_image, point_old, point_new):
			flag = False
			max_range = 100
			e = 0
			x1, y1, z1, r1, g1, b1 = point_old
			x2, y2, z2, r2, g2, b2 = point_new
			
			point_old_val = math.sqrt(x1 * x1 + y1 * y1 + z1 * z1 + r1* r1 + g1 * g1 + b1 * b1)
			point_new_val = math.sqrt(x2 * x2 + y2 * y2 + z2 * z2 + r2* r2 + g2 * g2 + b2 * b2)
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
	print("start")
	time.sleep(1)
	frame, image = c.depth_image, c.rgb_image
	color_d = color_follow()
	x, y, rgb = color_d.first_center_point(frame, image)
	
	color_d.first_center_color(rgb)
	
	while not rospy.is_shutdown():
		frame, image = c.depth_image, c.rgb_image
		if len(cnts) > 0:
		if cv2.waitKey(1) in [ord('q'), 27]:
			break

cv2.destroyAllWindows()