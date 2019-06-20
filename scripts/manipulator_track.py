#!/usr/bin/env python
from core import Astra as astra
from core import Manipulator as Manipulator
import numpy as np
import cv2
import rospy
import math

class object_detection(object):
	def __init__(self, color):
		
		self.max_range = 25
		
		self.mani_length = 33.5
		
		self.cameraH = 60 * math.pi / 180
		
		self.cameraV = 49.5 * math.pi / 180
		
		if color == "red":
			self.lower = [0, 0, 70]
			self.upper = [98, 53, 255]
			
		elif color == "blue":
			self.lower = [0, 0, 70]
			self.upper = [98, 53, 255]
		else:
			self.lower = [0, 0, 70]
			self.upper = [98, 53, 255]

	def color_detect(self, rgb_image, depth_image):
		lower = np.array(self.lower, dtype="uint8")
		
		upper = np.array(self.upper, dtype="uint8")
		
		print(lower, upper, rgb_image.shape)
		
		mask = cv2.inRange(rgb_image, lower, upper)
		
		ret,thresh = cv2.threshold(mask, 40, 255, 0)
		im2,contours,hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
		
		if len(contours) != 0:
			c = max(contours, key = cv2.contourArea)
			
			x,y,w,h = cv2.boundingRect(c)
			
			cv2.rectangle(rgb_image,(x,y),(x+w,y+h),(0,255,0),3)

			flag = False
			
			e = 0
			
			y = (y + h/2)
			
			x = (x + w/2)
			
			h2, w2 = depth_image.shape
			
			real_z = 0
			
			while not flag and e < self.max_range:
				depth = depth_image[max(y - e, 0):min(y + e, h2), max(x - e, 0):min(x + e, w2)].copy()
				indices = np.nonzero(depth)
				if len(indices[0]) > 0:
					real_z = np.min(depth[indices])
					flag = True
				else:
					e = e + 1

			z = real_z
			
			mid = [x, y, z]
		
		print('mid:', mid)
		return rgb_image, mid

	def calculation(self, mid):
		l = self.mani_length
		x0 = mid[0]
		y0 = mid[1]
		z0 = mid[2]/10
		
		# x1 = z0
		# y1 = y0
		# z1 = x0

		wR = 2 * z0 * math.tan(self.cameraH / 2)
		hR = 2 * z0 * math.tan(self.cameraV/2)
		print("Real W, h", wR, hR)
		xR = (wR * x0)/640
		yR = (hR * y0)/480
		zR = z0
		
		print("Real x, y, z", xR, yR, zR)
		return xR, yR, zR
		
	def 
	def run(self, rgb_image, depth_image):
		frame, mid = self.color_detect(rgb_image, depth_image)
		
		x, y, z = self.calculation(mid)
		
		return frame, x, y, z
		
		
		

if __name__ == "__main__":
	rospy.init_node("home_edu_manipulator_track", anonymous=True)
	rate = rospy.Rate(20)

	c = astra("top_camera")
	
	m = Manipulator()

	obj = object_detection("red")

	while not rospy.is_shutdown():

		frame, image = c.depth_image, c.rgb_image
		
		image, x, y, z = obj.run(c.rgb_image, c.depth_image)

		cv2.imshow("image", image)
		
		print(x, y, z)

		if cv2.waitKey(1) in [ord('q'), 27]:
			break

cv2.destroyAllWindows()