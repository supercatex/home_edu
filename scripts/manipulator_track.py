#!/usr/bin/env python
from core import Astra as astra
import numpy as np
import cv2
import rospy
import math


rospy.init_node("home_edu_object_track", anonymous=True)
rate = rospy.Rate(20)

print("ok")
c = astra("top_camera")

def color_detect(frame, image):
	red = [0, 0, 70], [98, 53, 255]
	
	lower = np.array(red[0], dtype="uint8")
	upper = np.array(red[1], dtype="uint8")
	
	mask = cv2.inRange(image, lower, upper)
	
	ret,thresh = cv2.threshold(mask, 40, 255, 0)
	im2,contours,hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
	
	if len(contours) != 0:
		c = max(contours, key = cv2.contourArea)
		
		x,y,w,h = cv2.boundingRect(c)
		
		cv2.rectangle(image,(x,y),(x+w,y+h),(0,255,0),3)
		mid = [(x + w/2), (y + h/2), (frame[(y + h/2), (x + w/2)])/10]
	
	return image, mid
	
def calculation(mid):
	l = 33.5
	cameraH = 60 * math.pi / 180
	cameraV = 49.5 * math.pi / 180
	x0 = mid[0]
	y0 = mid[1]
	z0 = mid[2]
	
	# x1 = z0
	# y1 = y0
	# z1 = x0

	wR = 2 * z0 * math.tan(cameraH / 2)
	hR = 2 * z0 * math.tan(cameraV/2)
	print("Real W, h", wR, hR)
	xR = (wR * x0)/640
	yR = (hR * y0)/480
	
	z = xR - wR/2
	y = 36.5 - (yR - hR/2)
	print("HI:", z, y, l * l - y * y - z * z)
	x = math.sqrt(l * l - y * y - z * z)
	
	return x, y, z
	
while not rospy.is_shutdown():
	frame, image = c.depth_image, c.rgb_image
	
	image, mid = color_detect(frame, image)
	
	x, y, z = calculation(mid)
	
	cv2.imshow("image", image)
	
	print(x, y, z)
	if cv2.waitKey(1) in [ord('q'), 27]:
		break

cv2.destroyAllWindows()