#!/usr/bin/env python
from core import Astra as astra
import numpy as np
import cv2
import rospy


rospy.init_node("home_edu_object_track", anonymous=True)
rate = rospy.Rate(20)

print("ok")
c = astra("top_camera")

def color_detect(image):
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
		mid = [(x + w/2), (y + h/2)]
		
	return mid

def 
while not rospy.is_shutdown():
	frame, image = c.depth_image, c.rgb_image
	
	image, mid = color_detect(image)
	
	cv2.imshow("image", image)
	print(mid)
	
	if cv2.waitKey(1) in [ord('q'), 27]:
		break

cv2.destroyAllWindows()