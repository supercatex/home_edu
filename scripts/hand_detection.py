#!/usr/bin/env python

import rospy
import cv2 as cv
from std_msgs.msg import String
from core import Astra as astra
import numpy as np

rospy.init_node("home_edu_waving_hand", anonymous=True)

rate = rospy.Rate(20)

c = astra("top_camera")


while not rospy.is_shutdown():
    frame = c.rgb_image
    ycc = cv.cvtColor(frame, cv.COLOR_BGR2YCR_CB)

    min_ycc = np.array([0, 133, 85], np.uint8)
    max_ycc = np.array([255, 170, 125], np.uint8)
    skin = cv.inRange(ycc, min_ycc, max_ycc)

    opening = cv.morphologyEx(skin, cv.MORPH_OPEN, np.ones((5, 5), np.uint8), iterations=3)
    sure_bg = cv.dilate(opening, np.ones((3, 3), np.uint8), iterations=2)

        im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
		
		for cnt in contours:

            x, y, w, h = cv2.boundingRect(cnt)
    
            cv2.rectangle(rgb_image, (x, y), (x + w, y + h), (0, 255, 0), 3)
    
            e = 0
    
            y = (y + h / 2)
    
            x = (x + w / 2)
    
            cv2.circle(rgb_image, (x, y), 5, (0, 255, 255), -1)
            h2, w2 = depth_image.shape
    
            real_z = 0
			
			cv2.rectangle(rgb_image, (x, y), (0 + 640, 0 + Y), (0, 255, 0), 3)
			frame = frame[0:640, 0:y].copy()
			
			
			x, y, c = image.shape()
			
			area1 = 100
			area2 = 40
			
			if area2 - area1 < 60:
				print("found hand")
			else:
				pass
        cv.imshow('frame', frame)
    if cv.waitKey(1) in [ord('q'), 27]:
        break
    
    rate.sleep()
        
cv.destroyAllWindows()
