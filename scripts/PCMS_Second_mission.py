#!/usr/bin/env python

import rospy
import time
import cv2 as cv
import numpy as np
#import FollowMe as follow
from core import Manipulator as mani
from core import Kobuki as kobuki
from core import Speaker as speaker
from core import Speech2Text as speech2text
from core import Astra as astra

def depth_detect(frame):
	image = frame[100:540, 100:380].copy()

	index = image[np.nonzero(image)]
	
	if index is None:
		return False
	else:
		min = np.min(index)
	print(min)

	if min <= 900 and min >= 500:
		return True
	else:
		return False

	#except Exception:
	#print("frame is none")

if __name__ == '__main__':
	rospy.init_node("home_edu_camera", anonymous=True)
	rate = rospy.Rate(20)
	s = speaker()
	print("boot")
	#time.sleep(1)
	#s.say("hello, I'm your assistant")
	c = astra("cam2")
	print("ok")
	while True:
		#frame = c.rgb_image
		image = c.depth_image
		status = depth_detect(image)
		if status == False:
			continue
		else:
			print("ok")
			break
		
	time.sleep(1)
	s.say("please stand in front of me")
	s.say("say follow me for following you")

