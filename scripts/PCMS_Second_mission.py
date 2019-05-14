#!/usr/bin/env python

import rospy
import time
#import FollowMe as follow
from core import Manipulator as mani
from core import Kobuki as kobuki
from core import Speaker as speaker
from core import Speech2Text as speech2text
from core import Astra as astra

def depth_detect(frame):
	array()
	try:
		'''
		image = frame[400 : 880, 400 : 560]

		for i in range len(image):
			array.append(i)
		for b in range len(array):
			if b == 0:
				return False
			elif b <= 1000 and b >= 500:
				return True
			else:
				return False
		'''
		depth = frame[1280/2, 960/2]
		print(depth)
	except EOFError:
		print("frame is none")
		
	return False

if __name__ == '__main__':
	rospy.init_node("home_edu_camera", anonymous=True)
	rate = rospy.Rate(20)
	s = speaker()
	print("boot")
	#time.sleep(1)
	#s.say("hello, I'm your assistant")
	c = astra("cam1")
	print("ok")
	while True:
		#frame = c.rgb_image
		image = c.depth_image
		image = cv.resize(image, (1280, 960))
		status = depth_detect(image)
		if status == False:
			continue
		else:
			pass
		
	time.sleep(1)
	s.say("please stand in front of me")
	s.say("say follow me for following you")

