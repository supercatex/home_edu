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
from core import ServiceController as follow
from turtlebot_msgs.srv import SetFollowState

def depth_detect(frame):
	try:
		image = frame[100:540, 100:380].copy()

		index = image[np.nonzero(image)]
		
		if index is None:
			return False
		else:
			min = np.min(index)

		print(min)

		if min <= 800 and min >= 500:
			return True
		else:
			return False

	except Exception:
		print("error")

if __name__ == '__main__':
	rospy.init_node("home_edu_camera", anonymous=True)
	rate = rospy.Rate(20)
	s = speaker()
	print("started")
	s.say("hello, I'm your assistant")
	c = astra("camera")
	t = speech2text()
	t.ambient_noise()
	f = follow()
	
	f.register(
		"follower",
		"/turtlebot_follower/change_state",
		SetFollowState
	)
	
	while True:
		#frame = c.rgb_image
		image = c.depth_image
		status = depth_detect(image)
		if status == True:
			print("ok")
			break
		else:
			continue
	
	time.sleep(1)
	s.say("please stand still and say follow me")

	while True:
		print("start listening...")
		cmd = t.listen()
		print("cmd:", cmd)
		
		if cmd == 'follow me':
			s.say("please walk slowly")
			f.send_message("follower", 1)
			print("after follower 1")
		elif cmd == 'stop':
			s.say("stop")
			f.send_message("follower", 0)
			print("after follower 0")
		rate.sleep()

	print("end of program")


