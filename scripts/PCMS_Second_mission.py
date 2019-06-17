#!/usr/bin/env python

import rospy
import time
import cv2 as cv
import numpy as np
#import FollowMe as follow
from core import Manipulator as mani
from core import RobotChassis as chassis
from core import Kobuki as kobuki
from core import Speaker as speaker
from core import Speech2Text as speech2text
from core import Astra as astra
#from core import ServiceController as follow
from turtlebot_msgs.srv import SetFollowState
from core import GenderDetection as Gender
from core import Manipulator as manipulator
from core import PH_Follow_me as follow

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
		
def listen_callback(data):
	msg = data

if __name__ == '__main__':
	rospy.init_node("home_edu_PCMS_Second_mission", anonymous=True)
	rate = rospy.Rate(20)
	s = speaker()
	print("started")
	s.say("hello, I'm your assistant")
	
	t = speech2text()
	t.ambient_noise()
	chassis = chassis()
	
	c = astra("cam_top")
	f = follow()
	
	rospy.Subscriber(
		"/home_edu_Listen/msg", 
		String, 
		listen_callback,
		queue_size=1
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
	m = manipulator()
	.
	goal = [[-1.49, 8.48, 0.00247], [7.51, 7.52, -0.00143], [10.6, -3.76, -0.00143]]
	
	while True:
		
		x, y, z = chassis.get_current_pose()
		print(x, y, z)
		print('finished append')
		time.sleep(1)
		s.say("please say the location of where the bag should be put")
		place = t.listen()
		if place == 'kitchen':
			i = 0
		elif place == 'bedroom':
			i = 1
		elif place == 'living room':
			i = 2
		else:
			i = 0
		print(place)
		s.say("please hand me the bag on my robot arm")
		time.sleep(1)
		m.reset()
		m.wait()
		m.exec_servos_pos(10,20,0,0)
		m.wait()
		m.open()
		time.sleep(5)
		m.close()
		time.sleep(1)
		s.say("girpped, i am now goin to the location")
		print(goal[i][0], goal[i][1], goal[i][2])
		chassis.move_to(goal[i][0], goal[i][1], goal[i][2])
		s.say("arrived to goal")
		m.open()
		s.say("please take the bag")
		break

	k = kobuki()
	g = Gender("/home/mustar/pcms/src/home_edu/scripts/libs/deploy_gender.prototxt", "/home/mustar/pcms/src/home_edu/scripts/libs/gender_net.caffemodel")
	
	cam = astra("camera")
	
	while not rospy.is_shutdown():
		frame = cam.rgb_image
		position = g.detect_face(frame)
		if len(position) == 0:
			print('moving')
			k.move(0, 0.3)
			cv.imshow("frame", frame)
		else:
			p1 = position[0]
			x_val = (p1[2] - p1[0]) / 2 + p1[0]
			print(x_val, frame.shape)
			if x_val < 300:
				k.move(0, 0.3)
			elif x_val > 340:
				k.move(0, -0.3)
			else:
				time.sleep(1)
				s.say("please follow me to the garage")
				chassis.move_to(x, y, z)
				s.say("arrived to the garage")
				break
		'''
			for x1, y1, x2, y2 in position:
				x_val = (x2 - x1) / 2 + x1
				cv.imshow('frame', frame)
				if x_val >= 360:
					k.move(0, 0.2)
				elif x_val <= 280:
					k.move(0, 0.2)
				elif 360 < x_val > 280:
					k.move(0, 0)
					time.sleep(5)
					break
		'''
		cv.waitKey(1)
	print("end of program")


