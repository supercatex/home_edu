#!/usr/bin/env python

import rospy
import time
import cv2 as cv
import numpy as np
#import FollowMe as follow
from std_msgs.msg import String
from core import RobotChassis as chassis
from core import Kobuki as kobuki
from core import Speaker as speaker
from core import Speech2Text as speech2text
from core import Astra as astra
#from core import ServiceController as follow
from turtlebot_msgs.srv import SetFollowState
from core import GenderDetection as Gender
from core import Manipulator as manipulator
from core import PH_Follow_me as PH_Follow_me
from manipulator_track import manipulator_track as manipulator_track

publisher = rospy.Publisher(
	"/home_edu/facial",
	String,
	queue_size=1,
	latch=True
)
	
def depth_detect(frame):
	try:
		image = frame[100:540, 100:380].copy()

		index = image[np.nonzero(image)]
		
		if index is None:
			return False
		else:
			min = np.min(index)

		print(min)

		if min <= 750 and min >= 350:
			return True
		else:
			return False

	except Exception:
		print("error")
		
def listen_callback(data):
	global msg
	msg = data.data
	print(msg)
	


if __name__ == '__main__':
	msg = ' '
	rospy.init_node("home_edu_PCMS_Second_mission", anonymous=True)
	rate = rospy.Rate(20)
	s = speaker()
	
	rospy.Subscriber(
		"/home_edu_Listen/msg", 
		String, 
		listen_callback,
		queue_size=1
	)
	t = speech2text()
	t.ambient_noise()
	chassis = chassis()
	
	c = astra("top_camera")
	f = PH_Follow_me()
	m = manipulator()
	k = kobuki()
	obj = manipulator_track("red")
	m.exec_servos_pos(10,15,0,-30)
	print("started")
	s.say("hello, I'm your assistant", "happy-1")
	_listen_publisher = rospy.Publisher("/home_edu_Listen/situation", String, queue_size=1)
	while True:
		#frame = c.rgb_image
		status = depth_detect(c.depth_image)
		if status == True:
			print("ok")
			break
	
	time.sleep(1)
	s.say("Please stand in front of me", "happy-1")
	s.say("please stand still and say follow me")
	
	_listen_publisher.publish("true")
	
	goal = [[-3.36, 8.17, 0.0025], [1.9, 5.51, -0.00137], [0.00489, -0.0209, -0.00137]]
	
	flag = 0
	
	while True:
		if msg == 'follow me':
			flag = 1
		elif msg == 'stop':
			flag = 2
		else:
			pass

		forward_speed, turn_speed = f.follow(c.depth_image, flag==1)
		k.move(forward_speed, turn_speed)
		
		if flag == 2:
			break
	_listen_publisher.publish("false")
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
	s.say("you said " + place)
	time.sleep(1)
	m.wait()
	s.say("please start the bag gripping task")

	signal = True
	
	m.open()
	while not rospy.is_shutdown():
	
		print(obj.area)
		frame, image = c.depth_image, c.rgb_image

		image, x, y, z, alpha = obj.run(c.rgb_image, c.depth_image)
		
		if signal == True:
			if obj.area < 1500 or obj.area is None:
				signal = False
				start_time = time.time()
				continue 
			else:
				m.exec_servos_pos(x, y, z, -60)
				cv.imshow("image", image)
				print('mani x, y, z:', x, y, z, -60)
				continue
		else:
			if obj.area < 1500 or obj.area is None:
				if time.time() - start_time > 3: 
					break
				else:
					continue
			else:
				signal = True 
				continue

		if cv.waitKey(1) in [ord('q'), 27]:
			break
	
	print("end simulate")

	time.sleep(1)
	
	m.close()
	
	m.exec_servos_pos(15,25,0,-60)
	
	s.say("gripped, i am now goin to the location")
	s.say("Please stand away from me", "wink")
	print(goal[i][0], goal[i][1], goal[i][2])
	chassis.move_to(goal[i][0], goal[i][1], goal[i][2])
	s.say("arrived to goal")
	m.open()
	s.say("please take the bag")


	g = Gender("/home/mustar/pcms/src/home_edu/scripts/libs/deploy_gender.prototxt", "/home/mustar/pcms/src/home_edu/scripts/libs/gender_net.caffemodel")
	
	cam = astra("top_camera")
	
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
				s.say("please follow me to the garage and stand behind me", "wink")
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
	s.say("finished task", "wink")


