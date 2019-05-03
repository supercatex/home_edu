#!/usr/bin/env python
import rospy
from Dynamixel import Dynamixel
import math
import numpy as np
import time

def initialize():
	alpha1 = 0
	alpha2 = 0
	return alpha1, alpha2

def calc_radian(x, y):
	alpha1 = 0.9
	alpha2 = 0.5
	return alpha1, alpha2

if __name__ == "__main__":
	rospy.init_node("home_edu_arm", anonymous=True)
	
	MAX_SPEED = 1.2

	servos = []
	servos.append(Dynamixel("shoulder_controller"))
	servos.append(Dynamixel("elbow_controller"))

	for i in range(len(servos)):
		servos[i].set_speed(MAX_SPEED)

	alpha1, alpha2 = initialize()
	
	servos[0].set_radian(alpha1)

	servos[1].set_radian(alpha2)
	
	time.sleep(1)

	alpha1, alpha2 = calc_radian(0, 0)

	servos[0].set_radian(alpha1)

	servos[1].set_radian(alpha2)
	