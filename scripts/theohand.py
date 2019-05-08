#!/usr/bin/env python
import rospy
import math
from core import Dynamixel
import numpy as np
import sys, termios, tty, os, time
ARM_LENGTH = 10.5
MAX_SPEED = 1.2

L1 = ARM_LENGTH
L2 = ARM_LENGTH


def go_to_coordinate(x,y):
	if (x*x + y*y < 10.5*10.5):
		print("OK LA")
	
	d = math.sqrt(x*x + y*y)
	theta1 = math.acos((L1*L1+d*d-L2*L2)/(2*L1*d))
	rotate_a = math.pi / 2 - math.atan2(y,x) - theta1
	theta2 = math.acos((L1*L1 + L2*L2 - d*d)/(2*L1*L2))
	rotate_b = math.pi - theta2
	shoulder.set_radian(rotate_a)
	elbow.set_radian(rotate_b)

def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
 
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

if __name__ == "__main__":
	button_delay = 0.1
	rospy.init_node("home_edu_arm", anonymous=True)
	shoulder = Dynamixel("shoulder_controller")
	
	elbow = Dynamixel("elbow_controller")

	shoulder.set_speed(MAX_SPEED)
	go_to_coordinate(0,21)
	
	elbow.set_speed(MAX_SPEED)

