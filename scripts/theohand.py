#!/usr/bin/env python
import rospy
from .core import Manipulator
import math
from .core import Dynamixel
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

class M3(Manipulator):
	l1 = 10.5
	l2 = 10.5
	l3 = 12.6
	MAX_SPEED = 1
	def __init__ (self):
		super(M3, self).__init__()
		print('M3')
		
	def reset(self):
		self.exec_servos_pos(0, M3.l1 + M3.l2 + M3.l3, 0, -90)
		self.close()
		self.wait()
		print("Manipulator is ready.")

 	def exec_servos_pos(self, x, y, z, w):
		if math.sqrt(x * x + y * y + z * z) > M3.l2 * 2:
			print("Cannot move to point(%.2f, %.2f, %.2f)" % (x, y, z))
			return
		
		if self.target_x == x and self.target_y == y and self.target_z == z and self.target_w == w:
			print("Same poisition point(%.2f, %.2f, %.2f)" % (x, y, z))
			return
		alpha = [0, 0, 0, 0]
		x = float(x)
		y = float(y)
		z = float(z)
		w = float(w)
		pi = math.pi
		x = math.sqrt(x**2 + z**2)
		r = w * pi / 180
		D = x**2 + y**2
		a = math.atan2(y, x)
		c = a + r
		l4 = math.sqrt(M3.l3**2 + D - (2 * M3.l3 * math.sqrt(D) * math.cos(c)))
		f = math.acos((M3.l1**2 + l4**2 - M3.l2**2) / (2 * l4 * M3.l1))
		e = math.acos((D + l4**2 - M3.l3**2) / (2 * math.sqrt(D) * l4))
		shoulder_angle = float(90 - (a * 180 / pi) - (f * 180 / pi) - (e * 180 / pi))
		g = math.acos((M3.l1**2 + M3.l2**2 - l4**2) / (2 * M3.l1 * M3.l2))
		elbow_angle = float(180 - (g * 180 / pi))
		h = 180 - (g * 180 / pi) - (f * 180 / pi)
		d = 180 - (e * 180 / pi) - (c * 180 / pi)
		wrist_angle = float(180 - h - d)
		waist_angle = float((math.atan2(z, x)) * 180 / pi)
		alpha[0] = waist_angle * pi / 180
		alpha[1] = shoulder_angle * pi / 180
		alpha[2] = elbow_angle * pi / 180
		alpha[3] = wrist_angle * pi / 180
		delta_list = []
		for i in range(len(alpha)):
			delta_list.append(abs(alpha[i] - self.servos[i].state.current_pos))
		max_delta = np.max(delta_list)
		
		for i in range(len(alpha)):
			self.servos[i].set_speed(M3.MAX_SPEED * delta_list[i] / max_delta)
			self.servos[i].set_radian(alpha[i])

		print(x, y, z, w)
		print(alpha)

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
	
	elbow.set_speed(MAX_SPEED)
	m3 = M3()
	m3.reset()
	x = 0
	y = 0
	
	char = getch()
 	while(1):
		if (char == "o"):

			go_to_coordinate(x+1,y)
			time.sleep(button_delay)
 
		if (char == "p"):
			go_to_coordinate(x,y+1)
			time.sleep(button_delay)
 
		elif (char == "k"):
			go_to_coordinate(x-1,y)
			time.sleep(button_delay)
 
		elif (char == "l"):
			go_to_coordinate(x,y-1)
			time.sleep(button_delay)
 
		elif (char == "s"):
			print("Down pressed")
			time.sleep(button_delay)
 
		elif (char == "1"):
			print("Number 1 pressed")
			time.sleep(button_delay)
	m3.exec_servos_pos(20, 15, -10, 0)
	m3.wait()

