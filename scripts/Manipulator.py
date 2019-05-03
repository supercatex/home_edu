#!/usr/bin/env python
import rospy
from Dynamixel import Dynamixel
import math
import numpy as np
import time


class Manipulator(object):

	ARM_LENGTH = 10.5
	MAX_SPEED = 1.0
	
	def __init__(self):
		self.servos = []
		self.servos.append(Dynamixel("waist_controller"))
		self.servos.append(Dynamixel("shoulder_controller"))
		self.servos.append(Dynamixel("elbow_controller"))
		self.servos.append(Dynamixel("wrist_controller"))
		self.servos.append(Dynamixel("hand_controller"))
		for servo in self.servos:
			servo.set_speed(Manipulator.MAX_SPEED)
		
		self.target_x = 0
		self.target_y = 0
		self.target_z = 0
		self.target_w = 0
		self.exec_servos_pos(0, Manipulator.ARM_LENGTH * 2, 0, -90)
		self.close()
		self.wait()
		print("Manipulator is ready.")
	
	def exec_servos_pos(self, x, y, z, w=0):
		if math.sqrt(x * x + y * y + z * z) > Manipulator.ARM_LENGTH * 2:
			print("Cannot move to point(%.2f, %.2f, %.2f)" % (x, y, z))
			return
		
		if self.target_x == x and self.target_y == y and self.target_z == z and self.target_w == w:
			print("Same poisition point(%.2f, %.2f, %.2f)" % (x, y, z))
			return
		
		alpha = [0, 0, 0, 0]
		x = math.sqrt(x * x + z * z)
		if x != 0:
			alpha[0] = math.asin(z / x)
		
		L1 = Manipulator.ARM_LENGTH
		L2 = Manipulator.ARM_LENGTH
		L1_2 = L1 * L1
		L2_2 = L2 * L2
		L3_2 = x * x + y * y
		
		theta1 = 0
		if L3_2 != 0:
			theta1 = math.acos((L1_2 + L3_2 - L2_2) / (2 * L1 * math.sqrt(L3_2)))
		alpha[1] = math.pi / 2 - theta1 - math.atan2(y, x)
		
		theta2 = 0
		theta2 = math.acos((L1_2 + L2_2 - L3_2) / (2 * L1 * L2))
		alpha[2] = math.pi - theta2
		
		alpha[3] = math.pi / 2 - alpha[1] - alpha[2] + w * math.pi / 180
		
		delta_list = []
		for i in range(len(alpha)):
			delta_list.append(abs(alpha[i] - self.servos[i].state.current_pos))
		max_delta = np.max(delta_list)
		
		for i in range(len(alpha)):
			self.servos[i].set_speed(Manipulator.MAX_SPEED * delta_list[i] / max_delta)
			self.servos[i].set_radian(alpha[i])
		self.target_x = x
		self.target_y = y
		self.target_z = z
		self.target_w = w
	
	def is_moving(self):
		for servo in self.servos:
			if servo.state.is_moving:
				return True
		return False
	
	def wait(self):
		time.sleep(1)
		while self.is_moving(): pass
	
	def open(self):
		self.servos[4].set_radian(-math.pi / 180 * 25)
	
	def close(self, offset=0):
		self.servos[4].set_radian(math.pi / 180 * max(39 - abs(offset), 0))


if __name__ == "__main__":
	rospy.init_node("home_edu_arm", anonymous=True)
	manipulator = Manipulator()
	
	manipulator.exec_servos_pos(10, 5, 0, 45)
	manipulator.wait()
	
	manipulator.exec_servos_pos(10, 5, 0, -45)
	manipulator.wait()
	
	manipulator.exec_servos_pos(10, 5, 0, 0)
	manipulator.wait()
	
	manipulator.exec_servos_pos(10, 5, -8, 0)
	manipulator.wait()
	
	manipulator.exec_servos_pos(10, 5, 8, 0)
	manipulator.wait()
	
	manipulator.exec_servos_pos(10, 5, 0, 0)
	manipulator.wait()
	
	manipulator.open()
	manipulator.wait()
	
	manipulator.close(20)
	manipulator.wait()
	
	manipulator.open()
	manipulator.wait()
	
	manipulator.close(0)
	manipulator.wait()
